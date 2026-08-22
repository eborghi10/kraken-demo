# Mission planning

*[Docs index](index.md) · [Navigation](navigation.md) · [Control](control.md)*

The job is not a pose to reach, it is a field to finish. That makes coverage a
**navigator**, not a script: `kraken_nav::CoverRowsNavigator` is a third
`bt_navigator` plugin alongside the two stock ones, answering an action of its
own. Nav2's own documentation names this case:

> It may be beneficial to write your own Navigator if you have a custom action
> message definition you'd like to use with Navigation rather than the provided
> `NavigateToPose` or `NavigateThroughPoses` interfaces (**e.g. doing complete
> coverage**).
>
> — *Nav2, Writing a New Navigator Plugin*

The alternative — a node above the stack sending `NavigateToPose` goals row by
row — brings the machine to a halt at the end of every row, because Nav2 stops
at a goal. That cost is what §4 below removes.

---

## 1. The action

`kraken_interfaces/action/CoverRows`:

```
uint16 aisles          float32 aisle_pitch    uint16 aisle_skip
float32 row_near_x     float32 row_far_x      float32 row_heading_deg
---
uint16 covered         uint16[] missed        builtin_interfaces/Duration total_time
uint16 error_code      string error_msg
---
uint16 current_aisle   uint16 legs_done       uint16 legs_total
float32 distance_remaining                    builtin_interfaces/Duration navigation_time
```

The field is described the way an orchard is laid out — a count of aisles and
their spacing — rather than as a list of poses, and pinned to the machine's
actual position at the moment of the goal. See
[the row frame](navigation.md#the-row-frame) for the transform that does the
pinning.

> **Trap.** `nav2_behavior_tree::BtActionServer<ActionT>::populateErrorCode`
> dereferences `result->error_code`. Any custom navigator action **must** carry
> a `uint16 error_code` field or the template will not compile, with an error
> that points nowhere useful.

---

## 2. The navigator plugin

`kraken_nav::CoverRowsNavigator` derives from
`nav2_core::BehaviorTreeNavigator<CoverRows>`. It owns the mission state and
nothing else:

- `goalReceived` — validates, anchors the row frame at the current pose, builds
  the leg list, and pushes it onto the blackboard.
- `onLoop` — publishes feedback.
- `onPreempt` — **refuses.** A coverage run has no meaningful "same goal,
  slightly different" preemption; changing the field mid-run silently would be
  worse than making the operator cancel.
- `goalCompleted` — fills in `covered` and `missed`.

`on_configure` and friends are `final` in the base class and must not be
overridden.

> **Trap.** The framework guarantees only `node`, `server_timeout`,
> `bt_loop_duration` and `cancel_timeout` on the blackboard. `tf_buffer`,
> `global_frame` and `robot_base_frame` are *not* set — the navigator sets them
> in `goalReceived` so the custom BT nodes can read TF.

---

## 3. The tree

Three custom BT nodes, because only three had no stock equivalent:

| Node | Kind | Does |
| --- | --- | --- |
| `NextLeg` | action | Hands out the next aisle; **fails when the field is finished**, which is how the loop exits. |
| `MissedLeg` | action | Records the current aisle as uncovered. |
| `TurnDue` | condition | True once per leg, when the row end is within 5 m. |

Everything else is Nav2's: `ComputePathToPose`, `GetPoseFromPath`,
`ConcatenatePaths`, `TruncatePath`, `FollowPath`, `PipelineSequence`,
`RecoveryNode`, `ClearEntireCostmap`, `BackUp`.

```
ForceSuccess
└── KeepRunningUntilFailure
    └── Sequence  "leg"
        ├── NextLeg  → row_goal, turn_goal, has_turn, aisle
        └── Fallback  "drive_or_write_off"
            ├── Sequence  "drive_the_leg"
            │   ├── ComputePathToPose  goal=row_goal  planner=GridBased  → row_path
            │   ├── TruncatePath  row_path → path   (distance=0.0, a stock copy)
            │   └── PipelineSequence  "row_then_turn"
            │       ├── Fallback
            │       │   ├── Inverter → TurnDue(path, 5.0, has_turn)
            │       │   └── Sequence  "plan_the_turn"
            │       │       ├── GetPoseFromPath  row_path, index=-1  → row_end
            │       │       ├── Fallback
            │       │       │   ├── ComputePathToPose  start=row_end  planner=Headland
            │       │       │   └── ComputePathToPose  start=row_end  planner=GridBased
            │       │       └── ConcatenatePaths  row_path + turn_path → path
            │       └── FollowPath  path
            └── Sequence  "write_off_and_recover"
                ├── MissedLeg
                ├── ClearEntireCostmap  local
                ├── ClearEntireCostmap  global
                └── ForceSuccess → BackUp 2.5 m
```

---

## 4. Why the machine never stops between rows

`PipelineSequence` ticks its children in order and keeps ticking earlier ones
while a later one is RUNNING. So `FollowPath` starts driving the row
immediately; five metres from the row end `TurnDue` fires; the headland turn is
planned from the row's *last pose* and **concatenated onto the same path**; and
`FollowPath` — which has a port on `{path}` — picks up the extended path on its
next tick without ever being halted. There is no new goal and no stop.

`ReactiveSequence` would be the intuitive choice and is wrong: BT.CPP 4 throws
if more than one child of a `ReactiveSequence` returns RUNNING. `PipelineSequence`
is what Nav2's own
`navigate_to_pose_w_replanning_and_recovery.xml` uses for exactly this shape.

`SingleTrigger` was evaluated for the once-per-leg behaviour and rejected: a
`Fallback` resets its children when it returns SUCCESS, which rearms the trigger
on every tick. Hence `TurnDue` latches internally on the leg index.

---

## 5. The two `ForceSuccess` decorators

They are not the same thing and only one of them survived.

**The outer one is correct and stays.** `NextLeg` returns FAILURE when the
aisles run out — that *is* the loop's exit condition, and
`KeepRunningUntilFailure` propagates it. Without `ForceSuccess` the action would
report ABORTED at the end of a mission that completed perfectly. It converts
"finished" into "succeeded", which is exactly what it means.

**The inner one was a bug and is gone.** It wrapped `plan_the_turn`, with the
intent "if no headland turn fits, don't lose the aisle — finish the row and let
the next leg's search planner find its own way round". What it actually did was
*destroy the information that no turn had been planned*. The tree then behaved
identically in both cases: `{path}` stayed the bare row, the machine drove to
the row end and stopped, and the leg reported success. The next leg then asked
Smac to plan out of a pose the geometric planner had just declared impossible to
turn from. Smac obliged — it checks the costmap, not the vehicle's swept
manoeuvre — the tracker refused to drive the result, and the run wedged for
**six consecutive legs**. One unfittable turn cost six aisles.

The replacement is a `Fallback` between two planners rather than a decorator
that hides the answer:

```xml
<Fallback>
  <ComputePathToPose ... planner_id="Headland"/>
  <ComputePathToPose ... planner_id="GridBased"/>
</Fallback>
```

Geometry first, because it answers in well under a millisecond and lands the row
centre. Where geometry will not fit, the search planner gets the **same start
and the same goal** and is free to reverse. Either way the result is
concatenated and the machine keeps moving. Only if *both* fail does the branch
fail — and then it fails loudly into the write-off branch, one leg lost instead
of six.

The general lesson: **`ForceSuccess` is right when failure is a valid terminal
state, and wrong when failure is information the rest of the tree needs.**

---

## 6. What went wrong here

The write-off branch above is the recovery for a leg that could not be driven;
whether it is enough to escape a genuinely blocked headland is
[the open problem](navigation.md#the-open-problem). The once-per-leg latch on
`TurnDue` is bug 5 of the
[five bugs only a real simulator finds](navigation.md#9-bugs-only-a-real-simulator-finds).
