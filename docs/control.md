# Control

*[Docs index](index.md) · [Navigation](navigation.md) · [Mission planning](mission-planning.md)*

How the machine follows the path it was given. `kraken_nav::ArcTracker` is a
`nav2_core::Controller`. It does not do pure pursuit. It tracks the *curvature
the planner asked for* and trims it.

Implemented in `ros2_ws/src/kraken_nav/src/arc_tracker.cpp`, configured under
`FollowPath` in `config/nav2.yaml`. It is the controller for every path the
stack produces, whether the path came from Smac, from the
[headland planner](navigation.md#5-turning-at-the-headland), or from a plain
`NavigateToPose` goal in the localisation scenarios.

---

## 1. Digesting the plan

A `nav_msgs/Path` is a list of poses, which is a lossy way to describe arcs.
Smac emits poses about 0.4 m apart with per-pose orientation noise, and naive
differencing of that reads a dead-straight 44 m row as *"two direction changes,
tightest radius 1.47 m"*. Two filters fix it:

- **Run-length filter.** A forward/reverse run shorter than `min_segment`
  (1.0 m) is absorbed into its neighbour. Direction changes are real events;
  they do not happen twice in 40 cm.
- **Sample-counted curvature window.** Curvature is measured over a window that
  terminates when it has *both* enough samples (`curvature_samples`, 4) *and*
  enough distance (`curvature_window`, 0.15 m). Counting only distance smeared
  the bulb turn's two opposite-sign arcs together and cost 0.75 m of tracking
  error.

Curvature below `straight_curvature` (0.02 m⁻¹) is snapped to zero.

---

## 2. The control law

With cross-track error $e$ (positive left of the path) and heading error
$\psi$, curvature limit $\Lambda = 1/R_{\min}$, and correction share
$\alpha = 0.3$:

$$
\tilde\psi = \begin{cases} \psi & \text{forward} \\ -\psi & \text{reverse} \end{cases}
$$

$$
\text{trim} = \operatorname{clamp}\Big(-\big(k_\psi\,\tilde\psi + k_e\,e\big),\ -\alpha\Lambda,\ +\alpha\Lambda\Big),
\qquad k_\psi = 0.8,\ k_e = 0.6
$$

$$
\kappa_{\text{cmd}} = \operatorname{clamp}\big(\kappa_{\text{ref}} + \text{trim},\ -\Lambda,\ +\Lambda\big)
$$

Two things deserve comment.

**The reverse sign flip is on the heading term, not the cross-track term.** When
reversing, steering right still moves the front of the machine right, but the
*path* progresses backwards, so the heading error's sense inverts while the
cross-track error's does not. Flipping the wrong one produced a divergent
0.77 m error that looked exactly like a gain problem and was not.

**The trim is clamped to a share of the limit, not to the limit.** The reference
curvature is allowed most of the steering; corrections get 30%. Combined with
the [roomy-radius rule](navigation.md#why-not-the-tightest-radius), the machine
always has authority left to correct with.

---

## 3. Speed

$$
v = \begin{cases}
v_{\text{turn}} = 0.4\ \text{m/s} & |\kappa_{\text{ref}}| > \kappa_{\text{straight}} \\
v_{\text{row}} = 0.8\ \text{m/s} & \text{otherwise}
\end{cases}
$$

then limited so the machine can always stop in the distance remaining — both to
the end of the path and to the next cusp (direction reversal):

$$
v \;\le\; \sqrt{2\,a_{\max}\,d_{\text{end}}},
\qquad
v \;\le\; \sqrt{2\,a_{\max}\,d_{\text{cusp}}},
\qquad a_{\max} = 0.4\ \text{m/s}^2
$$

finally signed by the current segment's direction and rate-limited by
$a_{\max}$.

---

## 4. Refusing to drive

Before each command the tracker samples the path ahead over
`collision_lookahead` (2.0 m), one pose per costmap cell, and vetoes on cost
**exactly 254**. Not 253. Cost 253 is `INSCRIBED_INFLATED_OBSTACLE` — inflation,
not an obstacle. Vetoing on 253 in an orchard, where the aisle is barely wider
than the inflation radius on both sides, aborts every single leg. This was the
first bug and it looked like a planner failure for a long time.

Against the headless simulator there is nothing in the costmap to veto on, so
this check only does work on the O3DE side, where the lidar populates the
obstacle layer. See [simulation](simulation.md#3-which-one-to-use) for which
sensors exist where.

---

## 5. From curvature to steering

The controller emits `Twist`, and the curvature clamp $\Lambda = 1/R_{\min}$ is
where the vehicle's kinematics enter the control loop: nothing the tracker
commands is tighter than the machine can steer.

What consumes that `Twist` differs by simulator. The headless model integrates
it directly as a unicycle. The O3DE Kraken subscribes to
`ackermann_msgs/AckermannDrive`, so `kraken_sim/ackermann_bridge` inverts the
bicycle relation for it,

$$
\delta = \arctan\!\big(L\,\omega_z / v\big),
$$

clamped to $\pm 0.7$ rad, and relays a turn on the spot as a stop rather than as
a guess.

Telling the controller the machine is a differential drive when it is not is not
a cosmetic error. It made the controller plan for a robot that could pivot, and
[`nav_terrain_dropout`](localisation.md#4-the-robustness-suite) reported success in
only 5 runs of 8 until that was corrected — after which it succeeded in all 8,
still 5.5 m from the goal, which is the worse and more interesting result.

---

## 6. What went wrong here

Three of the [five bugs only a real simulator
finds](navigation.md#9-bugs-only-a-real-simulator-finds) live in this file:
vetoing on cost 253, flipping the wrong term in reverse, and reading arcs out of
a pose list.
