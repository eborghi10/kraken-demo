# SPDX-License-Identifier: BSD-3-Clause
"""Run one scenario repeatedly and report the distribution of its metrics.

A single run is not reproducible. The simulator advances on a wall-clock timer
while the filter, fault injector and scorer consume its messages over DDS, so
the interleaving - and with it the result - changes from run to run. Measured on
total_gnss_dropout, worst_position_error moved over a 3x range across eight
runs. Quote a distribution from this tool, never a number from a single run.

    ros2 run kraken_scenarios sweep total_gnss_dropout -n 10

Varying the seed as well as the schedule is the default, so the summary answers
"what does this scenario do", not "what did seed 0 do once". Pass --fixed-seed
to hold the noise still and isolate the scheduling nondeterminism on its own.
"""
import argparse
import json
import os
import signal
import statistics
import subprocess
import sys
import tempfile
import time

METRICS = (
    'worst_position_error',
    'worst_heading_error_deg',
    'position_error',
    'heading_error_deg',
    'path_length',
    'path_rotation_deg',
)


def _read_report(path, attempts=5):
    """Read the report, tolerating the instant between create and write."""
    for _ in range(attempts):
        try:
            with open(path, 'r') as handle:
                return json.load(handle)
        except (json.JSONDecodeError, ValueError):
            time.sleep(0.5)
    return None


def one_run(scenario, seed, report, simulator, timeout):
    """Launch the stack once and return its report, or None if it never landed.

    `ros2 launch` does not exit when the runner finishes - the background nodes
    hold it open and it ignores SIGINT - so it goes into its own process group
    and the group is killed once the report appears.
    """
    if os.path.exists(report):
        os.remove(report)

    proc = subprocess.Popen(
        ['ros2', 'launch', 'kraken_scenarios', 'scenario.launch.py',
         'scenario:=%s' % scenario, 'report:=%s' % report,
         'simulator:=%s' % simulator, 'seed:=%d' % seed],
        stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL,
        start_new_session=True)
    try:
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            if os.path.exists(report):
                return _read_report(report)
            if proc.poll() is not None:
                return None
            time.sleep(0.5)
        return None
    finally:
        try:
            os.killpg(proc.pid, signal.SIGKILL)
        except (ProcessLookupError, PermissionError):
            pass
        proc.wait()


def summarise(reports):
    stats = {}
    for metric in METRICS:
        values = [r[metric] for r in reports if metric in r]
        if not values:
            continue
        stats[metric] = {
            'n': len(values),
            'mean': statistics.mean(values),
            'sd': statistics.stdev(values) if len(values) > 1 else 0.0,
            'min': min(values),
            'max': max(values),
            'median': statistics.median(values),
        }
    return stats


def main(argv=None):
    parser = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('scenario')
    parser.add_argument('-n', '--runs', type=int, default=10)
    parser.add_argument('--seed-base', type=int, default=0)
    parser.add_argument('--fixed-seed', action='store_true',
                        help='hold the seed at --seed-base to isolate scheduling jitter')
    parser.add_argument('--simulator', choices=('headless', 'o3de'), default='headless')
    parser.add_argument('--timeout', type=float, default=120.0,
                        help='seconds to wait for one run to produce a report')
    parser.add_argument('-o', '--output', help='write the aggregate JSON here')
    args = parser.parse_args(argv)

    reports = []
    failures = []
    workdir = tempfile.mkdtemp(prefix='kraken_sweep_')
    for i in range(args.runs):
        seed = args.seed_base if args.fixed_seed else args.seed_base + i
        report = os.path.join(workdir, 'run_%02d.json' % i)
        result = one_run(args.scenario, seed, report, args.simulator, args.timeout)
        if result is None:
            failures.append(i)
            print('run %2d  seed %-4d  FAILED' % (i, seed), flush=True)
            continue
        result['seed'] = seed
        reports.append(result)
        print('run %2d  seed %-4d  worst_position_error %8.4f  path_length %8.3f'
              % (i, seed, result['worst_position_error'], result['path_length']),
              flush=True)
        # Settling time between runs; DDS discovery is not instant.
        time.sleep(2.0)

    if not reports:
        print('every run failed', file=sys.stderr)
        return 1

    stats = summarise(reports)
    print('\n%-24s %8s %8s %8s %8s %8s' % ('metric', 'mean', 'sd', 'min', 'max', 'median'))
    for metric, s in stats.items():
        print('%-24s %8.4f %8.4f %8.4f %8.4f %8.4f'
              % (metric, s['mean'], s['sd'], s['min'], s['max'], s['median']))
    print('\n%d/%d runs completed%s'
          % (len(reports), args.runs,
             '' if not failures else ', failed: %s' % failures))

    aggregate = {
        'scenario': args.scenario,
        'simulator': args.simulator,
        'runs_requested': args.runs,
        'runs_completed': len(reports),
        'fixed_seed': args.fixed_seed,
        'seed_base': args.seed_base,
        'stats': stats,
        'runs': reports,
    }
    if args.output:
        with open(args.output, 'w') as handle:
            json.dump(aggregate, handle, indent=2, sort_keys=True)
        print('wrote %s' % args.output)
    return 0


if __name__ == '__main__':
    sys.exit(main())
