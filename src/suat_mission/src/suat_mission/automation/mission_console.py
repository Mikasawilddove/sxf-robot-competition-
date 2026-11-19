#!/usr/bin/env python3
"""Prepare one scoped run, accept one fresh terminal Enter, then print its report."""
import argparse
import json
import math
import os
import sys
import threading
import time
import uuid


def discard_pending_input():
    if os.name == 'nt':
        import msvcrt
        while msvcrt.kbhit():
            msvcrt.getwch()
    else:
        import termios
        termios.tcflush(sys.stdin.fileno(), termios.TCIFLUSH)


def main(argv=None, *, entered_at=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--scope', choices=('cake-only', 'full'), required=True)
    parser.add_argument('--prepare-timeout', type=float, default=120.0,
                        help='seconds allowed for read-only mission preparation')
    parser.add_argument('--report-timeout', type=float, default=660.0,
                        help='seconds waiting for the automatic result after Enter')
    args = parser.parse_args(argv)
    if any(not math.isfinite(value) or value <= 0 for value in (args.prepare_timeout, args.report_timeout)):
        parser.error('timeouts must be finite and positive')
    if entered_at is None and not sys.stdin.isatty():
        parser.error('a live terminal is required; redirected or buffered Enter is refused')
    if entered_at is not None and (type(entered_at) not in (int, float)
                                   or not math.isfinite(entered_at) or not 0 <= entered_at <= time.monotonic()):
        raise ValueError('invalid terminal Enter monotonic timestamp')
    import rospy
    from std_msgs.msg import String
    from suat_mission.automation.start_console import SingleEnterConsole

    if entered_at is None:
        rospy.init_node('suat_mission_console', anonymous=True)
    console = SingleEnterConsole(args.scope)
    preparation_id = uuid.uuid4().hex
    ready, done = threading.Event(), threading.Event()
    prepared_state, report = {}, {}
    run_id = [None]
    last_progress=[None]
    def state_message(msg):
        try:
            state = json.loads(msg.data)
            if state.get('preparation_id') == preparation_id:
                prepared_state.clear()
                prepared_state.update(state)
                console.observe(state)
                if run_id[0] and state.get('state')=='running':
                    phase=state.get('phase')
                    if phase and phase!=last_progress[0]:
                        last_progress[0]=phase
                        print('[自动阶段] '+str(phase),flush=True)
                if state.get('prepared') or state.get('error'):
                    ready.set()
            else:
                console.observe({})
        except (TypeError, ValueError, AttributeError):
            return
    def report_message(msg):
        try:
            value = json.loads(msg.data)
            if run_id[0] and value.get('run_id') == run_id[0]:
                report.update(value)
                done.set()
        except (TypeError, ValueError, AttributeError):
            return
    command = rospy.Publisher('/suat/mission/command', String, queue_size=1, latch=False)
    subscriptions = [rospy.Subscriber('/suat/mission/state', String, state_message, queue_size=1),
                     rospy.Subscriber('/suat/mission/report', String, report_message, queue_size=1)]
    def publish(value):
        command.publish(String(data=json.dumps(value)))
    def wait(event, timeout):
        deadline = time.monotonic() + timeout
        while not event.wait(.1):
            if rospy.is_shutdown() or time.monotonic() >= deadline:
                raise RuntimeError('mission response timed out; inspect mission state and saved run directory')
    try:
        connected_by = time.monotonic() + args.prepare_timeout
        while not command.get_num_connections():
            if rospy.is_shutdown() or time.monotonic() >= connected_by:
                raise RuntimeError('mission node is unavailable')
            time.sleep(.05)
        preparation = {'command': 'prepare', 'scope': args.scope, 'request_id': preparation_id}
        if entered_at is not None:
            preparation['entered_at_monotonic'] = entered_at
        publish(preparation)
        wait(ready, args.prepare_timeout)
        if not console.ready():
            raise RuntimeError(prepared_state.get('error') or 'mission preparation was refused')
        run_id[0] = prepared_state['run_id']
        print('Prepared {} run {}; records: {}'.format(args.scope, run_id[0], prepared_state['run_directory']))
        if entered_at is None:
            console.start_once(discard_pending_input, lambda: input('Press Enter once to start: '), publish)
        else:
            console.start_after_enter(run_id[0], entered_at, publish)
        print('Start sent once. Waiting for the automatic result; Ctrl+C requests an intervention stop.')
        wait(done, args.report_timeout)
        print(json.dumps(report, ensure_ascii=False, indent=2))
        return 0 if report.get('status') == 'completed' and not report.get('recording_or_stop_error') else 2
    except KeyboardInterrupt:
        publish({'command': 'stop'})
        print('Operator stop requested; inspect the saved result and actual robot state.', file=sys.stderr)
        return 130
    except (RuntimeError, ValueError, EOFError) as exc:
        print(str(exc), file=sys.stderr)
        return 2
    finally:
        for subscription in subscriptions:
            subscription.unregister()
        command.unregister()


if __name__ == '__main__':
    raise SystemExit(main())
