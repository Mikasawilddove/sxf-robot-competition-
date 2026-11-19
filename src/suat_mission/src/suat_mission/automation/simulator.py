"""Deterministic, hardware-free match scenario and command-line demonstration."""
import argparse
import json
import time
from collections import Counter
from itertools import permutations

from suat_mission.automation.mission import Mission, MissionConfig
from suat_mission.vision.perception import Detection, OBJECTS
from suat_mission.contracts import ActionResult, Outcome
from suat_mission.automation.world import SceneSnapshot


class SimRobot:
    def __init__(self, order=OBJECTS, fail_at=None):
        self.order = tuple(order)
        self.fail_at = fail_at
        self.stock = Counter({"cake": 1, "teddybear": 3, "clock": 3, "banana": 3})
        self.holding = None
        self.commands = []
        self.stopped = False
        self.scene_version = 0
        self.held_id = None
        self.picked_ids = set()

    def log(self, text):
        self.commands.append(text)

    def is_stopped(self):
        return self.stopped

    def navigate(self, name, remaining, context):
        success = self.fail_at != "navigate:" + name and remaining > 0
        return ActionResult(context, Outcome.CONFIRMED if success else Outcome.FAILED,
                            'synthetic navigation ' + name)

    def read_board_frames(self):
        frame = [Detection(name, 0.95, 100 + 150 * i, 220, 40, 40)
                 for i, name in enumerate(self.order)]
        return [list(frame) for _ in range(5)]

    def detect_objects(self):
        self.scene_version += 1
        detections = []
        for label, count in self.stock.items():
            ids = ['{}-{}'.format(label, i) for i in range(1, 4) if '{}-{}'.format(label, i) not in self.picked_ids]
            detections.extend(Detection(label, .95, 320 + 10 * i, 300, 60, 60, object_id)
                              for i, object_id in enumerate(ids[:count]))
        return SceneSnapshot('synthetic-fixture', self.scene_version, time.monotonic(), 'synthetic_camera',
                             tuple(detections))

    def pick(self, target, label, context):
        if self.fail_at == "pick:" + label or self.holding is not None or self.stock[label] == 0:
            return ActionResult(context, Outcome.FAILED, 'synthetic grasp failed')
        if target.object_id in self.picked_ids or context.object_id != target.object_id:
            return ActionResult(context, Outcome.FAILED, 'synthetic duplicate identity')
        self.stock[label] -= 1
        self.holding = label
        self.held_id = target.object_id
        self.picked_ids.add(target.object_id)
        return ActionResult(context, Outcome.CONFIRMED, 'synthetic attachment verified')

    def drop(self, zone, label, context):
        if self.fail_at == "drop:" + label or self.holding != label:
            return ActionResult(context, Outcome.UNKNOWN, 'synthetic release not verified')
        if self.held_id != context.object_id:
            return ActionResult(context, Outcome.FAILED, 'synthetic held identity mismatch')
        self.holding = None
        self.held_id = None
        return ActionResult(context, Outcome.CONFIRMED, 'synthetic object settled inside zone ' + zone)

    def stop_motion(self):
        self.commands.append("stop_motion")


def main():
    parser = argparse.ArgumentParser(description="Run a local simulated 10-block match")
    parser.add_argument("--board", choices=[",".join(p) for p in permutations(OBJECTS)],
                        default=",".join(OBJECTS), help="board labels from left to right")
    parser.add_argument("--fail-at", default=None, help="inject failure, e.g. pick:cake")
    args = parser.parse_args()
    robot = SimRobot(args.board.split(","), args.fail_at)
    report = Mission(robot, MissionConfig()).run()
    from dataclasses import asdict
    print(json.dumps(asdict(report), ensure_ascii=False, indent=2))
    return 0 if report.status == "completed" else 1


if __name__ == "__main__":
    raise SystemExit(main())
