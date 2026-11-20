"""Evaluate crop templates on the separately photographed classification board."""
import json
import sys
from pathlib import Path

import cv2
import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'src' / 'suat_mission' / 'src'))
from suat_mission.perception import board_order
from suat_mission.template_matcher import TemplateMatcher


def read_image(path):
    image = cv2.imdecode(np.fromfile(str(path), dtype=np.uint8), cv2.IMREAD_COLOR)
    if image is None:
        raise ValueError('cannot read image: ' + str(path))
    return image


def evaluate():
    labels = ('teddybear', 'clock', 'banana', 'cake')
    templates = {name: [read_image(ROOT / 'src' / 'suat_mission' / 'assets' / 'templates' / (name + '.png'))] for name in labels}
    templates['cake'].append(read_image(ROOT / 'src' / 'suat_mission' / 'assets' / 'templates' / 'cake_rule_reference.png'))
    image = cv2.resize(read_image(ROOT / 'assets' / 'source_images' / 'board.jpg'), (480, 640))
    matcher = TemplateMatcher(templates, scales=[.4, .5, .6, .7, .8, .9, 1., 1.1, 1.2, 1.3,
                                                 1.4, 1.5, 1.6, 1.7, 1.8, 1.9, 2.],
                              rotations=[0, 90, 180, 270], threshold=.6, max_peaks=8)
    detections = matcher.detect(image)
    order = board_order(detections, axis='y')
    return image, detections, order


def main():
    image, detections, order = evaluate()
    for det in detections:
        x1, y1 = int(det.x-det.width/2), int(det.y-det.height/2)
        x2, y2 = int(det.x+det.width/2), int(det.y+det.height/2)
        cv2.rectangle(image, (x1, y1), (x2, y2), (0, 200, 0), 2)
        cv2.putText(image, '{} {:.2f}'.format(det.label, det.score), (x1, max(15, y1-6)),
                    cv2.FONT_HERSHEY_SIMPLEX, .5, (0, 70, 240), 1, cv2.LINE_AA)
    dest = ROOT / 'assets' / 'evaluation'
    dest.mkdir(parents=True, exist_ok=True)
    ok, data = cv2.imencode('.png', image)
    if not ok:
        raise RuntimeError('could not encode evaluation image')
    (dest / 'board_detected.png').write_bytes(data.tobytes())
    report = {'scene': 'separate board photograph', 'score_threshold': .6, 'axis': 'y',
              'order': order, 'detections': [{'label': d.label, 'score': round(d.score, 4),
                                              'center': [round(d.x, 1), round(d.y, 1)]}
                                             for d in detections],
              'limitations': ['one board photo only', 'cake not present on this board',
                              'no real robot camera or motion was tested']}
    (dest / 'board_results.json').write_text(json.dumps(report, indent=2, ensure_ascii=False), encoding='utf-8')
    print(json.dumps(report, indent=2, ensure_ascii=False))
    if order != ('teddybear', 'clock', 'banana'):
        raise SystemExit('board photo did not match the manually inspected order')


if __name__ == '__main__':
    main()
