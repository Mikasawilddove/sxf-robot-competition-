"""Compare the rule illustration template against the separately photographed cake block."""
import json
import sys
from pathlib import Path

import cv2
import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'src' / 'suat_mission' / 'src'))
from suat_mission.template_matcher import TemplateMatcher


def read_image(path):
    image = cv2.imdecode(np.fromfile(str(path), dtype=np.uint8), cv2.IMREAD_COLOR)
    if image is None:
        raise ValueError('cannot read image: ' + str(path))
    return image


def evaluate():
    # Do not use cake.png here: that template was cropped from the test photo.
    templates = {label: [read_image(ROOT / 'src' / 'suat_mission' / 'assets' / 'templates' / (label + '.png'))]
                 for label in ('teddybear', 'clock', 'banana')}
    templates['cake'] = [read_image(ROOT / 'src' / 'suat_mission' / 'assets' / 'templates' / 'cake_rule_reference.png')]
    image = cv2.resize(read_image(ROOT / 'assets' / 'source_images' / 'cake.jpg'), (640, 480))
    matcher = TemplateMatcher(templates, scales=[.4, .5, .6, .7, .8, .9, 1., 1.1, 1.2, 1.3,
                                                 1.4, 1.5, 1.6, 1.7, 1.8, 1.9, 2.],
                              rotations=[0, 90, 180, 270], threshold=.6, max_peaks=8)
    return image, matcher.detect(image)


def main():
    image, detections = evaluate()
    for d in detections:
        x1, y1 = int(d.x-d.width/2), int(d.y-d.height/2)
        x2, y2 = int(d.x+d.width/2), int(d.y+d.height/2)
        cv2.rectangle(image, (x1, y1), (x2, y2), (0, 200, 0), 2)
        cv2.putText(image, '{} {:.2f}'.format(d.label, d.score), (x1, max(15, y1-6)),
                    cv2.FONT_HERSHEY_SIMPLEX, .5, (0, 70, 240), 1, cv2.LINE_AA)
    dest = ROOT / 'assets' / 'evaluation'
    dest.mkdir(parents=True, exist_ok=True)
    ok, data = cv2.imencode('.png', image)
    if not ok:
        raise RuntimeError('could not encode evaluation image')
    (dest / 'cake_detected.png').write_bytes(data.tobytes())
    result = {'scene': 'separate photograph of cake block', 'cake_template': 'draft-rule illustration',
              'score_threshold': .6,
              'detections': [{'label': d.label, 'score': round(d.score, 4),
                              'center': [round(d.x, 1), round(d.y, 1)]} for d in detections],
              'limitations': ['one photo only', 'no real robot camera or motion was tested']}
    (dest / 'cake_results.json').write_text(json.dumps(result, indent=2, ensure_ascii=False), encoding='utf-8')
    print(json.dumps(result, indent=2, ensure_ascii=False))
    if len(detections) != 1 or detections[0].label != 'cake':
        raise SystemExit('cake photo did not produce exactly one cake detection')


if __name__ == '__main__':
    main()
