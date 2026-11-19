"""Offline OpenCV template detector, usable with real reference crops later."""
import math
from dataclasses import asdict

import cv2
import numpy as np

from suat_mission.vision.perception import ALL_ITEMS, OBJECTS, Detection


def _iou(a, b):
    x1, y1 = max(a[0], b[0]), max(a[1], b[1])
    x2, y2 = min(a[2], b[2]), min(a[3], b[3])
    intersection = max(0, x2 - x1) * max(0, y2 - y1)
    union = (a[2]-a[0]) * (a[3]-a[1]) + (b[2]-b[0]) * (b[3]-b[1]) - intersection
    return intersection / union if union else 0.0


class TemplateMatcher:
    def __init__(self, templates, scales=(0.8, 1.0, 1.2), rotations=(0,),
                 threshold=0.8, nms_iou=0.3, max_peaks=30):
        if not templates or any(label not in ALL_ITEMS for label in templates):
            raise ValueError('templates need known item labels')
        if not 0 < threshold <= 1 or not 0 <= nms_iou <= 1 or max_peaks < 1:
            raise ValueError('invalid matching thresholds')
        self.variants = []
        self.threshold = float(threshold)
        self.nms_iou = float(nms_iou)
        self.max_peaks = int(max_peaks)
        for label, images in templates.items():
            if not images:
                raise ValueError('empty template list for ' + label)
            for image in images:
                if image is None or image.size == 0:
                    raise ValueError('empty template image for ' + label)
                gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY) if image.ndim == 3 else image
                if gray.ndim != 2 or np.std(gray) < 3:
                    raise ValueError('template must contain visible detail: ' + label)
                for angle in rotations:
                    if angle not in (0, 90, 180, 270):
                        raise ValueError('only quarter-turn rotations are supported')
                    rotated = np.rot90(gray, int(angle // 90))
                    for scale in scales:
                        if not math.isfinite(float(scale)) or scale <= 0:
                            raise ValueError('invalid template scale')
                        width = max(2, int(round(rotated.shape[1] * scale)))
                        height = max(2, int(round(rotated.shape[0] * scale)))
                        resized = cv2.resize(rotated, (width, height), interpolation=cv2.INTER_AREA)
                        self.variants.append((label, np.ascontiguousarray(resized)))

    def detect(self, image):
        if image is None or image.size == 0:
            raise ValueError('empty input image')
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY) if image.ndim == 3 else image
        candidates = []
        for label, template in self.variants:
            h, w = template.shape
            if h > gray.shape[0] or w > gray.shape[1]:
                continue
            scores = cv2.matchTemplate(gray, template, cv2.TM_CCOEFF_NORMED)
            peaks = (scores >= self.threshold) & (scores == cv2.dilate(scores, np.ones((3, 3), np.uint8)))
            ys, xs = np.where(peaks)
            if len(xs) > self.max_peaks:
                chosen = np.argpartition(scores[ys, xs], -self.max_peaks)[-self.max_peaks:]
                xs, ys = xs[chosen], ys[chosen]
            for x, y in zip(xs, ys):
                candidates.append((float(scores[y, x]), label, (int(x), int(y), int(x+w), int(y+h))))
        candidates.sort(key=lambda item: item[0], reverse=True)
        kept = []
        for score, label, box in candidates:
            if any(_iou(box, existing[2]) > self.nms_iou for existing in kept):
                continue
            kept.append((score, label, box))
        return [Detection(label, min(1.0, max(0.0, score)), (box[0]+box[2])/2.0,
                          (box[1]+box[3])/2.0, box[2]-box[0], box[3]-box[1])
                for score, label, box in kept]

    def detect_board(self, image, *, slot_edges=(0.,1/3,2/3,1.), containment=.9,
                     score_margin=.1):
        """Rectified board candidate matching, with explicit suppression evidence.

        A much weaker local texture inside a stronger full icon can survive IoU
        NMS. Only near-contained candidates in the SAME board slot compete here.
        Close scores and separate instances remain for the reader to reject.
        These are development parameters, not calibrated probabilities.
        """
        if (len(slot_edges)!=4 or slot_edges[0]!=0 or slot_edges[-1]!=1
                or not all(math.isfinite(v) for v in slot_edges)
                or any(a>=b for a,b in zip(slot_edges,slot_edges[1:]))
                or not math.isfinite(containment) or not 0<containment<=1
                or not math.isfinite(score_margin) or not 0<score_margin<=1):
            raise ValueError('invalid board candidate suppression parameters')
        raw=self.detect(image)
        kept=[];suppressed=[]
        def box(d):
            return (d.x-d.width/2,d.y-d.height/2,d.x+d.width/2,d.y+d.height/2)
        def slot(d):
            return int(np.searchsorted(slot_edges,d.x/image.shape[1],side='right'))-1
        for candidate in sorted((d for d in raw if d.label in OBJECTS),
                                key=lambda d:d.score,reverse=True):
            removed=False
            for winner in kept:
                if (slot(candidate)!=slot(winner) or not 0<=slot(candidate)<3
                        or winner.width*winner.height<candidate.width*candidate.height
                        or winner.score-candidate.score<score_margin):
                    continue
                a,b=box(candidate),box(winner)
                overlap=(max(0.,min(a[2],b[2])-max(a[0],b[0]))*
                         max(0.,min(a[3],b[3])-max(a[1],b[1])))
                coverage=overlap/(candidate.width*candidate.height)
                if coverage>=containment:
                    suppressed.append({'candidate':asdict(candidate),'winner':asdict(winner),
                                       'coverage_of_smaller':coverage,'slot':slot(candidate),
                                       'reason':'weaker_near_contained_board_texture'})
                    removed=True
                    break
            if not removed:kept.append(candidate)
        return {'detections':kept,'raw_detections':[asdict(d) for d in raw],
                'suppressed':suppressed,'coordinate_space':'normalized_board',
                'parameters':{'slot_edges':list(slot_edges),'containment':containment,
                              'score_margin':score_margin,'score_semantics':'TM_CCOEFF_NORMED'}}
