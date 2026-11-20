"""Experimental BRISK + yellow contour detector for read-only field evaluation.

Scores are geometric/color-shape heuristics, not probabilities. Never use these
outputs to drive hardware until validated on independent robot-camera scenes.
"""
from dataclasses import replace
from pathlib import Path

import cv2
import numpy as np

from suat_mission.vision.perception import Detection
from suat_mission.vision.template_matcher import _iou


TEMPLATE_FILES = {
    'banana': ['banana.png'],
    'cake': ['cake.png', 'cake_rule_reference.png'],
    'clock': ['clock.png'],
    'teddybear': ['teddybear.png'],
}


def measure_layer(rise, box, layer_rise=100.0, block_h=100.0, tol=0.25):
    """Conservative single-frame estimate: return (layer, rise), or (None, rise).

    The reference plane and heights must be independently validated.
    """
    if rise is None or not isinstance(rise, np.ndarray) or rise.ndim != 2:
        return None, None
    if not np.isfinite(layer_rise) or layer_rise <= 0 or not np.isfinite(block_h) or block_h <= 0:
        return None, None
    try:
        x, y, w, h = (int(v) for v in box)
    except (TypeError, ValueError, OverflowError):
        return None, None
    if w <= 0 or h <= 0:
        return None, None
    rows, cols = rise.shape
    x0, x1 = max(0, x), min(cols, x + w)
    y0, y1 = max(0, y), min(rows, y + h)
    if x1 - x0 < 8 or y1 - y0 < 8:
        return None, None
    width, height = x1 - x0, y1 - y0
    left, right = x0 + int(width * 0.10), x1 - int(width * 0.10)
    top = y0 + int(height * 0.05)
    bottom = y0 + max(int(height * 0.45), int(height * 0.05) + 4)
    band = rise[top:min(bottom, y1), left:right]
    finite = band[np.isfinite(band)]
    if finite.size < max(40, int(np.ceil(band.size * 0.25))):
        return None, None
    max_layer = min(2, int(250.0 // block_h))
    tolerance = min(max(float(tol), 0.0), 0.25) * layer_rise
    support = [np.abs(finite - (i + 1) * layer_rise) <= tolerance
               for i in range(max_layer + 1)]
    counts = [int(np.count_nonzero(mask)) for mask in support]
    winner = int(np.argmax(counts))
    competing = max([count for i, count in enumerate(counts) if i != winner] or [0])
    if (counts[winner] < max(40, int(np.ceil(0.60 * finite.size)))
            or competing >= 0.20 * finite.size):
        return None, float(np.median(finite))
    return winner, float(np.median(finite[support[winner]]))


class ExperimentalVision:
    @classmethod
    def from_template_directory(cls, directory):
        directory = Path(directory)
        templates = {}
        for label, files in TEMPLATE_FILES.items():
            images = []
            for filename in files:
                path = directory / filename
                image = cv2.imread(str(path))
                if image is None or not image.size:
                    raise RuntimeError('template unavailable: ' + str(path))
                images.append(image)
            templates[label] = images
        return cls(templates, banana_saturation_min=55)

    def __init__(self, templates, max_shape_distance=0.20, banana_saturation_min=80):
        if set(templates) != {'teddybear', 'clock', 'banana', 'cake'}:
            raise ValueError('four labels required')
        self.brisk = cv2.BRISK_create(thresh=15)
        self.matcher = cv2.BFMatcher(cv2.NORM_HAMMING)
        self.refs = []
        self.max_shape_distance = max_shape_distance
        if not 0 <= int(banana_saturation_min) <= 255:
            raise ValueError('invalid banana saturation threshold')
        self.banana_saturation_min = int(banana_saturation_min)
        for label in ('teddybear', 'clock', 'cake'):
            for image in templates[label]:
                if image is None or not image.size:
                    raise ValueError('empty image for ' + label)
                gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
                keypoints, desc = self.brisk.detectAndCompute(gray, None)
                if desc is None or len(keypoints) < 8:
                    raise ValueError('not enough features for ' + label)
                self.refs.append((label, gray.shape, keypoints, desc))
        banana = templates['banana'][0]
        if banana is None or not banana.size:
            raise ValueError('empty banana template')
        contours = self._yellow_contours(banana)
        if not contours:
            raise ValueError('banana template has no yellow contour')
        self.banana_shape = max(contours, key=cv2.contourArea)

    def detect_for_target(self, image, target, min_score=0.45, min_hits=1):
        """Use the grasp script's enhanced-image retry for a missing named target."""
        found = [item for item in self.detect(image) if item.score >= min_score]
        count = sum(item.label == target for item in found)
        if count >= min_hits:
            return found
        light, a, b = cv2.split(cv2.cvtColor(image, cv2.COLOR_BGR2LAB))
        light = cv2.createCLAHE(clipLimit=3.0, tileGridSize=(8, 8)).apply(light)
        enhanced = cv2.cvtColor(cv2.merge([light, a, b]), cv2.COLOR_LAB2BGR)
        enhanced = cv2.resize(enhanced, None, fx=2.0, fy=2.0, interpolation=cv2.INTER_CUBIC)
        recovered = [
            replace(item, x=item.x / 2.0, y=item.y / 2.0,
                    width=item.width / 2.0, height=item.height / 2.0)
            for item in self.detect(enhanced)
            if item.label == target and item.score >= min_score
        ]
        if len(recovered) > count:
            return [item for item in found if item.label != target] + recovered
        return found

    @staticmethod
    def _yellow_contours(image, saturation_min=80):
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, np.array([15, saturation_min, 70], np.uint8), np.array([40, 255, 255], np.uint8))
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, np.ones((3, 3), np.uint8))
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        return contours

    def detect(self, image):
        if image is None or not image.size:
            raise ValueError('empty image')
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        scene_keys, scene_desc = self.brisk.detectAndCompute(gray, None)
        hscene, wscene = gray.shape
        candidates = []
        if scene_desc is not None and len(scene_keys) >= 8:
            for label, shape, ref_keys, ref_desc in self.refs:
                pairs = self.matcher.knnMatch(ref_desc, scene_desc, k=2)
                matches = [p[0] for p in pairs if len(p) == 2 and p[0].distance < 0.8 * p[1].distance]
                if len(matches) < 8:
                    continue
                src = np.float32([ref_keys[m.queryIdx].pt for m in matches])
                dst = np.float32([scene_keys[m.trainIdx].pt for m in matches])
                matrix, inlier_mask = cv2.findHomography(src, dst, cv2.RANSAC, 3.0)
                if matrix is None or inlier_mask is None or not np.isfinite(matrix).all():
                    continue
                inliers = int(inlier_mask.sum())
                score = inliers / float(len(matches))
                if inliers < 8 or score < 0.45:
                    continue
                height, width = shape
                corners = np.float32([[0, 0], [width, 0], [width, height], [0, height]]).reshape(-1, 1, 2)
                quad = cv2.perspectiveTransform(corners, matrix).reshape(-1, 2)
                if not np.isfinite(quad).all() or not cv2.isContourConvex(quad.astype(np.float32)):
                    continue
                area = abs(cv2.contourArea(quad.astype(np.float32)))
                if area < 100 or area > hscene * wscene * .5:
                    continue
                if quad[:, 0].min() < 0 or quad[:, 1].min() < 0 or quad[:, 0].max() >= wscene or quad[:, 1].max() >= hscene:
                    continue
                x1, y1 = quad.min(axis=0); x2, y2 = quad.max(axis=0)
                candidates.append(Detection(label, score, float((x1+x2)/2), float((y1+y2)/2),
                                            float(x2-x1), float(y2-y1)))
        for contour in self._yellow_contours(image, self.banana_saturation_min):
            area = cv2.contourArea(contour)
            if not hscene*wscene*.0013 <= area <= hscene*wscene*.04:
                continue
            x, y, width, height = cv2.boundingRect(contour)
            if x <= 1 or y <= 1 or x+width >= wscene-1 or y+height >= hscene-1:
                continue
            distance = cv2.matchShapes(contour, self.banana_shape, cv2.CONTOURS_MATCH_I1, 0)
            if not np.isfinite(distance) or distance > self.max_shape_distance:
                continue
            # Similarity is heuristic and should never be interpreted as calibrated confidence.
            candidates.append(Detection('banana', float(1.0/(1.0+distance)),
                                        x+width/2., y+height/2., float(width), float(height)))
        # Different reference photographs of the same printed face can match
        # simultaneously. Suppress overlapping boxes of the same class only.
        selected = []
        for candidate in sorted(candidates, key=lambda d: d.score, reverse=True):
            box = (candidate.x-candidate.width/2, candidate.y-candidate.height/2,
                   candidate.x+candidate.width/2, candidate.y+candidate.height/2)
            duplicate = False
            for other in selected:
                other_box = (other.x-other.width/2, other.y-other.height/2,
                             other.x+other.width/2, other.y+other.height/2)
                if candidate.label == other.label and _iou(box, other_box) > 0.3:
                    duplicate = True
                    break
            if not duplicate:
                selected.append(candidate)
        return selected
