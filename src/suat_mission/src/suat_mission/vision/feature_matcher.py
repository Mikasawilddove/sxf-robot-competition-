"""Perspective-aware feature localization for the printed cake image.

The matching score is the RANSAC inlier fraction, not a calibrated probability.
"""
import cv2
import numpy as np

from suat_mission.vision.perception import Detection
from suat_mission.vision.template_matcher import _iou


class CakeFeatureMatcher:
    def __init__(self, templates, nfeatures=2000, ratio=0.75,
                 min_matches=14, min_inliers=14, min_inlier_ratio=0.30):
        if not templates or not 0 < ratio < 1 or min_matches < 4 or min_inliers < 4:
            raise ValueError('invalid feature-matcher configuration')
        if not 0 < min_inlier_ratio <= 1:
            raise ValueError('invalid inlier ratio')
        self.orb = cv2.ORB_create(nfeatures=int(nfeatures), edgeThreshold=5, patchSize=15, fastThreshold=5)
        self.bf = cv2.BFMatcher(cv2.NORM_HAMMING)
        self.ratio = float(ratio)
        self.min_matches = int(min_matches)
        self.min_inliers = int(min_inliers)
        self.min_inlier_ratio = float(min_inlier_ratio)
        self.references = []
        for image in templates:
            if image is None or image.size == 0:
                raise ValueError('empty cake reference')
            gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY) if image.ndim == 3 else image
            keys, descriptors = self.orb.detectAndCompute(gray, None)
            if descriptors is None or len(keys) < self.min_matches:
                raise ValueError('cake reference has too few features')
            self.references.append((gray.shape, keys, descriptors))

    def detect(self, image):
        if image is None or image.size == 0:
            raise ValueError('empty input image')
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY) if image.ndim == 3 else image
        scene_keys, scene_desc = self.orb.detectAndCompute(gray, None)
        if scene_desc is None or len(scene_keys) < self.min_matches:
            return []
        candidates = []
        for shape, ref_keys, ref_desc in self.references:
            pairs = self.bf.knnMatch(ref_desc, scene_desc, k=2)
            matches = [pair[0] for pair in pairs if len(pair) == 2 and pair[0].distance < self.ratio * pair[1].distance]
            if len(matches) < self.min_matches:
                continue
            src = np.float32([ref_keys[m.queryIdx].pt for m in matches]).reshape(-1, 1, 2)
            dst = np.float32([scene_keys[m.trainIdx].pt for m in matches]).reshape(-1, 1, 2)
            matrix, mask = cv2.findHomography(src, dst, cv2.RANSAC, 3.0)
            if matrix is None or mask is None or not np.isfinite(matrix).all():
                continue
            inliers = int(mask.sum())
            score = float(inliers) / len(matches)
            if inliers < self.min_inliers or score < self.min_inlier_ratio:
                continue
            h, w = shape
            reference_box = np.float32([[0, 0], [w, 0], [w, h], [0, h]]).reshape(-1, 1, 2)
            quad = cv2.perspectiveTransform(reference_box, matrix).reshape(-1, 2)
            if not np.isfinite(quad).all() or not cv2.isContourConvex(quad.astype(np.float32)):
                continue
            area = abs(cv2.contourArea(quad.astype(np.float32)))
            if area < 100 or area > gray.shape[0] * gray.shape[1] * 0.5:
                continue
            if (quad[:, 0].min() < 0 or quad[:, 1].min() < 0 or
                    quad[:, 0].max() >= gray.shape[1] or quad[:, 1].max() >= gray.shape[0]):
                continue
            x1, y1 = quad.min(axis=0); x2, y2 = quad.max(axis=0)
            if x2 - x1 < 10 or y2 - y1 < 10:
                continue
            candidates.append((score, (float(x1), float(y1), float(x2), float(y2))))
        candidates.sort(key=lambda item: item[0], reverse=True)
        selected = []
        for score, box in candidates:
            if not any(_iou(box, prior[1]) > 0.3 for prior in selected):
                selected.append((score, box))
        return [Detection('cake', score, (box[0]+box[2])/2, (box[1]+box[3])/2,
                          box[2]-box[0], box[3]-box[1]) for score, box in selected]
