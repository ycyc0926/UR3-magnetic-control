"""Dark-object tracking and calibrated plane projection; no robot interfaces."""
from pathlib import Path

import cv2
import numpy as np
import yaml


class PlaneMapper:
    def __init__(self, filename):
        self.filename = str(Path(filename).resolve())
        with open(filename, encoding='utf-8') as stream:
            data = yaml.safe_load(stream)
        self.sensor_size = np.asarray(data['camera']['image_size_px'], dtype=float)
        self.K = np.asarray(data['camera']['camera_matrix'], dtype=float)
        self.distortion = np.asarray(data['camera']['distortion_coefficients'], dtype=float)
        self.H = np.asarray(data['acrylic_plane']['H_world_xy_from_undistorted_pixel'], dtype=float)
        self.z = float(data['acrylic_plane']['world_z_m'])
        if not np.isfinite(self.H).all() or abs(np.linalg.det(self.H)) < 1e-15:
            raise ValueError('Invalid plane homography')

    def project(self, pixel, image_size):
        """Input is a full-FOV resized image, with no crop/binning offset.

        Undo OpenCV resize pixel-centre scaling BEFORE lens undistortion and
        homography. World coordinates describe a projection onto the acrylic.
        """
        scale = self.sensor_size / np.asarray(image_size, dtype=float)
        full_pixel = (np.asarray(pixel, dtype=float) + .5)*scale - .5
        point = cv2.undistortPoints(full_pixel.reshape(1, 1, 2), self.K,
                                   self.distortion, P=self.K).reshape(2)
        homogeneous = self.H @ np.r_[point, 1.0]
        if not np.isfinite(homogeneous).all() or abs(homogeneous[2]) < 1e-12:
            raise ValueError('Plane projection is undefined')
        return homogeneous[:2]/homogeneous[2], full_pixel

    def image_points(self, world_xy, image_size):
        """Draw world-plane references back on the distorted live image."""
        xy = np.asarray(world_xy, dtype=float).reshape(-1, 2)
        if not np.isfinite(xy).all():
            raise ValueError('Nonfinite reference path')
        homogeneous = (np.linalg.inv(self.H) @ np.c_[xy, np.ones(len(xy))].T).T
        if np.any(np.abs(homogeneous[:, 2]) < 1e-12):
            raise ValueError('Reference projects to infinity')
        pixels = homogeneous[:, :2]/homogeneous[:, 2:]
        rays = (np.linalg.inv(self.K) @ np.c_[pixels, np.ones(len(xy))].T).T
        raw, _ = cv2.projectPoints(rays, np.zeros(3), np.zeros(3), self.K, self.distortion)
        scale = self.sensor_size/np.asarray(image_size, dtype=float)
        return (raw.reshape(-1, 2)+.5)/scale-.5


def reference_path(kind, start_xy, size_mm=20.0):
    """Visual reference only; no IK, reachability, or robot command semantics."""
    start = np.asarray(start_xy, dtype=float)
    if start.shape != (2,) or not np.isfinite(start).all():
        raise ValueError('Invalid path origin')
    if not np.isfinite(size_mm) or not 5 <= size_mm <= 40:
        raise ValueError('Visual reference size must be 5--40 mm')
    offsets = {'line': [[0, 0], [0, 1]],
               'line_negative_y': [[0, 0], [0, -1]],
               'square': [[0, 0], [0, 1], [1, 1], [1, 0], [0, 0]]}
    if kind not in offsets:
        raise ValueError('Unknown reference shape')
    return start + np.asarray(offsets[kind])*size_mm/1000


class DarkTargetTracker:
    """Track the selected dark silhouette, not a semantic H classifier.

    Initial selection is near image centre. Position and area gates maintain
    identity; after sustained loss, an explicit reset is needed to reacquire.
    """
    def __init__(self, threshold=80, minimum_area=150, maximum_area=8000):
        self.threshold = threshold
        self.minimum_area = minimum_area
        self.maximum_area = maximum_area
        self.reset()

    def reset(self, seed=None):
        self.seed = None if seed is None else np.asarray(seed, float)
        self.last = None
        self.area = None
        self.misses = 0
        self.locked_out = False
        self.initialized = False
        self.diagnostics = {'reason': 'not_selected'}

    def detect(self, image):
        height, width = image.shape[:2]
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        gray = cv2.GaussianBlur(gray, (3, 3), 0)
        mask = cv2.inRange(gray, 0, self.threshold)
        margin_x, margin_y = int(width*.07), int(height*.07)
        mask[:margin_y] = 0; mask[height-margin_y:] = 0
        mask[:, :margin_x] = 0; mask[:, width-margin_x:] = 0
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        candidates = []
        rejected = {'area': 0, 'shape_or_border': 0, 'area_change': 0,
                    'moment': 0, 'distance': 0}
        reference = self.last if self.last is not None else (
            self.seed if self.seed is not None else np.array([width/2, height/2]))
        radius = 35 if self.last is not None else min(width, height)*.15
        for contour in contours:
            area = cv2.contourArea(contour)
            if not self.minimum_area <= area <= self.maximum_area:
                rejected['area'] += 1
                continue
            x, y, w, h = cv2.boundingRect(contour)
            if not .2 <= w/h <= 5 or x <= margin_x or y <= margin_y or x+w >= width-margin_x or y+h >= height-margin_y:
                rejected['shape_or_border'] += 1
                continue
            if self.area is not None and not .3 <= area/self.area <= 3:
                rejected['area_change'] += 1
                continue
            moment = cv2.moments(contour)
            if moment['m00'] <= 0:
                rejected['moment'] += 1
                continue
            center = np.array([moment['m10']/moment['m00'], moment['m01']/moment['m00']])
            distance = np.linalg.norm(center-reference)
            if distance <= radius:
                candidates.append((float(distance), center, area, contour, (x, y, w, h)))
            else:
                rejected['distance'] += 1
        self.diagnostics = {'contours': len(contours), 'candidates': len(candidates),
                            'rejected': rejected, 'radius_px': radius,
                            'threshold': self.threshold}
        if self.locked_out or not candidates:
            self.misses += 1
            if self.initialized and self.misses >= 5:
                self.locked_out = True
            self.diagnostics.update(reason='locked_out' if self.locked_out else 'no_candidate',
                                    misses=self.misses)
            return None
        candidates.sort(key=lambda c: c[0])
        if len(candidates) > 1 and candidates[1][0]-candidates[0][0] < 15:
            self.misses += 1
            if self.misses >= 5:
                self.locked_out = True
            self.diagnostics.update(reason='locked_out' if self.locked_out else 'ambiguous',
                                    misses=self.misses)
            return None
        distance, center, area, contour, box = candidates[0]
        self.last = center
        self.area = area if self.area is None else .95*self.area + .05*area
        self.misses = 0
        self.initialized = True
        self.diagnostics.update(reason='tracked', misses=0)
        return {'pixel': center, 'area_px': area, 'contour': contour, 'box': box,
                'confidence': max(.1, 1-distance/radius)}
