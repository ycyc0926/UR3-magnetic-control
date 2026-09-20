#!/usr/bin/env python3
"""Detect the physical ArUco target and return its 100 mm black-square corners.

The printed target has a white mounting margin that OpenCV can occasionally use
as a second, larger marker candidate.  The ID is therefore decoded with ArUco,
but the metric corners are taken from the largest dark quadrilateral centred on
the decoded marker.
"""

import argparse
import itertools
import json

import cv2
import numpy as np


def ordered_like(reference, candidate):
    reference = np.asarray(reference, dtype=np.float64)
    candidate = np.asarray(candidate, dtype=np.float64)
    ref_normalized = reference - reference.mean(axis=0)
    ref_normalized /= np.sqrt(np.mean(np.sum(ref_normalized**2, axis=1)))
    best = None
    for permutation in itertools.permutations(range(4)):
        trial = candidate[list(permutation)]
        trial_normalized = trial - trial.mean(axis=0)
        trial_normalized /= np.sqrt(np.mean(np.sum(trial_normalized**2, axis=1)))
        error = float(np.sum((trial_normalized - ref_normalized) ** 2))
        if best is None or error < best[0]:
            best = (error, trial)
    return best[1]


def geometric_marker_order(points):
    points = np.asarray(points, dtype=np.float64)
    sums = points.sum(axis=1)
    differences = np.diff(points, axis=1).ravel()
    top_left = points[np.argmin(sums)]
    bottom_right = points[np.argmax(sums)]
    top_right = points[np.argmin(differences)]
    bottom_left = points[np.argmax(differences)]
    # This physical marker's decoded order is BL, TL, TR, BR at its installed yaw.
    return np.asarray([bottom_left, top_left, top_right, bottom_right])


def detect(image_path, marker_id, threshold, allow_contour_only=False):
    image = cv2.imread(image_path)
    if image is None:
        raise RuntimeError(f"cannot read image: {image_path}")
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_ARUCO_ORIGINAL)
    corners, ids, _ = cv2.aruco.detectMarkers(image, dictionary)
    id_decoded = ids is not None and marker_id in ids.ravel()
    if id_decoded:
        index = list(ids.ravel()).index(marker_id)
        decoded = corners[index].reshape(4, 2).astype(np.float64)
        decoded_area = abs(float(cv2.contourArea(decoded.astype(np.float32))))
        decoded_center = decoded.mean(axis=0)
    elif not allow_contour_only:
        raise RuntimeError(f"ArUco ID {marker_id} was not detected")
    else:
        decoded = None
        decoded_area = None
        decoded_center = None

    binary = cv2.threshold(gray, threshold, 255, cv2.THRESH_BINARY_INV)[1]
    contours, _ = cv2.findContours(binary, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)
    candidates = []
    image_area = float(gray.shape[0] * gray.shape[1])
    for contour in contours:
        area = abs(float(cv2.contourArea(contour)))
        if id_decoded:
            if not 0.60 * decoded_area <= area <= 1.05 * decoded_area:
                continue
        elif not 0.05 * image_area <= area <= 0.25 * image_area:
            continue
        approximation = cv2.approxPolyDP(
            contour, 0.02 * cv2.arcLength(contour, True), True
        )
        if len(approximation) != 4 or not cv2.isContourConvex(approximation):
            continue
        points = approximation.reshape(4, 2).astype(np.float64)
        if id_decoded:
            center_limit = 0.20 * np.sqrt(decoded_area)
            if np.linalg.norm(points.mean(axis=0) - decoded_center) > center_limit:
                continue
        candidates.append((area, points))
    if not candidates:
        raise RuntimeError("the 100 mm black-square contour was not found")

    _, black_square = max(candidates, key=lambda item: item[0])
    black_square = (
        ordered_like(decoded, black_square)
        if id_decoded
        else geometric_marker_order(black_square)
    )
    area = abs(float(cv2.contourArea(black_square.astype(np.float32))))
    return {
        "image": image_path,
        "marker_id": int(marker_id),
        "id_decoded": bool(id_decoded),
        "corners_px": [[round(float(x), 3), round(float(y), 3)] for x, y in black_square],
        "area_px2": round(area, 3),
        "decoded_candidate_area_px2": None if decoded_area is None else round(decoded_area, 3),
    }


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("image")
    parser.add_argument("--marker-id", type=int, default=450)
    parser.add_argument("--threshold", type=int, default=100)
    parser.add_argument("--allow-contour-only", action="store_true")
    arguments = parser.parse_args()
    print(
        json.dumps(
            detect(
                arguments.image,
                arguments.marker_id,
                arguments.threshold,
                arguments.allow_contour_only,
            )
        )
    )


if __name__ == "__main__":
    main()
