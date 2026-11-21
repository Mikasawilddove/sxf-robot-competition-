#!/usr/bin/env python3
"""Detect the block, map it into arm coordinates, and pick it up.

Geometry comes from a DLT fit collected on this robot (tools/fit_dlt.py).
The factory planar file (~/thefile.txt) does NOT match this camera mount -- it
predicts arm x=343 for a pose that was actually commanded at x=180 -- so it is
deliberately not used here.

Model:  cam_point = A @ arm_point + b       (mm, camera frame = pinhole)
        cam_point = ((u-cx)*d/fx, (v-cy)*d/fy, d)   with d = aligned depth

The block sits on a fixed support surface, so its top-face height BLOCK_Z is a
constant of the workspace; that is what lets a single (u, v) pixel invert to a
unique (x, y).

    rosrun suat_mission arm_grab_block.py --execute
"""
import argparse
import time

import cv2
import numpy as np
import rospy
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from swiftpro.msg import position, status

FX, FY, CX, CY = 604.3, 603.0, 330.1, 251.2

A = np.array([[0.044, -0.8683, 0.0222],
              [-0.8145, 0.0118, -0.208],
              [0.300, 0.0125, -0.9455]])
B = np.array([23.08, 109.02, 359.0])
A_INV = np.linalg.inv(A)

BRIDGE = CvBridge()


def pixel_to_arm(u, v, depth_mm, block_z):
    """Invert the projection given the block's known top-face height."""
    cam = np.array([(u - CX) * depth_mm / FX, (v - CY) * depth_mm / FY, depth_mm])
    # A[:, 0]*x + A[:, 1]*y = cam - B - A[:, 2]*z, solved on the u,v rows
    rhs = (cam - B - A[:, 2] * block_z)[:2]
    xy = np.linalg.solve(A[:2, :2], rhs)
    return float(xy[0]), float(xy[1])


def find_block(colour, depth, min_area=3000, near=495, far=548, v_max=340):
    """Segment the block primarily by depth.

    The white-face colour test proved fragile: once the cube is rotated the
    printed banana covers most of the face and the white blob collapses to a
    few hundred pixels. Depth banding is stable because the block is the only
    thing sitting at that range, and the arm body (300-450) and wall (650+)
    fall outside it.
    """
    hsv = cv2.cvtColor(colour, cv2.COLOR_BGR2HSV)
    white_ratio_hint = ((hsv[:, :, 1] < 75) & (hsv[:, :, 2] > 105))
    band = ((depth > near) & (depth < far)).astype(np.uint8) * 255
    band[v_max:, :] = 0
    band = cv2.morphologyEx(band, cv2.MORPH_CLOSE, np.ones((21, 21), np.uint8))
    band = cv2.morphologyEx(band, cv2.MORPH_OPEN, np.ones((7, 7), np.uint8))
    contours, _ = cv2.findContours(band, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    best = None
    for blob in contours:
        area = cv2.contourArea(blob)
        if area < min_area:
            continue
        x, y, w, h = cv2.boundingRect(blob)
        if w <= 0 or h <= 0:
            continue
        fill = area / float(w * h)
        if fill < 0.45 or not 0.4 < w / float(h) < 2.5:
            continue
        patch = white_ratio_hint[y:y + h, x:x + w]
        if patch.size and patch.mean() < 0.10:
            continue
        moments = cv2.moments(blob)
        candidate = {'u': moments['m10'] / moments['m00'],
                     'v': moments['m01'] / moments['m00'], 'area': area,
                     'box': (x, y, w, h), 'fill': fill}
        if best is None or area > best['area']:
            best = candidate
    if best is None:
        return None, band
    best['depth'] = int(depth[int(best['v']), int(best['u'])])

    # The blob covers the whole cube (front face + top face). The suction cup
    # must land on the TOP face, so isolate it: the camera looks down, which
    # makes the horizontal top face consistently farther than the vertical
    # front face. Taking the far tail of the depth distribution inside the box
    # picks the top face out without relying on colour or view angle.
    x, y, w, h = best['box']
    patch = depth[y:y + h, x:x + w]
    valid = patch[patch > 0]
    if valid.size >= 200:
        threshold = np.percentile(valid, 72)
        top_mask = ((patch >= threshold) & (patch > 0)).astype(np.uint8)
        top_mask = cv2.morphologyEx(top_mask, cv2.MORPH_OPEN, np.ones((5, 5), np.uint8))
        moments = cv2.moments(top_mask, binaryImage=True)
        if moments['m00'] > 300:
            best['u'] = x + moments['m10'] / moments['m00']
            best['v'] = y + moments['m01'] / moments['m00']
            best['depth'] = int(depth[int(best['v']), int(best['u'])])
            best['top_face'] = True
    return best, band


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--block-z', type=float, default=-132.0,
                    help='arm z of the block top face (from fit_dlt.py)')
    ap.add_argument('--y-offset', type=float, default=32.6,
                    help='empirical y correction: measured plate centre vs real cup')
    ap.add_argument('--safe-z', type=float, default=70.0)
    ap.add_argument('--step', type=float, default=45.0)
    ap.add_argument('--observe', default='150,130,60',
                    help='arm pose that keeps the block unobstructed for detection')
    ap.add_argument('--execute', action='store_true')
    args = ap.parse_args()

    rospy.init_node('arm_grab_block', anonymous=True)
    pos_pub = rospy.Publisher('/position_write_topic', position, queue_size=1)
    status_pub = rospy.Publisher('/swiftpro_status_topic', status, queue_size=1)
    pump_pub = rospy.Publisher('/pump_topic', status, queue_size=1)
    time.sleep(1.5)

    if args.execute:
        for _ in range(3):
            status_pub.publish(status(1))
            time.sleep(0.2)
        time.sleep(0.8)
        ox, oy, oz = (float(v) for v in args.observe.split(','))
        pos_pub.publish(position(ox, oy, oz))
        print('observe pose (%.0f, %.0f, %.0f)' % (ox, oy, oz), flush=True)
        time.sleep(3.0)

    colour_msg = rospy.wait_for_message('/camera/color/image_raw', Image, timeout=5)
    depth_msg = rospy.wait_for_message('/camera/aligned_depth_to_color/image_raw', Image, timeout=5)
    colour = BRIDGE.imgmsg_to_cv2(colour_msg, 'bgr8')
    depth = BRIDGE.imgmsg_to_cv2(depth_msg, 'passthrough')
    cv2.imwrite('/tmp/grab_detect.jpg', colour)

    block, mask = find_block(colour, depth)
    cv2.imwrite('/tmp/grab_mask.png', mask)
    if block is None:
        print('NO BLOCK FOUND')
        return 1
    debug = colour.copy()
    bx, by, bw, bh = block['box']
    cv2.rectangle(debug, (bx, by), (bx + bw, by + bh), (0, 200, 255), 2)
    cv2.drawMarker(debug, (int(block['u']), int(block['v'])), (0, 255, 0),
                   cv2.MARKER_CROSS, 20, 2)
    cv2.imwrite('/tmp/grab_detect_annotated.jpg', debug)

    x, y = pixel_to_arm(block['u'], block['v'], block['depth'], args.block_z)
    y += args.y_offset
    print('block pixel uv=(%.1f, %.1f) depth=%d area=%.0f' %
          (block['u'], block['v'], block['depth'], block['area']))
    print('block arm  x=%.1f y=%.1f z=%.1f' % (x, y, args.block_z))

    if not args.execute:
        print('dry run (pass --execute to move)')
        return 0

    def move(z, dwell=3.0):
        pos_pub.publish(position(x, y, z))
        time.sleep(dwell)
        print('  move -> (%.1f, %.1f, %.1f)' % (x, y, z), flush=True)

    move(args.safe_z)
    z = args.safe_z
    while z > args.block_z:
        z = max(args.block_z, z - args.step)
        move(z)
    move(args.block_z, dwell=3.4)

    pump_pub.publish(status(1))
    print('pump ON', flush=True)
    time.sleep(2.5)

    move(args.block_z + 100, dwell=3.6)
    move(args.safe_z, dwell=3.0)

    image = BRIDGE.imgmsg_to_cv2(
        rospy.wait_for_message('/camera/color/image_raw', Image, timeout=5), 'bgr8')
    cv2.imwrite('/tmp/grab_final.jpg', image)
    print('done; pump left ON (rostopic pub -1 /pump_topic swiftpro/status "{status: 0}")')
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
