"""Draw what the current mission observed, on the frame it actually used."""
import cv2
import numpy as np
import rospy
from sensor_msgs.msg import Image


COLORS = {'banana': (0, 220, 255), 'clock': (255, 180, 0),
          'teddybear': (0, 220, 0), 'cake': (255, 0, 255)}
WHITE = (240, 240, 240)
AMBER = (0, 180, 255)


class MissionOverlay:
    def __init__(self, name, bridge, draw):
        self.publisher = rospy.Publisher('/suat/mission/debug/' + name, Image, queue_size=1)
        self.bridge, self.draw = bridge, draw

    def render(self, msg, image, *args, save=None):
        # The display is optional and cannot fail an observation or a mission.
        try:
            connected = self.publisher.get_num_connections() > 0
            if not connected and save is None:
                return
            annotated = image.copy()
            self.draw(annotated, *args)
            if connected:
                output = self.bridge.cv2_to_imgmsg(annotated, 'bgr8')
                output.header = msg.header
                self.publisher.publish(output)
            if save is not None and not cv2.imwrite(str(save), annotated):
                rospy.logwarn('Could not save mission overlay %s', save)
        except Exception as exc:
            rospy.logwarn_throttle(5., 'Mission overlay unavailable: %s', exc)


def text(image, label, origin, color=WHITE, scale=.43):
    x, y = origin
    width = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, scale, 1)[0][0]
    x = max(2, min(int(x), image.shape[1] - width - 4))
    y = max(13, min(int(y), image.shape[0] - 4))
    cv2.putText(image, label, (x, y), cv2.FONT_HERSHEY_SIMPLEX, scale,
                (0, 0, 0), 3, cv2.LINE_AA)
    cv2.putText(image, label, (x, y), cv2.FONT_HERSHEY_SIMPLEX, scale,
                color, 1, cv2.LINE_AA)


def polygon(image, corners, color):
    points = np.rint(corners).astype(np.int32)
    cv2.polylines(image, [points], True, color, 2, cv2.LINE_AA)
    return int(points[:, 0].min()), int(points[:, 1].min())


def draw_board(image, view, markers, detections, zones, status, mapping, direction):
    if view is not None:
        polygon(image, view.corners, WHITE)
        width, height = view.normalized_size
        for i, zone in enumerate(zones):
            x = width * i / 3
            a, b = np.rint(view.to_image([(x, 0), (x, height - 1)])).astype(int).tolist()
            cv2.line(image, tuple(a), tuple(b), WHITE, 1)
            label = view.to_image([(x + width / 6, height - 1)])[0]
            text(image, zone, (label[0], label[1] + 16))
        for d in detections:
            color = COLORS.get(d.label, WHITE)
            left, top = d.x - d.width / 2, d.y - d.height / 2
            right, bottom = d.x + d.width / 2, d.y + d.height / 2
            x, y = polygon(image, view.to_image([(left, top), (right, top), (right, bottom), (left, bottom)]), color)
            zone = mapping.get(d.label)
            text(image, '%s %.2f%s' % (d.label, d.score, ' -> ' + zone if zone else ''), (x, y - 4), color)
    for marker in markers:
        x, y = polygon(image, marker['quad'], AMBER)
        detail = marker['zone'] or marker['text'] or '?'
        text(image, 'Marker %s  %s' % (detail, marker['source']), (x, y - 5), AMBER)
    text(image, 'BOARD | ' + status, (7, 18))
    text(image, direction, (7, 35))


def draw_scene(image, active, diagnostics, accepted, tool, hand, placement, target_top, floor, error=''):
    for d, box, obj, reason in diagnostics:
        color = COLORS.get(d.label, WHITE)
        left, top, right, bottom = (int(round(v)) for v in box)
        cv2.rectangle(image, (left, top), (right, bottom), color, 2)
        state = reason
        if obj is not None:
            surface = obj['surface']
            xyz = surface['arm_xyz_mm']
            state = ('published' if obj['object_id'] in accepted else
                     'warming %d/3' % obj['support_frames'])
            if not obj['graspable']:
                state += ' blocked'
            text(image, '%s | arm %.0f %.0f %.0f mm' % (obj['object_id'], *xyz),
                 (left, bottom + 14), color)
            if floor is not None:
                text(image, 'height %.0f mm | margin %.0f mm' %
                     (xyz[2] - floor, surface['edge_margin_mm']), (left, bottom + 29), color)
        text(image, '%s %.2f | %s' % (d.label, d.score, state), (left, top - 4), color)
    if tool.get('pixel') is not None:
        tool_pixel = tuple(int(round(value)) for value in tool['pixel'])
        cv2.drawMarker(image, tool_pixel, WHITE, cv2.MARKER_CROSS, 15, 2)
        text(image, 'Arm marker', (tool_pixel[0] + 9, tool_pixel[1]))
    text(image, 'GRASP | %s | target %s | %d published' %
         (active.get('phase') or 'waiting', active.get('label') or '-', len(accepted)), (7, 18))
    text(image, 'Tool %s | Hold %s | Top %s | Place %s' %
         (tool.get('state', '-'), hand.get('state', '-'), (target_top or {}).get('state', '-'),
          (placement or {}).get('zone', '-')), (7, 35))
    reason = error or (target_top or {}).get('reason')
    if reason:
        text(image, reason, (7, image.shape[0] - 29), AMBER)
