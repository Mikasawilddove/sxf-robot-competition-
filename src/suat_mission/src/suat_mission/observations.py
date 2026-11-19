"""Strict JSON boundaries for future perception/placement observers, with no ROS import."""
import json
import math

from .contracts import ActionResult, Outcome
from .perception import Detection
from .world import SceneSnapshot


def decode_scene(payload, received_at, ros_now, max_age, calibration_id, min_frames):
    data = json.loads(payload)
    age = ros_now - float(data['stamp'])
    if not math.isfinite(age) or not 0 <= age <= max_age:
        raise ValueError('instance source timestamp is stale or in the future')
    if not calibration_id or data['calibration_id'] != calibration_id:
        raise ValueError('instance calibration version does not match')
    if type(data['support_frames']) is not int or data['support_frames'] < min_frames:
        raise ValueError('insufficient independent supporting frames')
    if (not isinstance(data['scene_id'], str) or not data['scene_id'].strip()
            or not isinstance(data['frame_id'], str) or not data['frame_id'].strip()
            or type(data['version']) is not int or data['version'] < 1):
        raise ValueError('invalid instance source identity/frame/version')
    objects = tuple(Detection(d['label'], float(d['score']), float(d['x']), float(d['y']),
                              float(d['width']), float(d['height']), d['object_id'],
                              d.get('surface', {}), tuple(d.get('occluded_by', ())),
                              d.get('graspable')) for d in data['objects'])
    if any(not d.valid() or not isinstance(d.object_id, str) or not d.object_id.strip() for d in objects):
        raise ValueError('invalid instance detection or missing physical object_id')
    if len({d.object_id for d in objects}) != len(objects):
        raise ValueError('duplicate physical object_id')
    if not isinstance(data.get('manipulation',{}),dict):
        raise ValueError('manipulation observation must be an object')
    return SceneSnapshot(data['scene_id'], data['version'], received_at - age, data['frame_id'], objects,
                         data.get('manipulation', {}),float(data['stamp']))


def decode_placement(payload, context, zone):
    data = json.loads(payload)
    if any(data[key] != expected for key, expected in
           (('run_id', context.run_id), ('goal_id', context.goal_id),
            ('object_id', context.object_id), ('scene_version', context.scene_version), ('zone', zone))):
        raise ValueError('placement observation does not match the active request')
    outcome = Outcome(data['outcome'])
    evidence = data.get('evidence', '')
    if not isinstance(evidence, str) or not evidence.strip():
        raise ValueError('placement observation lacks evidence')
    return ActionResult(context, outcome, evidence)
