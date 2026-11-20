"""Candidate field->map rigid alignment. Coordinates are metres and radians.

Field origin is the inside A corner, +X toward D and +Y toward B. Fitting
never estimates an applied scale and never declares field accuracy accepted.
"""
import copy
import math
import numpy as np
from suat_mission.navigation.map_identity import map_identity, validate_map_identity


def wrap_angle(value):
    if type(value) not in (int, float) or not math.isfinite(value):
        raise ValueError('angle must be finite radians')
    return (value+math.pi) % (2*math.pi)-math.pi


def _vector(value, size, label):
    if (not isinstance(value, (list, tuple)) or len(value) != size
            or any(type(v) not in (int, float) or not math.isfinite(v) for v in value)):
        raise ValueError(label+' requires '+str(size)+' finite numbers')
    return np.asarray(value, dtype=float)


def fit_alignment(references, check_points, binding, version, measurement):
    """Fit p_map=R(theta)*p_field+t; keep checks out of the fit."""
    map_identity(binding)
    if not isinstance(version,str) or not version.strip() or version == 'UNALIGNED':
        raise ValueError('a named candidate alignment version is required')
    if (not isinstance(measurement,dict)
            or any(not isinstance(measurement.get(k),str) or not measurement[k].strip() for k in ('client_time','method'))
            or not isinstance(measurement.get('source_images'),list)
            or any(not isinstance(v,str) or not v.strip() for v in measurement['source_images'])):
        raise ValueError('measurement requires client_time, method and explicit source_images')
    if not isinstance(references,list) or len(references) < 3 or not isinstance(check_points,list) or not check_points:
        raise ValueError('at least three dispersed references and an independent check point are required')
    names=set(); locations={'field':set(),'map':set()}
    for row in references+check_points:
        if not isinstance(row,dict) or not isinstance(row.get('id'),str) or not row['id'].strip() or row['id'] in names:
            raise ValueError('observations require distinct named references and checks')
        names.add(row['id'])
        uncertainty=row.get('uncertainty_m')
        if type(uncertainty) not in (int,float) or not math.isfinite(uncertainty) or uncertainty < 0:
            raise ValueError('each observation requires its nonnegative uncertainty_m')
        for frame in ('field','map'):
            point=tuple(_vector(row.get(frame),2,frame+' observation'))
            if point in locations[frame]:
                raise ValueError('reference/check positions must be distinct and independent')
            locations[frame].add(point)
    field = np.asarray([row['field'] for row in references], dtype=float)
    mapped = np.asarray([row['map'] for row in references], dtype=float)
    center_f, center_m = field.mean(axis=0), mapped.mean(axis=0)
    if np.linalg.matrix_rank(field-center_f) < 2 or np.linalg.matrix_rank(mapped-center_m) < 2:
        raise ValueError('reference geometry is degenerate or collinear')
    u, _, vt = np.linalg.svd((field-center_f).T @ (mapped-center_m))
    rotation = vt.T @ u.T
    if np.linalg.det(rotation) < 0:
        vt[-1] *= -1
        rotation = vt.T @ u.T
    translation = center_m-rotation @ center_f
    theta = wrap_angle(float(math.atan2(rotation[1,0],rotation[0,0])))
    record = {'schema':1,'direction':'field_to_map','units':'m','angle_units':'rad','scale':1.0,
              'field_axes':{'origin':'inside_A_corner','positive_x':'A_to_D','positive_y':'A_to_B'},
              'map_identity':map_identity(binding),'alignment_version':version,
              'theta_rad':theta,'translation_m':translation.tolist(),
              'references':copy.deepcopy(references),'check_points':copy.deepcopy(check_points),
              'measurement':copy.deepcopy(measurement),'status':'CANDIDATE_NOT_FIELD_VALIDATED',
              'confirmation_level':'candidate','hardware_ready':False}
    record['diagnostic_map_to_field_span_ratio']=float(np.linalg.norm(mapped-center_m)/np.linalg.norm(field-center_f))
    record['scale_note']='diagnostic ratio only; applied scale stays 1; investigate map/measurement scale before field acceptance'
    for key, rows in (('reference',references),('check',check_points)):
        residuals=[]
        for row in rows:
            delta=rotation @ np.asarray(row['field'],dtype=float)+translation-np.asarray(row['map'],dtype=float)
            residuals.append({'id':row['id'],'dx_m':float(delta[0]),'dy_m':float(delta[1]),
                              'distance_m':float(np.linalg.norm(delta))})
        record[key+'_residuals']=residuals
        record[key+'_rms_m']=math.sqrt(sum(row['distance_m']**2 for row in residuals)/len(residuals))
    return record


def validate_alignment(alignment, binding, expected_version=None):
    validate_map_identity(alignment,binding)
    expected={'schema':1,'direction':'field_to_map','units':'m','angle_units':'rad','scale':1.0}
    if any(alignment.get(key) != value for key,value in expected.items()):
        raise ValueError('alignment must be a schema 1 field_to_map rigid transform in m/rad, with scale=1')
    if not isinstance(alignment.get('alignment_version'),str) or not alignment['alignment_version'].strip():
        raise ValueError('missing alignment version')
    if expected_version is not None and alignment.get('alignment_version') != expected_version:
        raise ValueError('alignment version differs from requested version')
    wrap_angle(alignment.get('theta_rad'))
    _vector(alignment.get('translation_m'),2,'translation_m')
    return alignment


def transform_pose(alignment, pose, binding, inverse=False, expected_version=None):
    """Convert [x,y,yaw]; an optional expected fitted version must match exactly."""
    validate_alignment(alignment,binding,expected_version)
    x,y,yaw=_vector(pose,3,'pose')
    theta=alignment['theta_rad']
    tx,ty=alignment['translation_m']
    c,s=math.cos(theta),math.sin(theta)
    if inverse:
        dx,dy=x-tx,y-ty
        return [float(c*dx+s*dy),float(-s*dx+c*dy),wrap_angle(float(yaw-theta))]
    return [float(c*x-s*y+tx),float(s*x+c*y+ty),wrap_angle(float(yaw+theta))]


def static_transform_args(alignment):
    """Arguments for Noetic's static map → field transform publisher."""
    x, y = alignment['translation_m']
    half_yaw = alignment['theta_rad'] / 2
    return [str(x), str(y), '0', '0', '0', str(math.sin(half_yaw)),
            str(math.cos(half_yaw)), 'map', 'field']
