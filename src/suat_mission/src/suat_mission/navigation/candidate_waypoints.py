"""Candidate station records, separate from Start_A and accepted task stations."""
import copy
import math
from suat_mission.navigation.map_identity import map_identity, validate_map_identity


def _validate_fields(name, pose, metadata):
    if not isinstance(name,str) or not name.strip() or name == 'Start_A':
        raise ValueError('a task station name is required; use the separate Start_A recorder for Start_A')
    if (not isinstance(pose,(list,tuple)) or len(pose) != 3
            or any(type(v) not in (int,float) or not math.isfinite(v) for v in pose)):
        raise ValueError('candidate pose requires finite [x,y,yaw] in m/rad')
    if (not isinstance(metadata,dict)
            or any(not isinstance(metadata.get(key),str) or not metadata[key].strip() for key in ('client_time','purpose','notes'))
            or not isinstance(metadata.get('source_images'),list)
            or any(not isinstance(v,str) or not v.strip() for v in metadata['source_images'])):
        raise ValueError('candidate metadata requires client_time, purpose, notes and explicit source_images')


def make_candidate(name, pose, binding, metadata, source_frame='map', alignment=None, expected_version=None):
    _validate_fields(name,pose,metadata)
    identity=map_identity(binding)
    converted=[float(v) for v in pose]
    version=binding['alignment_version']
    if source_frame == 'field':
        if alignment is None or not isinstance(expected_version,str) or not expected_version.strip():
            raise ValueError('field candidate requires alignment and its explicit version')
        from suat_mission.navigation.field_alignment import transform_pose
        converted=transform_pose(alignment,pose,binding,expected_version=expected_version)
        version=alignment['alignment_version']
    elif source_frame != 'map':
        raise ValueError('candidate source frame must be field or map')
    converted[2]=(converted[2]+math.pi) % (2*math.pi)-math.pi
    return {'schema':1,'name':name,'frame_id':'map','units':'m','angle_units':'rad','pose':converted,
            'source_frame':source_frame,'source_pose':list(pose),'map_identity':identity,
            'layout_version':binding['layout_version'],'alignment_version':version,
            'pose_source':'field_conversion_candidate' if source_frame == 'field' else 'recorded_map_pose_candidate',
            'metadata':copy.deepcopy(metadata),'confirmation_level':'candidate',
            'status':'CANDIDATE_NOT_FIELD_VALIDATED','field_validated':False,'hardware_ready':False}


def make_mapping_candidate(name, pose, snapshot, metadata):
    """Keep an unsaved map snapshot explicitly unbound until static-map re-observation."""
    _validate_fields(name,pose,metadata)
    if (not isinstance(snapshot,dict) or snapshot.get('status') != 'UNSAVED_MAP_SNAPSHOT_NOT_A_SAVED_MAP_BINDING'
            or not snapshot.get('layout_version') or not snapshot.get('grid_sha256') or 'image_sha256' in snapshot):
        raise ValueError('an explicit unsaved mapping snapshot is required')
    return {'schema':1,'name':name,'frame_id':'map','units':'m','angle_units':'rad','pose':list(pose),
            'source_frame':'map','source_pose':list(pose),'map_identity':None,'map_snapshot':copy.deepcopy(snapshot),
            'layout_version':snapshot['layout_version'],'alignment_version':'UNALIGNED',
            'pose_source':'mapping_snapshot_candidate','metadata':copy.deepcopy(metadata),
            'confirmation_level':'candidate','status':'UNBOUND_MAPPING_CANDIDATE',
            'field_validated':False,'hardware_ready':False}


def candidate_document(existing, record, binding, replace=False):
    """Add a station to a bound candidate collection; never mutate the caller's old data."""
    mapping=binding is None
    if mapping:
        if record.get('status') != 'UNBOUND_MAPPING_CANDIDATE' or record.get('map_identity') is not None:
            raise ValueError('mapping collections require unbound snapshot candidates')
        identity=None
    else:
        identity=validate_map_identity(record,binding)
    if (record.get('confirmation_level') != 'candidate' or record.get('field_validated') is not False
            or record.get('hardware_ready') is not False):
        raise ValueError('only unvalidated candidate records belong in this collection')
    if existing is None:
        data={'schema':2,'frame_id':'map','map_identity':identity,'waypoints':{},'candidate_metadata':{},
              'layout_version':record['layout_version'],
              'status':'UNBOUND_MAPPING_CANDIDATE' if mapping else 'CANDIDATE_NOT_FIELD_VALIDATED',
              'field_validated':False,'hardware_ready':False}
    else:
        if (not isinstance(existing,dict) or existing.get('schema') != 2
                or not isinstance(existing.get('waypoints'),dict) or not isinstance(existing.get('candidate_metadata'),dict)
                or existing.get('field_validated') is not False or existing.get('hardware_ready') is not False
                or existing.get('status') not in ('UNBOUND_MAPPING_CANDIDATE','CANDIDATE_NOT_FIELD_VALIDATED')):
            raise ValueError('existing legacy waypoint data is preserved; choose a new candidate output file')
        if mapping:
            if (existing.get('status') != 'UNBOUND_MAPPING_CANDIDATE' or existing.get('map_identity') is not None
                    or existing.get('layout_version') != record['layout_version']):
                raise ValueError('mapping candidates cannot be mixed with bound resources or another layout')
        else:
            validate_map_identity(existing,binding)
        data=copy.deepcopy(existing)
    name=record['name']
    if name in data['waypoints'] and not replace:
        raise ValueError('candidate already exists; explicit replacement and a saved backup are required')
    data['waypoints'][name]=copy.deepcopy(record['pose'])
    data['candidate_metadata'][name]=copy.deepcopy(record)
    return data
