"""Bind navigation to the selected map path, geometry and layout versions."""
import math
from suat_mission.navigation.saved_grid import load_saved_grid
from suat_mission.navigation.map_identity import map_identity


def map_snapshot(msg, layout_version):
    """Identity of an unsaved mapping snapshot; never invent an image identity."""
    if not isinstance(layout_version,str) or not layout_version.strip():
        raise ValueError('mapping candidates require an explicit layout_version')
    info=msg.info; origin=info.origin; q=origin.orientation
    values=(info.resolution,origin.position.x,origin.position.y,origin.position.z,q.x,q.y,q.z,q.w)
    if (not all(type(v) in (int,float) and math.isfinite(v) for v in values)
            or info.resolution <= 0 or abs(origin.position.z) > 1e-6
            or max(abs(q.x),abs(q.y)) > 1e-6 or abs(q.z*q.z+q.w*q.w-1) > 1e-6
            or msg.header.frame_id.lstrip('/') != 'map'
            or type(info.width) is not int or type(info.height) is not int
            or min(info.width,info.height) <= 0 or info.width*info.height > 4000000
            or len(msg.data) != info.width*info.height
            or any(type(v) is not int or not -1 <= v <= 100 for v in msg.data)):
        raise ValueError('invalid mapping snapshot metadata or cells')
    return {'frame_id':'map','layout_version':layout_version,'resolution':float(info.resolution),
            'width':info.width,'height':info.height,'origin':[float(origin.position.x),float(origin.position.y)],
            'origin_yaw':math.atan2(2*q.w*q.z,1-2*q.z*q.z),
            'identity_method':'frame_layout_and_geometry',
            'occupancy_interpretation':'ROS_OccupancyGrid_-1_unknown_0_to_100',
            'status':'UNSAVED_MAP_SNAPSHOT_NOT_A_SAVED_MAP_BINDING'}


def load_binding(root):
    return {key:value for key,value in load_saved_grid(root).items() if key != 'data'}


def verify_map(msg, binding):
    map_identity(binding)
    if binding['origin_yaw'] != 0:
        raise ValueError('rotated saved maps are not supported by the current navigation geometry')
    origin = msg.info.origin
    q = origin.orientation
    values = (msg.info.resolution, origin.position.x, origin.position.y, origin.position.z, q.x, q.y, q.z, q.w)
    if not all(math.isfinite(v) for v in values):
        raise ValueError('non-finite map metadata')
    if (msg.header.frame_id.lstrip('/') != binding['frame_id'] or msg.info.width != binding['width']
            or msg.info.height != binding['height'] or abs(msg.info.resolution-binding['resolution']) > 1e-6
            or abs(origin.position.x-binding['origin'][0]) > 1e-6
            or abs(origin.position.y-binding['origin'][1]) > 1e-6
            or abs(origin.position.z) > 1e-6 or max(abs(q.x), abs(q.y), abs(q.z)) > 1e-6 or abs(abs(q.w)-1) > 1e-6):
        raise ValueError('live map geometry differs from the selected saved map')
    if (len(msg.data) != binding['width']*binding['height']
            or any(type(v) is not int or v not in (-1, 0, 100) for v in msg.data)):
        raise ValueError('invalid live trinary map cells')
    return dict(binding, data=msg.data)
