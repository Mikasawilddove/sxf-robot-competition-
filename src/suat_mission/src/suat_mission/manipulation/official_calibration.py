"""Optional factory XY mapping; DLT stays the default pending field comparison.

Coefficient order/formula: auto_match/nodes/runtask.py:127-165. A raw four-value
file cannot describe its camera installation, image size or calibration height.
"""
import copy
import math
from pathlib import Path

from suat_mission.manipulation.calibration import PlanarCalibration


def load_profile(filename):
    import yaml
    path=Path(filename).expanduser().resolve()
    profile=yaml.safe_load(path.read_text(encoding='utf-8'))
    planes=profile.get('planes') if isinstance(profile,dict) else None
    if not isinstance(planes,list) or not planes:
        raise ValueError('official calibration profile requires height-specific planes')
    models=[];file_heights={}
    for plane in planes:
        top=plane.get('top_z_mm')
        if top is not None and (type(top) not in (int,float) or not math.isfinite(top)):
            raise ValueError('official plane top_z_mm must be a measured number or null for comparison only')
        contact_z=plane.get('contact_z_mm')
        labels=plane.get('labels')
        if not isinstance(labels,list) or not labels or any(
                label not in ('cake','clock','teddybear','banana') for label in labels):
            raise ValueError('official plane requires explicit object labels')
        source=Path(plane['file']).expanduser()
        if not source.is_absolute():source=path.parent/source
        source=source.resolve()
        previous=file_heights.setdefault(str(source),top)
        if previous!=top:
            raise ValueError('one coefficient file cannot represent two different calibration heights')
        calibration=PlanarCalibration.from_values(source.read_text(encoding='utf-8-sig').split())
        size=plane.get('image_size')
        if size is not None and (not isinstance(size,list) or len(size)!=2 or
                                 any(type(v) is not int or v<=0 for v in size)):
            raise ValueError('official plane image_size must be calibrated [width,height]')
        if contact_z is not None and (top is None or size is None or
                                      type(contact_z) not in (int,float) or not math.isfinite(contact_z)):
            raise ValueError('contact_z_mm requires a finite arm value and calibrated height and image size')
        stat=source.stat()
        models.append({'labels':labels,'top_z_mm':top,'image_size':size,'contact_z_mm':contact_z,
                       'coefficients':[calibration.kx,calibration.bx,calibration.ky,calibration.by],
                       'source_file':str(source),'source_size_bytes':stat.st_size,
                       'source_mtime_ns':stat.st_mtime_ns})
    return {'profile_file':str(path),'planes':models,'physical_accuracy':'NOT_TESTED_BY_IMPORT',
            'pixel_convention':'original_color_image_u_v',
            'formula':'arm_x=kx*v+bx; arm_y=ky*u+by'}


def bind_official_geometry(values,profile_file,source='dlt'):
    """Freeze one selection for both the observation and execution nodes."""
    if source not in ('dlt','official-planar'):raise ValueError('unknown grasp XY source')
    result=copy.deepcopy(values)
    if profile_file:
        official=load_profile(profile_file)
        result['official_planar']=official
        stamps='-'.join(str(p['source_mtime_ns']) for p in official['planes'])
        result['calibration_id']=result['calibration_id']+'|official-comparison:'+source+':'+stamps
    if source=='official-planar' and not result.get('official_planar'):
        raise ValueError('official-planar requires --calibration with a factory coefficient profile')
    result['grasp_xy_source']=source
    return result


def project_top(values,label,pixel,observed_top_z,image_size,require_height=True):
    official=values.get('official_planar')
    if not official:raise ValueError('official planar model is not loaded')
    tolerance=float(values.get('height_tolerance_mm',12.))
    matches=[p for p in official['planes'] if label in p['labels'] and
             (not require_height or (p['top_z_mm'] is not None and
                                     abs(p['top_z_mm']-observed_top_z)<=tolerance))]
    if len(matches)!=1:
        raise ValueError('no unique official calibration for '+label+' at top Z '+str(observed_top_z)+' mm')
    model=matches[0]
    if model.get('image_size') is not None and list(image_size or [])!=model['image_size']:
        raise ValueError('image size differs from official calibration; no implicit resizing')
    if not isinstance(pixel,(list,tuple)) or len(pixel)!=2:
        raise ValueError('official projection requires an observed interior top pixel')
    calibration=PlanarCalibration.from_values(model['coefficients'])
    x,y=calibration.project(*pixel)
    return [x,y,observed_top_z],{'source':'official_planar','source_file':model['source_file'],
        'coefficients':model['coefficients'],'top_z_mm':model['top_z_mm'],
        'contact_z_mm':model.get('contact_z_mm'),
        'observed_top_z_mm':observed_top_z,'grasp_pixel':list(pixel),
        'height_applicable':model['top_z_mm'] is not None and abs(model['top_z_mm']-observed_top_z)<=tolerance,
        'physical_accuracy':official['physical_accuracy']}
