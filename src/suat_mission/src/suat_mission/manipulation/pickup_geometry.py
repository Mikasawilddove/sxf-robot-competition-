"""Height-aware top surfaces and one tool-offset convention, all lengths in mm."""
import math
from pathlib import Path


ORDINARY_BLOCKS = ('banana', 'clock', 'teddybear')


class GeometryUnavailable(ValueError):
    pass


class PickupGeometry:
    def __init__(self, values):
        self.values = values
        self.calibration_id = str(values.get('calibration_id', '')).strip()
        if not self.calibration_id:
            raise GeometryUnavailable('pickup geometry requires calibration_id')
        controller_limits=values.get('workspace_source')=='controller_ik'
        self.bounds = None if controller_limits and not values.get('arm_bounds_mm') else self.vector('arm_bounds_mm',6)
        self.work = None if controller_limits and not values.get('reliable_work_area_mm') else self.vector('reliable_work_area_mm',4)
        if ((self.bounds and any(self.bounds[i]>=self.bounds[i+1] for i in (0,2,4))) or
                (self.work and any(self.work[i]>=self.work[i+1] for i in (0,2)))):
            raise GeometryUnavailable('workspace minima must be smaller than maxima')
        self.support_z = self.number('support_z_mm')
        self.cup_offset = self.number('cup_offset_mm')
        self.cup_radius = self.number('cup_radius_mm')
        if self.cup_radius <= 0:
            raise GeometryUnavailable('cup_radius_mm must describe the real suction cup')

    @classmethod
    def load(cls, filename):
        import yaml
        return cls(yaml.safe_load(Path(filename).read_text(encoding='utf-8')))

    def number(self, name):
        value = self.values.get(name)
        if type(value) not in (int, float) or not math.isfinite(value):
            raise GeometryUnavailable('missing/invalid pickup geometry: '+name)
        return float(value)

    def vector(self, name, count):
        value = self.values.get(name)
        if not isinstance(value, (list, tuple)) or len(value) != count:
            raise GeometryUnavailable('missing pickup geometry vector: '+name)
        if any(type(v) not in (int, float) or not math.isfinite(v) for v in value):
            raise GeometryUnavailable('nonfinite pickup geometry: '+name)
        return tuple(float(v) for v in value)

    def check_pose(self, xyz, *, pickup_label=None):
        if len(xyz) != 3 or any(not math.isfinite(v) for v in xyz):
            raise GeometryUnavailable('invalid arm pose')
        if self.bounds:
            # The old X minimum was sampled high above single-layer blocks.
            # Forward pickup reach at the requested XYZ is checked by the controller.
            lower_x = 0. if pickup_label in ORDINARY_BLOCKS else self.bounds[0]
            upper_x = (self.values.get('cake_forward_limit_mm', self.bounds[1])
                       if pickup_label == 'cake' else self.bounds[1])
            limits = (lower_x, upper_x, *self.bounds[2:])
            if not all(limits[2*i] <= xyz[i] <= limits[2*i+1] for i in range(3)):
                raise GeometryUnavailable('arm pose outside measured workspace: '+str(xyz))
        return list(xyz)

    def point(self, detection, require_fit=True):
        surface = detection.surface
        if not isinstance(surface, dict) or surface.get('top_face') is not True:
            raise GeometryUnavailable('current instance does not have an observed top surface')
        if surface.get('calibration_id', self.calibration_id) != self.calibration_id:
            raise GeometryUnavailable('top surface belongs to another calibration')
        xyz = surface.get('grasp_point_arm_mm', surface.get('arm_xyz_mm'))
        normal = surface.get('normal_arm')
        if not isinstance(xyz, (list, tuple)) or len(xyz) != 3 or not isinstance(normal, (list, tuple)) or len(normal) != 3:
            raise GeometryUnavailable('top surface requires arm-frame XYZ and normal')
        if any(type(v) not in (int, float) or not math.isfinite(v) for v in (*xyz, *normal)):
            raise GeometryUnavailable('nonfinite observed surface')
        norm = math.sqrt(sum(v*v for v in normal))
        tilt = float(self.values.get('max_top_tilt_deg', 15.))
        if norm == 0 or abs(normal[2])/norm < math.cos(math.radians(tilt)):
            raise GeometryUnavailable('detected face is not a horizontal top')
        margin = surface.get('edge_margin_mm')
        if require_fit and (type(margin) not in (int, float) or not math.isfinite(margin) or margin < self.cup_radius):
            raise GeometryUnavailable('suction disk is not contained inside the observed top')
        return [float(v) for v in xyz]

    def payload_point(self,detection):
        point=self.point(detection,False)
        anchor=detection.surface.get('attachment_point_arm_mm')
        if anchor is not None and len(anchor)==3 and all(math.isfinite(v) for v in anchor):return list(anchor)
        return point

    def contact(self, detection):
        return self.check_pose(self.contact_target(detection), pickup_label=detection.label)

    def expected_pickup_top_z(self, label, support_z):
        layer = 2 if label == 'cake' else 1
        height = float(self.values.get('object_height_mm', {}).get(label, 100.))
        return support_z + layer * height

    def contact_target(self, detection, *, projection=None):
        """Observed contact target before evaluating the stopped arm's reach."""
        point, evidence = self.grasp_projection(detection) if projection is None else projection
        tolerance = float(self.values.get('height_tolerance_mm', 12.))
        support=float(detection.surface.get('support_z_mm',self.support_z))
        if abs(point[2]-self.expected_pickup_top_z(detection.label, support)) > tolerance:
            raise GeometryUnavailable('observed top has the wrong layer height')
        measured_z = (evidence['official_model'].get('contact_z_mm')
                      if evidence['selected_source'] == 'official-planar' else None)
        return [point[0], point[1], point[2]+self.cup_offset if measured_z is None else measured_z]

    def grasp_source(self, label):
        if label in self.values.get('rgbd_pickup_labels', ()):
            return 'rgbd-top'
        return self.values.get('grasp_xy_source','dlt')

    def grasp_projection(self, detection):
        """Select only contact XY; RGB-D surface/height/holding checks remain."""
        point=self.point(detection)
        evidence={'selected_source':self.grasp_source(detection.label),'calibration_id':self.calibration_id}
        if evidence['selected_source'] == 'rgbd-top':
            evidence['rgbd_point_arm_mm'] = point
            return point, evidence
        evidence['dlt_point_arm_mm'] = point
        if self.values.get('official_planar'):
            from suat_mission.manipulation.official_calibration import project_top
            try:
                official,details=project_top(self.values,detection.label,
                    detection.surface.get('grasp_pixel'),point[2],detection.surface.get('image_size'),
                    require_height=evidence['selected_source']=='official-planar')
                evidence.update(official_point_arm_mm=official,official_model=details,
                                xy_difference_mm=[official[0]-point[0],official[1]-point[1]])
            except ValueError as exc:
                evidence['official_unavailable']=str(exc)
        if evidence['selected_source']=='official-planar':
            if 'official_point_arm_mm' not in evidence:
                raise GeometryUnavailable(evidence.get('official_unavailable','official calibration not loaded'))
            if self.values.get('geometry_method') == 'rigid_rgbd':
                outline = detection.surface.get('top_polygon_px')
                if not outline or len(outline) < 3:
                    raise GeometryUnavailable('missing measured top outline')
                import cv2
                import numpy as np

                kx, bx, ky, by = evidence['official_model']['coefficients']
                point_xy = tuple(official[:2])

                def signed_distance(contour):
                    u, v = np.asarray(contour).T
                    arm = np.column_stack((kx*v+bx, ky*u+by)).astype(np.float32)
                    return cv2.pointPolygonTest(arm, point_xy, True)

                margin = min([signed_distance(outline)] +
                             [-signed_distance(hole) for hole in detection.surface.get('top_holes_px', ())])
                if margin < self.cup_radius:
                    raise GeometryUnavailable('observed top does not fit the suction cup in official calibration')
                evidence['official_edge_margin_mm'] = margin
            return evidence['official_point_arm_mm'],evidence
        return point,evidence

    def base_adjustment(self, detection, *, projection=None):
        if self.work is None or detection.label in ORDINARY_BLOCKS:
            return 0., 0.  # The controller checks the stopped three-dimensional pickup plan.
        (x,y,_),_=self.grasp_projection(detection) if projection is None else projection
        xmin,xmax,ymin,ymax = self.work
        if detection.label=='cake' and self.values.get('reachable_top_selection') is True:
            xmax=self.number('cake_forward_limit_mm')
        return x-min(xmax,max(xmin,x)), y-min(ymax,max(ymin,y))

    def pickup_options(self, detection):
        """Keep the measured central point first, then other supported cup placements."""
        yield detection
        source = self.grasp_source(detection.label)
        if (self.values.get('reachable_top_selection') is not True
                or not (source in ('dlt', 'rgbd-top') or (source == 'official-planar' and detection.label == 'cake'))):
            return
        from dataclasses import replace
        for option in detection.surface.get('alternate_suction_points', ()):
            yield replace(detection, surface=dict(detection.surface, **option))

    def release(self, zone, label, slot=0):
        poses = self.values.get('drop_slots_arm_mm', {}).get(zone, [])
        if not 0 <= slot < len(poses):
            raise GeometryUnavailable('no recorded free placement slot: '+zone+':'+str(slot))
        x,y = poses[slot][:2]
        support = float(self.values.get('drop_support_z_mm', {}).get(zone, self.support_z))
        height = float(self.values.get('object_height_mm', {}).get(label, 100.))
        bottom_clearance = float(self.values.get('release_bottom_clearance_mm', 100.))
        # At release: bottom = flange_z - tool_offset - object_height.
        return self.check_pose([x,y,support+height+bottom_clearance+self.cup_offset])

    def transport_release(self,zone,label,slot):
        """Optional lower-score fallback, only at recorded same-zone poses."""
        poses=self.values.get('transport_slots_arm_mm',{}).get(zone,[])
        region=self.values.get('transport_regions_arm_mm',{}).get(zone)
        if not 0<=slot<len(poses) or not isinstance(region,list) or len(region)!=4:
            raise GeometryUnavailable('no recorded transport fallback slots/region for '+zone)
        if not all(type(v) in (int,float) and math.isfinite(v) for v in region) or not (
                region[0]<region[1] and region[2]<region[3]):
            raise GeometryUnavailable('invalid recorded transport fallback region')
        x,y=poses[slot][:2]
        support=float(self.values.get('transport_support_z_mm',{}).get(zone,self.support_z))
        height=float(self.values.get('object_height_mm',{}).get(label,100.))
        z=support+height+float(self.values.get('release_bottom_clearance_mm',100.))+self.cup_offset
        return self.check_pose([x,y,z]),region,support


def fit_top_surface(depth_mm, camera_matrix, bbox, geometry, label, expected_top_z_mm=None,
                    require_fit=True, center_top=False):
    """Fit a gravity-consistent RGB-D plane and an inscribed suction point.

    Called by the production camera adapter. Missing depth or extrinsics is an
    unavailable observation, never an instruction to use the pattern centre.
    """
    import cv2
    import numpy as np
    values = geometry.values
    transform = np.asarray(values.get('camera_to_arm_mm'), dtype=float)
    if transform.shape not in ((3,4),(4,4)) or not np.isfinite(transform).all():
        raise GeometryUnavailable('camera_to_arm_mm must be a measured 3x4/4x4 transform')
    a,b = transform[:3,:3], transform[:3,3]
    affine=values.get('geometry_method')=='affine_depth_baseline'
    if not affine and (not np.allclose(a.T@a,np.eye(3),atol=1.e-3) or abs(np.linalg.det(a)-1.)>1.e-3):
        raise GeometryUnavailable('camera-to-arm matrix must be a rigid measured transform, not an affine fit')
    k = np.asarray(camera_matrix, dtype=float).reshape(3,3)
    fx,fy,cx,cy = k[0,0],k[1,1],k[0,2],k[1,2]
    if fx <= 0 or fy <= 0:
        raise GeometryUnavailable('invalid CameraInfo intrinsics')
    x0,y0,x1,y1 = map(int,bbox)
    h,w = depth_mm.shape
    x0,y0,x1,y1 = max(0,x0),max(0,y0),min(w,x1),min(h,y1)
    if center_top:
        seed = (x0,y0,x1,y1)
        px,py = (x1-x0)//2,(y1-y0)//2
        x0,y0,x1,y1 = max(0,x0-px),max(0,y0-py),min(w,x1+px),min(h,y1+py)
    yy,xx = np.mgrid[y0:y1,x0:x1]
    z = depth_mm[y0:y1,x0:x1].astype(float)
    valid = np.isfinite(z)&(z>0)
    pixels = np.column_stack((xx[valid],yy[valid]))
    z = z[valid]
    if len(z)<20:
        raise GeometryUnavailable('not enough aligned depth on this target')
    camera = np.column_stack(((pixels[:,0]-cx)*z/fx,(pixels[:,1]-cy)*z/fy,z))
    arm = camera @ a.T+b
    # Keep held/placed surfaces observable: cake is not always still on layer 2.
    # contact() enforces the picking layer, separately from surface measurement.
    close = np.ones(len(arm),dtype=bool) if geometry.bounds is None else (
        (arm[:,2]>=geometry.bounds[4]-geometry.cup_offset)&(arm[:,2]<=geometry.bounds[5]))
    if expected_top_z_mm is not None:
        close &= abs(arm[:,2]-float(expected_top_z_mm)) <= float(values.get('height_tolerance_mm',12.))
    arm,pixels = arm[close],pixels[close]
    if len(arm)<20:
        raise GeometryUnavailable('no depth points at the requested top height')
    sample = arm[np.linspace(0,len(arm)-1,min(1500,len(arm)),dtype=int)]
    rng = np.random.RandomState(0)
    residual = float(values.get('plane_residual_mm',5.))
    cosine = math.cos(math.radians(float(values.get('max_top_tilt_deg',15.))))
    best = None
    for _ in range(40):
        p,q,r = sample[rng.choice(len(sample),3,replace=False)]
        normal = np.cross(q-p,r-p); norm=np.linalg.norm(normal)
        if norm<1e-9 or abs(normal[2])/norm<cosine:
            continue
        normal/=norm
        mask=abs((sample-p)@normal)<=residual
        score=int(mask.sum())
        if best is None or score>best[0]:
            best=(score,normal,float(p@normal))
    if best is None or best[0]<20:
        raise GeometryUnavailable('no supported horizontal top plane')
    inlier=abs(arm@best[1]-best[2])<=residual
    points=arm[inlier]; centre=points.mean(axis=0)
    normal=np.linalg.svd(points-centre,full_matrices=False)[2][-1]
    if normal[2]<0: normal=-normal
    if normal[2]<cosine:
        raise GeometryUnavailable('refined plane is not a top surface')
    supported=pixels[inlier].astype(np.int32)
    mask=np.zeros((h,w),dtype=np.uint8);mask[supported[:,1],supported[:,0]]=1
    # Close only tiny sampling gaps; do not bridge an occluded face with its hull.
    mask=cv2.morphologyEx(mask,cv2.MORPH_CLOSE,np.ones((3,3),np.uint8))
    _,components,stats,_=cv2.connectedComponentsWithStats(mask,8)
    if len(stats)<2:raise GeometryUnavailable('no connected observed top surface')
    if center_top:
        sx0,sy0,sx1,sy1 = seed
        counts=np.bincount(components[sy0:sy1,sx0:sx1].ravel(),minlength=len(stats))
        if not counts[1:].any():
            raise GeometryUnavailable('depth face does not overlap the recognized block')
        component=1+int(counts[1:].argmax())
    else:
        component=1+int(stats[1:,cv2.CC_STAT_AREA].argmax())
    mask=(components==component).astype(np.uint8)
    if center_top:
        selected = mask[supported[:,1],supported[:,0]]!=0
        supported,points = supported[selected],points[selected]
        if len(points)<20:
            raise GeometryUnavailable('not enough depth on centered ordinary block')
        centre = points.mean(axis=0)
        normal = np.linalg.svd(points-centre,full_matrices=False)[2][-1]
        if normal[2]<0:
            normal = -normal
        if normal[2]<cosine:
            raise GeometryUnavailable('centered ordinary block does not have a horizontal top')
    distance=cv2.distanceTransform(mask,cv2.DIST_L2,5)
    v,u=np.unravel_index(int(distance.argmax()),distance.shape)
    plane_d=float(centre@normal)
    def intersect(pixel):
        ray=np.array([(pixel[0]-cx)/fx,(pixel[1]-cy)/fy,1.])
        denominator=float(normal@(a@ray))
        if abs(denominator)<1e-9: raise GeometryUnavailable('camera ray parallel to top')
        return a@ray*((plane_d-float(normal@b))/denominator)+b
    point=intersect((u,v))
    contours=cv2.findContours(mask,cv2.RETR_LIST,cv2.CHAIN_APPROX_SIMPLE)[-2]
    segments = []
    for contour in contours:
        polygon = [intersect(p) for p in contour.reshape(-1,2)]
        segments.extend(zip(polygon, polygon[1:]+polygon[:1]))
    if not segments:
        raise GeometryUnavailable('observed top has no boundary')
    starts,ends=np.asarray(segments).transpose(1,0,2)
    edges=ends-starts
    edge_sq=np.maximum(np.sum(edges*edges,axis=1),1e-12)

    def boundary_distance(xyz):
        frac=np.clip(np.sum((xyz-starts)*edges,axis=1)/edge_sq,0.,1.)
        return float(np.linalg.norm(xyz-starts-edges*frac[:,None],axis=1).min())

    if center_top:
        width = float(values.get('object_width_mm', {}).get(label,100.))
        outline = cv2.convexHull(cv2.findNonZero(mask)).reshape(-1,2)
        physical_xy = np.asarray([intersect(p)[:2] for p in outline],np.float32)
        (cx_arm,cy_arm),size,_ = cv2.minAreaRect(physical_xy)
        if min(size)<.7*width or max(size)>1.4*width:
            raise GeometryUnavailable('cannot center on an incompletely observed ordinary block')
        point = np.array([cx_arm,cy_arm,(plane_d-normal[0]*cx_arm-normal[1]*cy_arm)/normal[2]])
        projection = np.linalg.solve(a,point-b)
        u,v = (int(round(fx*projection[0]/projection[2]+cx)),
               int(round(fy*projection[1]/projection[2]+cy)))
        if not (0<=u<w and 0<=v<h and mask[v,u]):
            raise GeometryUnavailable('ordinary block center has no observed support')

    margin=boundary_distance(point)
    if require_fit and margin<geometry.cup_radius:
        raise GeometryUnavailable('visible top is too narrow for the suction cup')
    outline = max(contours, key=cv2.contourArea)
    result = {'top_face':True,'calibration_id':geometry.calibration_id,
            'arm_xyz_mm':point.tolist(),'grasp_point_arm_mm':point.tolist(),
            'normal_arm':normal.tolist(),'edge_margin_mm':margin,
            'plane_residual_mm':float(np.median(abs((points-centre)@normal))),
            'support_pixels':len(points),'top_polygon_px':outline.reshape(-1,2).tolist(),
            'top_holes_px':[c.reshape(-1,2).tolist() for c in contours if c is not outline and len(c) >= 3],
            'grasp_pixel':[int(u),int(v)],
            'image_size':[int(w),int(h)],
            'plane_normal_arm':normal.tolist(),'plane_offset_arm_mm':plane_d,
            'support_polygon_arm_mm':[intersect(p).tolist() for p in cv2.convexHull(supported).reshape(-1,2)],
            'geometry_method':'affine_depth_baseline' if affine else 'rigid_rgbd'}
    if require_fit and values.get('reachable_top_selection') is True:
        # Use the actual depth component and all its boundary contours, including
        # holes. A convex hull could let the cup bridge an occluded segment.
        options=[]
        min_clearance=geometry.cup_radius+float(values.get('alignment_tolerance_mm',5.))
        for dv in range(-45,46,15):
            for du in range(-45,46,15):
                row,col=v+dv,u+du
                if (not (du or dv) or not (0<=row<h and 0<=col<w)
                        or distance[row,col]<3.):
                    continue
                xyz=intersect((col,row))
                if center_top and np.linalg.norm(xyz-point)>2.*geometry.cup_radius:
                    continue
                edge_mm=boundary_distance(xyz)
                if edge_mm>=min_clearance:
                    options.append(dict(grasp_point_arm_mm=xyz.tolist(),
                        grasp_pixel=[int(col),int(row)],edge_margin_mm=edge_mm))
        result['alternate_suction_points']=options
    return result


def observe_payload_volume(depth_mm, camera_matrix, geometry, arm_pose_mm, label, object_id):
    """Positive free-space rays through the expected payload box; occlusion stays unknown."""
    import numpy as np
    values=geometry.values
    transform=np.asarray(values.get('camera_to_arm_mm'),dtype=float)
    if transform.shape not in ((3,4),(4,4)) or not np.isfinite(transform).all():
        raise GeometryUnavailable('gripper observation requires measured camera_to_arm_mm')
    rotation,origin=transform[:3,:3],transform[:3,3]
    if values.get('geometry_method')!='affine_depth_baseline' and (
            not np.allclose(rotation.T@rotation,np.eye(3),atol=1.e-3) or abs(np.linalg.det(rotation)-1.)>1.e-3):
        raise GeometryUnavailable('gripper observation requires a rigid transform')
    k=np.asarray(camera_matrix,dtype=float).reshape(3,3)
    height=float(values.get('object_height_mm',{}).get(label,100.))
    half=float(values.get('object_width_mm',{}).get(label,100.))/2.
    top=np.asarray(arm_pose_mm,dtype=float)-[0.,0.,geometry.cup_offset]
    # Exclude the cup/contact edge and an outer border; inspect the payload interior.
    lower=top+[-half*.6,-half*.6,-height*.8]
    upper=top+[half*.6,half*.6,-height*.2]
    yy,xx=np.mgrid[:depth_mm.shape[0],:depth_mm.shape[1]]
    rays=np.stack(((xx-k[0,2])/k[0,0],(yy-k[1,2])/k[1,1],np.ones_like(xx)),axis=-1)@rotation.T
    parallel=np.abs(rays)<1.e-9
    safe=np.where(parallel,1.,rays)
    a=(lower-origin)/safe;b=(upper-origin)/safe
    near=np.minimum(a,b);far=np.maximum(a,b)
    inside=(origin>=lower)&(origin<=upper)
    near=np.where(parallel,np.where(inside,-np.inf,np.inf),near)
    far=np.where(parallel,np.where(inside,np.inf,-np.inf),far)
    enter=np.max(near,axis=-1);leave=np.min(far,axis=-1)
    intersects=(leave>np.maximum(0.,enter))
    depth=np.asarray(depth_mm,dtype=float)
    valid=intersects&np.isfinite(depth)&(depth>0)
    tolerance=float(values.get('plane_residual_mm',5.))
    empty=int(np.sum(valid&(depth>leave+tolerance)))
    occupied=int(np.sum(valid&(depth>=enter-tolerance)&(depth<=leave+tolerance)))
    total=int(intersects.sum());visible=int(valid.sum())
    # Candidate observation parameters; their field results are recorded, not presumed.
    enough=(total>=int(values.get('gripper_min_rays',30)) and
            empty>=float(values.get('gripper_free_fraction',.8))*total and occupied==0)
    return {'object_id':object_id,'state':'empty' if enough else 'unknown',
            'evidence':'aligned depth rays through measured payload volume',
            'free_rays':empty,'occupied_rays':occupied,'valid_rays':visible,'total_rays':total}


def planar_top_surface(top_mask, geometry, plane_key, observed_top_z_mm):
    """Keep the agreed height-specific planar baseline; mask must be the real top.

    This does not classify a bbox as a top or infer height from a class name.
    The camera adapter supplies that observation and a matching calibrated plane.
    """
    import cv2
    import numpy as np
    from suat_mission.manipulation.calibration import PlanarCalibration
    model=geometry.values.get('plane_models',{}).get(plane_key)
    if not model:raise GeometryUnavailable('height-specific planar model missing: '+plane_key)
    height=float(model['top_z_mm'])
    if abs(float(observed_top_z_mm)-height)>float(geometry.values.get('height_tolerance_mm',12.)):
        raise GeometryUnavailable('observed surface height differs from the selected planar calibration')
    calibration=PlanarCalibration.from_values(model['coefficients'])
    mask=(np.asarray(top_mask)>0).astype(np.uint8)
    if mask.ndim!=2 or not np.any(mask):raise GeometryUnavailable('observed top mask is empty')
    roi=model['pixel_roi'];ys,xs=np.nonzero(mask)
    if xs.min()<roi[0] or xs.max()>roi[2] or ys.min()<roi[1] or ys.max()>roi[3]:
        raise GeometryUnavailable('top surface falls outside the calibrated image area')
    metric=cv2.distanceTransform(mask,cv2.DIST_L2,5)
    v,u=np.unravel_index(int(metric.argmax()),metric.shape)
    def project(p):
        return np.array([calibration.kx*p[1]+calibration.bx,
                         calibration.ky*p[0]+calibration.by,height])
    point=project((u,v));contours=cv2.findContours(mask,cv2.RETR_LIST,cv2.CHAIN_APPROX_SIMPLE)[-2]
    margins=[]
    for contour in contours:
        polygon=[project(p) for p in contour.reshape(-1,2)]
        for left,right in zip(polygon,polygon[1:]+polygon[:1]):
            edge=right-left;length=float(edge@edge)
            t=min(1.,max(0.,float((point-left)@edge)/length)) if length else 0.
            margins.append(float(np.linalg.norm(point-(left+t*edge))))
    margin=min(margins) if margins else 0.
    if margin<geometry.cup_radius:raise GeometryUnavailable('observed top cannot contain the suction disk')
    return {'top_face':True,'calibration_id':geometry.calibration_id,'arm_xyz_mm':point.tolist(),
            'grasp_point_arm_mm':point.tolist(),'normal_arm':[0.,0.,1.],
            'normal_source':'height-specific calibrated horizontal plane','edge_margin_mm':margin,
            'grasp_pixel':[int(u),int(v)],'geometry_method':'planar','plane_key':plane_key}
