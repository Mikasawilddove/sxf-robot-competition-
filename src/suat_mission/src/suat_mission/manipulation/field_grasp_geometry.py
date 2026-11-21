"""Actual RGB-D surfaces and orange calibration-reference observations.

The affine DLT is the robot's existing baseline, not a rigid camera transform.
All returned coordinates retain that provenance. No class box becomes a top
face without depth-plane support.
"""
import math
import cv2
import numpy as np

from suat_mission.manipulation.pickup_geometry import GeometryUnavailable,fit_top_surface
from suat_mission.vision.experimental_vision import measure_layer


class FieldGraspGeometry:
    def __init__(self,geometry):
        self.geometry=geometry
        values = geometry.values
        self.method = values.get('geometry_method', 'affine_depth_baseline')
        if self.method == 'rigid_rgbd':
            transform = np.asarray(values.get('camera_to_arm_mm'), dtype=float)
            if transform.shape != (3, 4) or not np.isfinite(transform).all():
                raise GeometryUnavailable('rigid_rgbd requires a finite 3x4 camera-to-arm transform')
            rotation = transform[:, :3]
            if (not np.allclose(rotation.T @ rotation, np.eye(3), atol=.001)
                    or abs(np.linalg.det(rotation)-1.) > .001):
                raise GeometryUnavailable('camera_to_arm_mm must be a rigid right-handed transform')
            self.a = rotation.T
            self.b = -self.a @ transform[:, 3]
        else:
            self.method = 'affine_depth_baseline'
            self.a=np.asarray(values['legacy_camera_A'],dtype=float).reshape(3,3)
            self.b=np.asarray(values['legacy_camera_B_mm'],dtype=float)
        self.initial_b=self.b.copy();self.samples=[];self.frozen=False
        self._apply()

    def _apply(self):
        inverse=np.linalg.inv(self.a)
        self.geometry.values['camera_to_arm_mm']=np.column_stack((inverse,-inverse@self.b)).tolist()
        self.geometry.values['geometry_method']=self.method

    @staticmethod
    def camera_points(depth,k,stride=1):
        yy,xx=np.mgrid[0:depth.shape[0]:stride,0:depth.shape[1]:stride]
        z=depth[::stride,::stride].astype(float)
        valid=np.isfinite(z)&(z>0)
        return np.column_stack(((xx[valid]-k[0,2])*z[valid]/k[0,0],
                                (yy[valid]-k[1,2])*z[valid]/k[1,1],z[valid]))

    def floor(self,depth,k):
        camera=self.camera_points(depth,k,8)
        points=(camera-self.b)@np.linalg.inv(self.a).T
        prior=self.geometry.support_z
        points=points[(points[:,2]>prior-150.)&(points[:,2]<prior+100.)]
        if len(points)<100:raise GeometryUnavailable('floor is not visible in aligned depth')
        rng=np.random.RandomState(7);candidates=[]
        residual=float(self.geometry.values.get('plane_residual_mm',5.))
        cosine=math.cos(math.radians(float(self.geometry.values.get('max_top_tilt_deg',15.))))
        for _ in range(40):
            p,q,r=points[rng.choice(len(points),3,replace=False)]
            n=np.cross(q-p,r-p);length=np.linalg.norm(n)
            if length<1e-8 or abs(n[2])/length<cosine:continue
            n/=length;mask=abs((points-p)@n)<=residual
            if mask.sum()>=max(100,len(points)*.12):
                candidates.append((float(np.median(points[mask,2])),int(mask.sum())))
        if not candidates:raise GeometryUnavailable('no supported floor plane; height remains unknown')
        return min(candidates,key=lambda p:p[0])[0]

    def observe_tool(self,image,depth,k,active):
        pose=active.get('arm_pose_mm')
        if not pose:return {'state':'unknown','reason':'no measured arm pose'}
        predicted=self.a@np.asarray(pose)+self.b
        if predicted[2]<=0:return {'state':'unknown','reason':'tool outside view'}
        uv=np.array([k[0,0]*predicted[0]/predicted[2]+k[0,2],k[1,1]*predicted[1]/predicted[2]+k[1,2]])
        hsv=cv2.cvtColor(image,cv2.COLOR_BGR2HSV)
        mask=cv2.inRange(hsv,np.array([4,100,60],np.uint8),np.array([25,255,255],np.uint8))
        # Separate the foreground reference from similarly colored objects before finding contours.
        reference_depth=np.isfinite(depth)&(depth>0)&(abs(depth-predicted[2])<=120.)
        mask[~reference_depth]=0
        mask=cv2.morphologyEx(mask,cv2.MORPH_OPEN,np.ones((3,3),np.uint8))
        contours=cv2.findContours(mask,cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)[-2]
        options=[]
        for contour in contours:
            area=cv2.contourArea(contour);x,y,w,h=cv2.boundingRect(contour)
            if not 30<=area<=3000 or not .4<=w/max(h,1)<=2.5 or area/(w*h)<.65:continue
            moment=cv2.moments(contour)
            if not moment['m00']:continue
            u,v=moment['m10']/moment['m00'],moment['m01']/moment['m00']
            distance=float(np.linalg.norm(np.array([u,v])-uv))
            if distance>120:continue
            px,py=int(round(u)),int(round(v))
            patch=depth[max(0,py-3):py+4,max(0,px-3):px+4]
            valid=patch[np.isfinite(patch)&(patch>0)]
            if len(valid)<12:continue
            d=float(np.median(valid))
            if abs(d-predicted[2])>120.:continue
            camera=np.array([(u-k[0,2])*d/k[0,0],(v-k[1,2])*d/k[1,1],d])
            options.append((distance,camera,[float(u),float(v)],max(w,h)/2.))
        options.sort(key=lambda item:item[0])
        if not options or (len(options)>1 and options[1][0]-options[0][0]<15.):
            return {'state':'unknown','reason':'orange reference not uniquely observed',
                    'predicted_pixel':uv.tolist()}
        error_px,camera,pixel,radius_px=options[0]
        if active.get('phase','').startswith('calibration_view') and not self.frozen:
            if not self.samples or min(math.dist(pose,p) for p,c in self.samples)>=25.:
                self.samples.append((list(pose),camera.tolist()))
            if len(self.samples)>=2:
                offsets=np.array([np.asarray(c)-self.a@np.asarray(p) for p,c in self.samples])
                fitted=offsets.mean(axis=0);error=float(np.max(np.linalg.norm(offsets-fitted,axis=1)))
                if error<=float(self.geometry.values.get('calibration_pair_residual_mm',10.)):
                    self.b=fitted;self.frozen=True;self._apply()
                else:return {'state':'unknown','reason':'two measured references disagree','residual_mm':error}
        measured=np.linalg.solve(self.a,camera-self.b)
        cup=measured-[0.,0.,self.geometry.cup_offset]
        return {'state':'observed','reference':'orange_plate_center_as_legacy_dlt',
                'pixel':pixel,'reference_arm_mm':measured.tolist(),'cup_point_arm_mm':cup.tolist(),
                'view_margin_mm':float((radius_px+error_px)*camera[2]/max(k[0,0],k[1,1])),
                'measured_pose_residual_mm':float(np.linalg.norm(measured-np.asarray(pose)))}

    def calibration(self):
        return {'status':'translation_refined' if self.frozen else 'baseline_not_refined',
                'samples':len(self.samples),'legacy_camera_B_mm':self.b.tolist(),
                'reference_pairs':[{'joint_fk_reference_arm_mm':p,'observed_reference_camera_mm':c} for p,c in self.samples],
                'translation_change_mm':(self.b-self.initial_b).tolist(),
                'physical_accuracy':'NOT_INDEPENDENTLY_VERIFIED'}

    def observe_layer(self, depth, k, bbox, floor, label):
        x0, y0, x1, y1 = map(int, bbox)
        height, width = depth.shape
        x0, y0, x1, y1 = max(0, x0), max(0, y0), min(width, x1), min(height, y1)
        patch = depth[y0:y1, x0:x1]
        if patch.size == 0:
            return None, None
        yy, xx = np.mgrid[y0:y1, x0:x1]
        ax, ay, az, shift = self.geometry.values['camera_to_arm_mm'][2]
        rise = patch * (ax * (xx - k[0, 2]) / k[0, 0] + ay * (yy - k[1, 2]) / k[1, 1] + az)
        rise += shift - floor
        rise[~np.isfinite(patch) | (patch <= 0)] = np.nan
        block_height = float(self.geometry.values.get('object_height_mm', {}).get(label, 100.0))
        return measure_layer(rise, (0, 0, patch.shape[1], patch.shape[0]), block_height, block_height)

    def surface(self,depth,k,bbox,label,floor,active,tool):
        carrying=(active.get('object_id') and active.get('label')==label and active.get('phase') in
                  ('lift','hold_view','transport','carrying','observe_placement','place_approach',
                   'release_height','withdraw','verify_placement'))
        expected = None if carrying else self.geometry.expected_pickup_top_z(label, floor)
        aligning=active.get('label')==label and active.get('phase') in ('approach','align_tool','contact')
        surface=fit_top_surface(depth,k,bbox,self.geometry,label,expected_top_z_mm=expected,
                                require_fit=not (carrying or aligning),
                                center_top=bool(active.get('center_ordinary_top') and label!='cake'
                                                and not (carrying or aligning)))
        surface.update(support_z_mm=floor,geometry_revision=2 if self.frozen else 1)
        if not (carrying or aligning):
            layer, rise_mm = self.observe_layer(depth, k, bbox, floor, label)
            # The fitted physical top defines the layer. Print-box depth is diagnostic.
            surface['observed_layer'] = {'index_from_zero': layer, 'rise_mm': rise_mm}
        if carrying and tool.get('state')=='observed':
            cup=np.asarray(tool['cup_point_arm_mm']);n=np.asarray(surface['plane_normal_arm'])
            point=cup-n*(n@cup-surface['plane_offset_arm_mm'])
            polygon=np.ascontiguousarray(np.asarray(surface['support_polygon_arm_mm'])[:,:2],dtype=np.float32)
            if cv2.pointPolygonTest(polygon.reshape(-1,1,2),(float(point[0]),float(point[1])),False)>=0:
                surface['attachment_point_arm_mm']=point.tolist()
        return surface

    def locked_top(self, depth, k, reference, label):
        """Refit the remaining top of the selected stationary block from this depth frame."""
        polygon=np.asarray(reference['polygon_px'],dtype=np.int32)
        if (reference['image_size']!=[depth.shape[1],depth.shape[0]]
                or polygon.ndim!=2 or polygon.shape[1]!=2 or len(polygon)<3):
            raise GeometryUnavailable('saved top contour does not match this image')
        x,y,w,h=cv2.boundingRect(polygon)
        mask=np.zeros(depth.shape,dtype=np.uint8)
        cv2.fillPoly(mask,[polygon],1)
        expected=float(reference['top_z_mm'])
        surface=fit_top_surface(np.where(mask,depth,0.),k,(x,y,x+w,y+h),self.geometry,label,
                                expected_top_z_mm=expected,require_fit=False)
        return {key:surface[key] for key in ('plane_normal_arm','plane_offset_arm_mm','support_pixels')}

    def raised_payload_depth(self, depth, k, cup_arm_mm, label):
        """Use the measured depth of the payload below the cup, including tilted blocks."""
        values = self.geometry.values
        width = float(values.get('object_width_mm', {}).get(label, 100.))
        height = float(values.get('object_height_mm', {}).get(label, 100.))
        gap = float(values.get('height_tolerance_mm', 12.))
        radius = width / math.sqrt(2.) + self.geometry.cup_radius
        cup = np.asarray(cup_arm_mm, dtype=float)
        raw = depth[::2,::2]
        valid = np.isfinite(raw) & (raw > 0.)
        arm = (self.camera_points(depth,k,2)-self.b) @ np.linalg.inv(self.a).T
        relative = arm-cup
        mask = np.zeros(raw.shape,np.uint8)
        mask[valid] = ((abs(relative[:,0])<=radius)&(abs(relative[:,1])<=radius)
                       &(relative[:,2]<=-gap)&(relative[:,2]>=-height-gap))
        connected = cv2.morphologyEx(mask,cv2.MORPH_CLOSE,np.ones((3,3),np.uint8))
        count, ids, stats, _ = cv2.connectedComponentsWithStats(connected,8)
        for component in sorted(range(1,count),key=lambda i:-stats[i,cv2.CC_STAT_AREA]):
            cluster = arm[(ids[valid]==component)&(mask[valid]!=0)]
            if len(cluster)<int(values.get('gripper_min_rays',30)):
                continue
            lo,hi = np.percentile(cluster,[5.,95.],axis=0)
            if np.any((hi-lo)[:2]<width*.4):
                continue
            center = np.median(cluster,axis=0)
            center[2] = hi[2]
            return {'point_arm_mm':center.tolist(),'support_pixels':len(cluster)}
        raise GeometryUnavailable('no block-sized raised depth component below the cup')

    def placement(self,image,depth,k,active,objects,floor):
        zone=active.get('zone')
        if not zone or active.get('phase')!='observe_placement':return None
        widths=self.geometry.values.get('object_width_mm',{})
        half_width=float(widths.get(active.get('label'),100.))/2.
        margin=math.sqrt(2)*half_width
        hsv=cv2.cvtColor(image,cv2.COLOR_BGR2HSV)
        mask=cv2.inRange(hsv,np.array([0,90,60],np.uint8),np.array([10,255,255],np.uint8))
        mask|=cv2.inRange(hsv,np.array([170,90,60],np.uint8),np.array([179,255,255],np.uint8))
        camera=self.camera_points(depth,k)
        valid=np.isfinite(depth)&(depth>0)
        arm=np.zeros((*depth.shape,3),float);arm[valid]=(camera-self.b)@np.linalg.inv(self.a).T
        mask[(~valid)|(abs(arm[:,:,2]-floor)>12.)]=0
        contours=cv2.findContours(mask,cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)[-2]
        if not contours:return None
        contour=max(contours,key=cv2.contourArea)
        if cv2.contourArea(contour)<500:return None
        boundary=contour.reshape(-1,2);polygon=arm[boundary[:,1],boundary[:,0],:2].astype(np.float32)
        polygon=cv2.convexHull(polygon).reshape(-1,2)
        lo=polygon.min(axis=0);hi=polygon.max(axis=0);options=[]
        for x in np.arange(lo[0],hi[0],10.):
            for y in np.arange(lo[1],hi[1],10.):
                # The carried block may be rotated inside the measured red floor.
                if cv2.pointPolygonTest(polygon.reshape(-1,1,2),(float(x),float(y)),True)<margin:continue
                covered=True
                for dx in (-half_width,0.,half_width):
                    for dy in (-half_width,0.,half_width):
                        cam=self.a@np.array([x+dx,y+dy,floor])+self.b
                        if cam[2]<=0:covered=False;break
                        u=int(round(k[0,0]*cam[0]/cam[2]+k[0,2]));v=int(round(k[1,1]*cam[1]/cam[2]+k[1,2]))
                        if not (0<=u<mask.shape[1] and 0<=v<mask.shape[0] and mask[v,u]):covered=False;break
                    if not covered:break
                if not covered:continue
                occupied=False
                for obj in objects:
                    if obj['object_id']==active.get('object_id'):continue
                    point=obj.get('surface',{}).get('arm_xyz_mm')
                    other_margin=math.sqrt(2)*float(widths.get(obj['label'],100.))/2.
                    if point and math.hypot(point[0]-x,point[1]-y)<margin+other_margin:occupied=True;break
                if not occupied:options.append([float(x),float(y)])
        options.sort(key=lambda p:math.hypot(p[0]-200.,p[1]))
        return {'zone':zone,'support_z_mm':floor,'polygon_arm_mm':polygon.tolist(),
                'candidates_arm_mm':options,'source':'observed_red_floor_surface'}
