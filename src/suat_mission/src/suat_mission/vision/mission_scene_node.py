#!/usr/bin/env python3
"""Core-owned grasp observations; does not change the B-line camera adapter."""
import json
import math
from pathlib import Path
import threading
import time
import uuid
from dataclasses import replace
from types import SimpleNamespace

import cv2
import message_filters
import numpy as np
import rospy
import tf2_ros
from cv_bridge import CvBridge
from sensor_msgs.msg import Image,CameraInfo
from std_msgs.msg import String

from suat_mission.vision.experimental_vision import ExperimentalVision
from suat_mission.manipulation.field_grasp_geometry import FieldGraspGeometry
from suat_mission.manipulation.pickup_geometry import PickupGeometry,GeometryUnavailable,observe_payload_volume


class SceneNode:
    def __init__(self):
        root=Path(rospy.get_param('~workspace_root'))
        filename=Path(rospy.get_param('~geometry_file',str(root/'data/calibration/field_20261117/pickup_geometry_active.yaml')))
        self.geometry=PickupGeometry.load(filename);self.measurement=FieldGraspGeometry(self.geometry)
        self.active={};self.lock=threading.Lock();self.bridge=CvBridge()
        self.tf=tf2_ros.Buffer();self.listener=tf2_ros.TransformListener(self.tf)
        templates={name:[cv2.imread(str(root/'src/suat_mission/assets/templates'/(name+'.png')))]
                   for name in ('cake','clock','teddybear','banana')}
        self.detector=ExperimentalVision(templates,banana_saturation_min=55)
        self.scene_id=uuid.uuid4().hex;self.version=0;self.last=0.;self.next_id=0;self.tracks={}
        self.source_stamp=0.;self.source_frames=0
        self.pub=rospy.Publisher('/suat/perception/grasp_instances',String,queue_size=1)
        self.status=rospy.Publisher('/suat/perception/grasp_status',String,queue_size=1,latch=True)
        self.active_sub=rospy.Subscriber('/suat/manipulation/active',String,self.on_active,queue_size=1)
        color=message_filters.Subscriber('/camera/color/image_raw',Image,queue_size=1)
        depth=message_filters.Subscriber('/camera/aligned_depth_to_color/image_raw',Image,queue_size=1)
        info=message_filters.Subscriber('/camera/color/camera_info',CameraInfo,queue_size=1)
        self.sync=message_filters.ApproximateTimeSynchronizer([color,depth,info],6,.12)
        self.sync.registerCallback(self.frame)

    def on_active(self,msg):
        try:
            value=json.loads(msg.data)
            with self.lock:self.active=value
        except ValueError:pass

    def associate(self,label,position,point,active,used):
        selected=active.get('object_id');phase=active.get('phase','')
        carrying=phase in ('lift','hold_view','transport','carrying','observe_placement','place_approach',
                           'release_height','withdraw','verify_placement')
        if carrying and selected in self.tracks and label==active.get('label') and selected not in used:
            expected=active.get('arm_pose_mm')
            if expected and math.dist(point,[expected[0],expected[1],expected[2]-self.geometry.cup_offset])<70.:
                key=selected
            else:key=None
        else:key=None
        if key is None:
            options=sorted((math.dist(position[:2],v['position'][:2]),key) for key,v in self.tracks.items()
                           if v['label']==label and key not in used and math.dist(position[:2],v['position'][:2])<.07)
            if len(options)>1 and options[1][0]-options[0][0]<.01:return None
            key=options[0][1] if options else None
        if key is None:
            self.next_id+=1;key=label+'-'+str(self.next_id)
            self.tracks[key]={'label':label,'position':position,'count':0}
        track=self.tracks[key];now=time.monotonic()
        count=track['count']+1 if now-track.get('last_seen',now)<1.5 else 1
        track.update(position=position,count=count,last_seen=now)
        used.add(key)
        return key

    def frame(self,color,depth_msg,info):
        now=time.monotonic()
        if now-self.last<.35 or not self.lock.acquire(False):return
        self.last=now
        try:
            stamp=color.header.stamp.to_sec()
            if stamp<=self.source_stamp:return
            self.source_stamp=stamp;self.source_frames+=1
            image=self.bridge.imgmsg_to_cv2(color,'bgr8')
            depth=self.bridge.imgmsg_to_cv2(depth_msg,'passthrough').astype(float)
            if depth_msg.encoding=='32FC1':depth*=1000.
            elif depth_msg.encoding!='16UC1':raise GeometryUnavailable('unknown aligned depth units')
            if depth.shape!=image.shape[:2]:raise GeometryUnavailable('RGB and aligned depth sizes differ')
            k=np.asarray(info.K,dtype=float).reshape(3,3)
            active=dict(self.active)
            if color.header.stamp.to_sec()<float(active.get('stamp',0.)):return
            tool=self.measurement.observe_tool(image,depth,k,active)
            floor=self.measurement.floor(depth,k)
            objects=[];used=set()
            # Camera->map is used for identity; no camera-position buckets.
            tr=self.tf.lookup_transform('map',color.header.frame_id,color.header.stamp,rospy.Duration(.15)).transform
            q=tr.rotation
            rotation=np.array([[1-2*(q.y*q.y+q.z*q.z),2*(q.x*q.y-q.z*q.w),2*(q.x*q.z+q.y*q.w)],
                               [2*(q.x*q.y+q.z*q.w),1-2*(q.x*q.x+q.z*q.z),2*(q.y*q.z-q.x*q.w)],
                               [2*(q.x*q.z-q.y*q.w),2*(q.y*q.z+q.x*q.w),1-2*(q.x*q.x+q.y*q.y)]])
            translation=np.array([tr.translation.x,tr.translation.y,tr.translation.z])
            detections=self.detector.detect(image)
            if not any(d.label=='clock' for d in detections):
                lab=cv2.cvtColor(image,cv2.COLOR_BGR2LAB)
                lab[:,:,0]=cv2.createCLAHE(clipLimit=3.,tileGridSize=(8,8)).apply(lab[:,:,0])
                enhanced=cv2.resize(cv2.cvtColor(lab,cv2.COLOR_LAB2BGR),None,fx=2.,fy=2.,interpolation=cv2.INTER_CUBIC)
                detections.extend(replace(d,x=d.x/2.,y=d.y/2.,width=d.width/2.,height=d.height/2.)
                    for d in self.detector.detect(enhanced) if d.label=='clock')
            for d in detections:
                box=[d.x-d.width/2,d.y-d.height/2,d.x+d.width/2,d.y+d.height/2]
                try:surface=self.measurement.surface(depth,k,box,d.label,floor,active,tool)
                except GeometryUnavailable:continue
                if self.geometry.values.get('official_planar'):
                    try:
                        _,comparison=self.geometry.grasp_projection(SimpleNamespace(label=d.label,surface=surface))
                        surface['calibration_comparison']=comparison
                    except GeometryUnavailable as exc:
                        surface['calibration_comparison']={'selected_source':'official-planar','error':str(exc)}
                point=surface['arm_xyz_mm']
                camera=self.measurement.a@np.asarray(point)+self.measurement.b
                position=(rotation@(camera/1000.)+translation).tolist()
                key=self.associate(d.label,position,point,active,used)
                if key is None:continue
                objects.append({'object_id':key,'label':d.label,'score':d.score,'x':d.x,'y':d.y,
                    'width':d.width,'height':d.height,'surface':surface,'occluded_by':[],
                    'graspable':surface['edge_margin_mm']>=self.geometry.cup_radius,
                    'support_frames':self.tracks[key]['count'],'position_map':position})
            # Never select a lower block under a visible higher physical surface.
            for obj in objects:
                p=obj['surface']['arm_xyz_mm']
                obj['occluded_by']=[other['object_id'] for other in objects if other is not obj
                    and other['surface']['arm_xyz_mm'][2]>p[2]+50.
                    and math.dist(other['surface']['arm_xyz_mm'][:2],p[:2])<60.]
                if obj['occluded_by']:obj['graspable']=False
            placement=self.measurement.placement(image,depth,k,active,objects,floor)
            hand={'state':'unknown','evidence':'tool or calibrated view unavailable'}
            if self.measurement.frozen and tool.get('state')=='observed':
                hand=observe_payload_volume(depth,k,self.geometry,tool['reference_arm_mm'],
                    active.get('label',''),active.get('object_id',''))
            self.version+=1
            payload={'scene_id':self.scene_id,'version':self.version,'stamp':color.header.stamp.to_sec(),
                'frame_id':color.header.frame_id,'calibration_id':self.geometry.calibration_id,
                'support_frames':min(3,self.source_frames),'objects':[o for o in objects if o['support_frames']>=3],
                'manipulation':{'tool':tool,'gripper':hand,'calibration':self.measurement.calibration(),
                    'placement':placement,'floor_z_mm':floor}}
            self.pub.publish(String(data=json.dumps(payload)))
            self.status.publish(String(data=json.dumps({'state':'streaming','stamp':payload['stamp'],
                'instances':len(payload['objects']),'tool':tool.get('state'),
                'grasp_xy_source':self.geometry.values.get('grasp_xy_source','dlt'),
                'calibration':self.measurement.calibration()})))
        except Exception as exc:
            self.status.publish(String(data=json.dumps({'state':'observation_unavailable','error':str(exc)})))
            rospy.logwarn_throttle(3.,'grasp observation: %s',exc)
        finally:self.lock.release()


if __name__=='__main__':
    rospy.init_node('suat_mission_scene')
    SceneNode();rospy.spin()
