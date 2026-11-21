#!/usr/bin/env python3
"""Read-only multi-view detection visualization. Never publishes grasp targets."""
import argparse
import json
import os
from pathlib import Path
import sys
import threading
import time

import cv2
import rospy
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
from std_msgs.msg import String

ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'src/suat_mission/src'))
from suat_mission.multiview_preview import MultiViewPreview, FaceStability

def load_detector():
    folder=ROOT/'src/suat_mission/assets/templates'
    templates={name:[cv2.imread(str(folder/(name+'.png')))]
               for name in ('cake','clock','teddybear','banana')}
    templates['cake'].append(cv2.imread(str(folder/'cake_rule_reference.png')))
    return MultiViewPreview(templates)

def annotate(image,detections,age):
    im=image.copy()
    for d in detections:
        x1,y1,x2,y2=[int(round(v)) for v in d['box']]
        if d['partial']:
            color=(255,80,255);tag=' PARTIAL'
        elif d['method']=='rotated_template':
            color=(255,210,0);tag=' ROT'
        else:
            color=(0,220,0);tag=' FEAT'
        cv2.rectangle(im,(x1,y1),(x2,y2),color,2)
        text='%s%s %.2f [%dF]'%(d['label'],tag,d['score'],d.get('support_frames',1))
        cv2.putText(im,text,(max(0,x1),max(39,y1-5)),cv2.FONT_HERSHEY_SIMPLEX,.48,color,2,cv2.LINE_AA)
    cv2.putText(im,'PREVIEW ONLY - NO GRASP  age %.2fs'%age,(7,20),
                cv2.FONT_HERSHEY_SIMPLEX,.48,(0,0,255) if age>2 or age<-.5 else (255,255,0),2,cv2.LINE_AA)
    return im

class LatestFrame:
    def __init__(self):
        self.lock=threading.Lock();self.item=None
    def callback(self,msg):
        with self.lock:self.item=msg
    def take(self):
        with self.lock:
            msg=self.item;self.item=None;return msg

def main(argv=None):
    parser=argparse.ArgumentParser(description='Real-time image-face candidates, never grasp targets')
    parser.add_argument('--no-window',action='store_true',help='publish debug image without opening a window')
    parser.add_argument('--offline',metavar='IMAGE',help='evaluate a saved image; does not start ROS')
    parser.add_argument('--output',metavar='IMAGE',help='optional annotated image output in offline mode')
    args=parser.parse_args(argv)
    cv2.setNumThreads(1)
    detector=load_detector()
    if args.offline:
        im=cv2.imread(args.offline)
        if im is None:print('Cannot read image: '+args.offline,file=sys.stderr);return 2
        started=time.monotonic();found=detector.detect(im)
        print(json.dumps({'elapsed_s':round(time.monotonic()-started,3),'detections':found,
                          'use':'preview_only_not_grasp'},ensure_ascii=False,indent=2))
        if args.output and not cv2.imwrite(args.output,annotate(im,found,0.)):
            print('Cannot save annotated output',file=sys.stderr);return 2
        return 0
    if not args.no_window and not os.environ.get('DISPLAY'):
        print('DISPLAY missing; run from robot desktop or use --no-window',file=sys.stderr);return 2
    rospy.init_node('suat_multiview_preview',anonymous=False)
    bridge=CvBridge();latest=LatestFrame();stability=FaceStability()
    sub=rospy.Subscriber('/camera/color/image_raw',Image,latest.callback,queue_size=1,buff_size=2**24)
    image_pub=rospy.Publisher('/suat/multiview_preview/image',Image,queue_size=1)
    status_pub=rospy.Publisher('/suat/multiview_preview/status',String,queue_size=1)
    print('Preview started: /suat/multiview_preview/image; candidates are NOT grasp targets.',flush=True)
    print('Press Q in the preview window or Ctrl+C in this terminal to stop.',flush=True)
    last_notice=0.
    try:
        while not rospy.is_shutdown():
            msg=latest.take()
            if msg is None:
                time.sleep(.03)
                if not args.no_window and cv2.waitKey(1)&0xff==ord('q'):break
                continue
            try:
                image=bridge.imgmsg_to_cv2(msg,'bgr8')
                started=time.monotonic();found=detector.detect(image)
                found=stability.update(found,time.monotonic())
                duration=time.monotonic()-started
                age=(rospy.Time.now()-msg.header.stamp).to_sec()
                shown=annotate(image,found,age)
                result=bridge.cv2_to_imgmsg(shown,'bgr8');result.header=msg.header;image_pub.publish(result)
                payload={'frame_seq':msg.header.seq,'stamp':msg.header.stamp.to_sec(),
                         'age_s':round(age,3),'elapsed_s':round(duration,3),
                         'detections':found,'use':'preview_only_not_grasp'}
                status_pub.publish(String(data=json.dumps(payload,ensure_ascii=False)))
                if not args.no_window:
                    cv2.imshow('SUAT multi-view preview - no grasp',shown)
                    if cv2.waitKey(1)&0xff==ord('q'):break
                now=time.monotonic()
                if now-last_notice>5:
                    print('age=%.2fs candidates='%age+
                          ', '.join('%s:%s'%(d['label'],d['method']) for d in found),flush=True)
                    last_notice=now
            except (CvBridgeError,ValueError,cv2.error) as exc:
                rospy.logwarn_throttle(5,'preview error: %s',exc)
    except KeyboardInterrupt:
        pass
    finally:
        sub.unregister()
        if not args.no_window:cv2.destroyAllWindows()
    return 0

if __name__=='__main__':raise SystemExit(main())
