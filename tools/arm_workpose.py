#!/usr/bin/env python3
"""Empty-arm measured pose recovery. Describe by default; no pump or grasp commands."""
import argparse
import json
import math


def arm_log(step,target=None,result=None):
    print('[机械臂] 当前步骤：%s' % step,flush=True)
    if target is not None:
        print('[机械臂] 目标位置：(%.1f, %.1f, %.1f) mm' % tuple(target),flush=True)
    print('[机械臂] 目标姿态：当前接口仅控制 XYZ；参考坐标系：arm_base；坐标单位：毫米',flush=True)
    if result is not None:
        print('[机械臂] 执行结果：%s' % result,flush=True)


parser=argparse.ArgumentParser(description=__doc__)
parser.add_argument('--target',nargs=3,type=float,default=[200.,0.,150.],metavar=('X_MM','Y_MM','Z_MM'))
parser.add_argument('--feed',type=float,default=1000.,help='G0 feed; teammate fast candidate 80000')
parser.add_argument('--execute',action='store_true')
parser.add_argument('--site-ready',action='store_true')
args=parser.parse_args()
if not all(math.isfinite(v) for v in args.target):parser.error('target must contain finite numbers')
if not math.isfinite(args.feed) or not 0 < args.feed <= 80000:parser.error('feed must be within (0, 80000]')
if not args.execute:
 arm_log('仅检查参数，不发送运动',args.target,'未执行（需要 --execute --site-ready）')
 print(json.dumps({'mode':'DESCRIBE_ONLY','target_xyz':args.target,'motion_commands_written':False,
                   'required_flags':['--execute','--site-ready'],'pump_commands_sent':False}));raise SystemExit(0)
if not args.site_ready:parser.error('--execute requires --site-ready')
import base64
from dataclasses import asdict
import json
import os
from pathlib import Path
import re
import shutil
import subprocess
import sys
import time
import xmlrpc.client
import fcntl
import select
import struct
import termios
import rospy
from nav_msgs.msg import Odometry

class PosixSerial:
    """Bounded standard-library transport; no dependency installation."""
    def __init__(self,port):
        self.fd=os.open(port,os.O_RDWR|os.O_NOCTTY|os.O_NONBLOCK)
        try:
            fcntl.flock(self.fd,fcntl.LOCK_EX|fcntl.LOCK_NB)
            fcntl.ioctl(self.fd,termios.TIOCEXCL)
            self.original=termios.tcgetattr(self.fd)
            attrs=termios.tcgetattr(self.fd)
            attrs[0]=0;attrs[1]=0;attrs[2]=termios.CS8|termios.CLOCAL|termios.CREAD;attrs[3]=0
            attrs[4]=termios.B115200;attrs[5]=termios.B115200
            attrs[6][termios.VMIN]=0;attrs[6][termios.VTIME]=0
            termios.tcsetattr(self.fd,termios.TCSANOW,attrs)
        except BaseException:
            os.close(self.fd);raise
    @property
    def in_waiting(self):
        return struct.unpack('I',fcntl.ioctl(self.fd,termios.FIONREAD,struct.pack('I',0)))[0]
    def read(self,count):
        if not select.select([self.fd],[],[],.05)[0]:return b''
        try:return os.read(self.fd,count)
        except BlockingIOError:return b''
    def write(self,data):
        if not select.select([],[self.fd],[],.3)[1]:raise TimeoutError('serial write not ready')
        return os.write(self.fd,data)
    def close(self):
        try:termios.tcsetattr(self.fd,termios.TCSANOW,self.original)
        finally:os.close(self.fd)

root=Path(__file__).resolve().parents[1]
arm_log('空载工作位恢复',args.target,'准备检查控制权与底盘静止状态')
from datetime import datetime
from uuid import uuid4
run=root/'reports/runs'/('arm_workpose_'+datetime.now().strftime('%Y%m%d_%H%M%S')+'_'+uuid4().hex[:6])
run.mkdir(parents=True,exist_ok=False)
sys.path.insert(0,str(root/'src/suat_mission/src'))
from suat_mission.arm_feedback import SwiftProParser
from suat_mission.arm_recovery import recover_pose,matched_reply_or_fault
from suat_mission.contracts import ActionContext


source=Path('/home/spark/spark_noetic/src/spark_driver/arm/uArm/swiftpro/src/swiftpro_write_node.cpp')
ports=re.findall(r'_serial\.setPort\("([^"\n]+)"\)',source.read_text())
if len(ports)!=1:raise RuntimeError('cannot establish actual factory serial port')
port=ports[0]
if not shutil.which('fuser'):raise RuntimeError('fuser missing; do not replace owner')
rospy.init_node('suat_arm_workpose',anonymous=True)
odom=rospy.wait_for_message('/odom',Odometry,timeout=3)
if abs(odom.twist.twist.linear.x)>.015 or abs(odom.twist.twist.angular.z)>.03:
    raise RuntimeError('base is not stopped')
master=xmlrpc.client.ServerProxy('http://localhost:11311')
state=master.getSystemState(rospy.get_name())[2]
pubs=dict(state[0]);inputs=('position_write_topic','swiftpro_status_topic','angle4th_topic','gripper_topic','pump_topic')
active={topic:pubs.get('/'+topic,[]) for topic in inputs if pubs.get('/'+topic)}
if active:raise RuntimeError('arm command publishers still present: '+repr(active))
code,_,uri=master.lookupNode(rospy.get_name(),'/swiftpro_write_node')
if code!=1:raise RuntimeError('expected original arm owner missing')
owner=xmlrpc.client.ServerProxy(uri);pid=owner.getPid(rospy.get_name())[2]
report={'queries':[],'port':port,'original_pid':pid,'chunks':[],'parsed':[],
        'responses':[],'motion_commands_written':False,'pump_commands_written':False,
        'arrival':'NOT_TESTED','holding':'NOT_TESTED','physical_measurement_accuracy':'NOT_TESTED'}
handle=None;paused=False;motion_sent=False;parser=None;sequence=0
try:
    owner.shutdown(rospy.get_name(),'controlled empty-arm startup recovery with measured feedback')
    paused=True;until=time.monotonic()+4
    while Path('/proc/%d'%pid).exists() and time.monotonic()<until:time.sleep(.05)
    if Path('/proc/%d'%pid).exists():raise RuntimeError('old owner has not exited')
    held=subprocess.run(['fuser',port],stdout=subprocess.PIPE,stderr=subprocess.PIPE,universal_newlines=True)
    if held.returncode!=1 or held.stdout.strip():raise RuntimeError('serial device still owned')
    handle=PosixSerial(port);parser=SwiftProParser()
    def receive():
        data=handle.read(max(1,min(handle.in_waiting,8192)))
        if not data:return []
        at=time.monotonic();report['chunks'].append({'received_monotonic':at,'base64':base64.b64encode(data).decode()})
        frames=parser.feed(data,at);report['parsed'].extend(asdict(f) for f in frames);return frames
    until=time.monotonic()+4
    while time.monotonic()<until:receive()
    sequence=int(time.monotonic()*1000)%100000000+1000
    def ask(command,timeout=1.5):
        global sequence,motion_sent
        sequence+=1;wire=('#%d %s\r\n'%(sequence,command)).encode('ascii')
        sent=time.monotonic()
        active=getattr(globals().get('backend',None),'active_deadline',None)
        if active is not None:
            timeout=min(timeout,max(.05,active-sent))
            if sent>=active and command!='M2000':raise TimeoutError('request after action deadline')
        row={'command':command,'sequence':sequence,'sent_monotonic':sent,'wire':wire.decode()}
        report['responses'].append(row)
        if command.startswith('G0 '):motion_sent=True;report['motion_commands_written']=True
        if handle.write(wire)!=len(wire):raise RuntimeError('partial write')
        while time.monotonic()<sent+timeout:
            frames=receive()
            frame=matched_reply_or_fault(frames,sequence,sent)
            if frame is not None:
                row['frame']=asdict(frame)
                if frame.kind!='reply':raise RuntimeError('device rejected '+command+': '+frame.raw)
                return frame
        row['timeout']=True;raise TimeoutError('no matched reply: '+command)
    def values(frame,names):
        if len(frame.payload)!=len(names) or [p[0] for p in frame.payload]!=list(names):raise RuntimeError('wrong numeric fields')
        result=[float(p[1:]) for p in frame.payload]
        if not all(math.isfinite(v) for v in result):raise RuntimeError('non-finite reply')
        return result
    if values(ask('P2234'),'V')!=[1.]:raise RuntimeError('external power not present')
    if values(ask('P2231'),'V')!=[0.]:raise RuntimeError('pump not off; empty recovery only')
    if values(ask('M2200'),'V')!=[0.]:raise RuntimeError('arm not idle')
    class Backend:
        active_deadline=None
        def now(self):return time.monotonic()
        def validate_target(self,target,deadline):
            self.active_deadline=deadline
            valid=values(ask('M2222 X%.4f Y%.4f Z%.4f'%tuple(target)),'V')==[1.]
            row={'command_xyz':list(target),'controller_valid':valid}
            report.setdefault('target_validations',[]).append(row)
            if not valid:return False
            row['command_joint_deg']=values(ask('M2220 X%.4f Y%.4f Z%.4f'%tuple(target)),'BLR')
            return True
        def read_pose(self,deadline):
            self.active_deadline=deadline
            joint=ask('P2200');angles=values(joint,'BLR')
            fk=ask('M2221 B%.4f L%.4f R%.4f'%tuple(angles))
            return {'xyz':values(fk,'XYZ'),'source':'P2200+M2221','joints_deg':angles,
                    'joint_sequence':joint.sequence,'fk_sequence':fk.sequence,
                    'joint_received_at':joint.received_at,'received_at':fk.received_at}
        def move(self,target,deadline):
            self.active_deadline=deadline
            if values(ask('M2200'),'V')!=[0.]:raise RuntimeError('moving before correction')
            ask('M2000')  # synchronise native planner to measured joints before each correction
            ask('G0 X%.4f Y%.4f Z%.4f F%.0f'%(*target,args.feed),timeout=12)
            idle_count=0;trace=[]
            while self.now()<deadline:
                pose=self.read_pose(deadline);moving=values(ask('M2200'),'V')[0]
                trace.append(dict(pose,controller_moving=moving))
                idle_count=idle_count+1 if moving==0 else 0
                if idle_count>=3:
                    report.setdefault('correction_motion_traces',[]).append(trace);return
                time.sleep(.1)
            raise TimeoutError('correction exceeded work deadline')
        def stop(self,deadline):
            self.active_deadline=deadline
            frame=ask('M2000');idle=values(ask('M2200'),'V')==[0.]
            return {'acknowledged':frame.kind=='reply','controller_idle':idle,'stop_sequence':frame.sequence}
    import math
    backend=Backend()
    context=ActionContext(run.name,run.name+':empty_recovery','empty_recovery','',0,time.monotonic()+60.)
    report['recovery']=recover_pose(backend,args.target,context,tolerance_mm=5.,max_moves=3,stop_budget_s=2.,max_correction_step_mm=10.,max_command_offset_mm=25.)
    report['operational_policy_note']='5 mm device-model convergence for this empty recovery experiment; absolute tip accuracy not independently validated'
    report['outcome']=report['recovery']['outcome']
    final_pose=report['recovery'].get('final_measurement',{}).get('xyz')
    if final_pose:
        print('[机械臂] 当前实测位置：(%.1f, %.1f, %.1f) mm' % tuple(final_pose),flush=True)
    arm_log('空载工作位恢复',args.target,
            '成功（设备模型闭环到位）' if report['outcome']=='measured_target_reached' else report['outcome'])
except (Exception,KeyboardInterrupt) as exc:
    report['error']=str(exc) or type(exc).__name__
    arm_log('空载工作位恢复',args.target,'失败：'+report['error'])
    if motion_sent and handle is not None:
        try:report['fault_stop_reply']=asdict(ask('M2000'))
        except Exception as stop_exc:report['fault_stop_error']=str(stop_exc)
finally:
    if handle is not None:
        try:handle.close()
        except (Exception,KeyboardInterrupt) as exc:report['serial_close_error']=str(exc) or type(exc).__name__
    if paused:
        try:
            holders=subprocess.run(['fuser',port],stdout=subprocess.PIPE,stderr=subprocess.PIPE,universal_newlines=True,timeout=3)
            if holders.returncode!=1 or holders.stdout.strip():raise RuntimeError('serial ownership not released; do not open a second owner')
            with (run/'factory_owner_restore.log').open('wb') as log:
                restored=subprocess.Popen(['rosrun','swiftpro','swiftpro_write_node'],stdout=log,stderr=subprocess.STDOUT,stdin=subprocess.DEVNULL,start_new_session=True)
            time.sleep(4)
            report['factory_restore']={'pid':restored.pid,'process_running':restored.poll() is None,'original_startup_sends_M17':True,'lookup_code':master.lookupNode(rospy.get_name(),'/swiftpro_write_node')[0]}
        except (Exception,KeyboardInterrupt) as exc:report['factory_restore_error']=str(exc) or type(exc).__name__
    report['pump_commands_written']=False;report['hardware_ready']=False
    try:(run/'result.json').write_text(json.dumps(report,indent=2),encoding='utf-8')
    except (Exception,KeyboardInterrupt) as exc:report['report_write_error']=str(exc) or type(exc).__name__
    print(json.dumps(report))

restored=report.get('factory_restore',{})
ok=(report.get('recovery',{}).get('outcome')=='measured_target_reached'
    and restored.get('process_running') is True and restored.get('lookup_code')==1
    and not any(key in report for key in ('serial_close_error','factory_restore_error','report_write_error')))
raise SystemExit(0 if ok else 2)
