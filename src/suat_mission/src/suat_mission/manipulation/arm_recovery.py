"""Bounded empty-arm recovery from measured joints, with injected physical I/O.

This reports device-model convergence. It does not certify absolute cup accuracy,
collision clearance, holding, or readiness of the competition mission.
"""
import math
from suat_mission.contracts import ActionContext


def matched_reply_or_fault(frames,sequence,sent_at):
    """A normal reply must never hide a fresh controller/parse fault."""
    for frame in frames:
        if frame.received_at>=sent_at and frame.kind in ('unknown','invalid','unnumbered_error','error'):
            raise ValueError('arm protocol fault: '+frame.raw)
    reply_kind = 'unnumbered_reply' if sequence is None else 'reply'
    matches=[frame for frame in frames if frame.sequence==sequence
             and frame.first_byte_at>=sent_at and frame.kind==reply_kind]
    if len(matches)>1:raise ValueError('multiple replies for one arm request')
    return matches[0] if matches else None


def recover_pose(io,target,context,*,tolerance_mm,max_moves,stop_budget_s,
                 max_correction_step_mm=None,max_command_offset_mm=None, previous_move_at=None,
                 stop_after_success=True,target_prevalidated=False,move_with_feedback=None):
    if (not isinstance(context,ActionContext) or len(target)!=3
            or not all(type(v) in (int,float) and math.isfinite(v) for v in target)
            or not math.isfinite(tolerance_mm) or tolerance_mm<=0
            or type(max_moves) is not int or max_moves<1
            or not math.isfinite(stop_budget_s) or stop_budget_s<=0
            or not math.isfinite(context.deadline)):
        raise ValueError('recovery needs a finite target, explicit policy and action deadline')
    compensate=max_correction_step_mm is not None or max_command_offset_mm is not None
    if compensate and (max_correction_step_mm is None or max_command_offset_mm is None
            or not all(type(v) in (int,float) and math.isfinite(v) and v>0
                       for v in (max_correction_step_mm,max_command_offset_mm))):
        raise ValueError('feedback compensation needs explicit positive step and total command limits')
    report={'outcome':'failed','target_xyz':list(target),'tolerance_mm':tolerance_mm,
            'max_moves':max_moves,'measurements':[],'moves':[],'motion_commands_sent':previous_move_at is not None,
            'hardware_ready':False,'absolute_tip_accuracy':'NOT_TESTED'}
    report['command_compensation']={'enabled':compensate,'max_step_mm':max_correction_step_mm,
                                    'max_offset_mm':max_command_offset_mm}
    work_deadline=context.deadline-stop_budget_s
    latest_joint_sequence=0;last_motion_at=previous_move_at
    def check(deadline=work_deadline):
        if io.now()>=deadline:raise TimeoutError('recovery work deadline reached')
    def measure(deadline=work_deadline, *, feedback=None):
        nonlocal latest_joint_sequence
        check(deadline)
        read_started_at=io.now() if feedback is None else last_motion_at
        pose=io.read_pose(deadline) if feedback is None else feedback
        check(deadline)
        xyz=pose.get('xyz',[])
        if (pose.get('source')!='P2200+M2221' or len(xyz)!=3
                or not all(type(v) in (int,float) and math.isfinite(v) for v in xyz)
                or type(pose.get('joint_sequence')) is not int
                or type(pose.get('fk_sequence')) is not int
                or not latest_joint_sequence<pose['joint_sequence']<pose['fk_sequence']
                or not all(type(pose.get(k)) in (int,float) and math.isfinite(pose[k])
                           for k in ('joint_received_at','received_at'))
                or not (read_started_at<=pose['joint_received_at']<=pose['received_at']<=io.now())
                or (last_motion_at is not None and pose['joint_received_at']<=last_motion_at)):
            raise ValueError('fresh, ordered joint-derived physical pose required')
        latest_joint_sequence=pose['joint_sequence']
        report['measurements'].append(pose)
        report['final_error_mm']=math.dist(xyz,target)
        return pose
    try:
        check()
        if not target_prevalidated and io.validate_target(target,work_deadline) is not True:
            raise ValueError('controller rejected recovery target')
        pose=measure()
        command=list(target)
        for index in range(max_moves):
            if report['final_error_mm']<=tolerance_mm:break
            corrected=compensate and (index>0 or previous_move_at is not None)
            if corrected:
                error=[goal-measured for goal,measured in zip(target,pose['xyz'])]
                scale=min(1.,max_correction_step_mm/math.dist(pose['xyz'],target))
                candidate=[value+scale*delta for value,delta in zip(command,error)]
                if math.dist(candidate,target)>max_command_offset_mm:
                    raise ValueError('feedback correction exceeds command envelope')
                check()
                if io.validate_target(candidate,work_deadline) is not True:
                    raise ValueError('controller rejected corrected recovery command')
                command=candidate
            check();last_motion_at=io.now()
            report['motion_commands_sent']=True
            report['moves'].append({'index':index+1,'sent_at':last_motion_at,'from_pose':pose,
                                    'target_xyz':list(target),'command_xyz':list(command)})
            previous_error=report['final_error_mm']
            if move_with_feedback is None:
                io.move(command,work_deadline)
                pose=measure()
            else:
                feedback=move_with_feedback(command,work_deadline,start_pose=pose)
                pose=measure(feedback=feedback)
            if corrected and report['final_error_mm']>=previous_error:
                raise ValueError('measured feedback correction made no progress')
        if report['final_error_mm']<=tolerance_mm:
            report['outcome']='measured_target_reached'
        else:report['error']='bounded recovery exhausted before measured target convergence'
    except (Exception,KeyboardInterrupt) as exc:
        report['error']=str(exc) or type(exc).__name__
        report['outcome']='timeout' if isinstance(exc,TimeoutError) else 'failed'
    finally:
        if report['motion_commands_sent'] and (stop_after_success or report['outcome']!='measured_target_reached'):
            try:
                report['stop']=io.stop(context.deadline)
                if (report['stop'].get('acknowledged') is not True
                        or report['stop'].get('controller_idle') is not True):
                    report['outcome']='failed';report['error']='recovery stop/idle unconfirmed'
                elif report['outcome']=='measured_target_reached':
                    last_motion_at=io.now()
                    measure(context.deadline)
                    if report['final_error_mm']>tolerance_mm:
                        report['outcome']='failed';report['error']='stopped measured pose left target tolerance'
            except (Exception,KeyboardInterrupt) as exc:
                report['outcome']='failed';report['stop_error']=str(exc) or type(exc).__name__
        if io.now()>=context.deadline:
            report['outcome']='timeout';report['error']='parent recovery deadline reached'
    return report
