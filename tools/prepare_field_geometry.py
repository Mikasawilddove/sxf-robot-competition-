#!/usr/bin/env python3
"""Import the robot's actual baseline; never executes arm_pick_place.py."""
import argparse
import ast
import json
from pathlib import Path


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--cup-diameter-mm',type=float,required=True,help='actual contact diameter, measured on this cup')
    parser.add_argument('--output',type=Path)
    parser.add_argument('--source',type=Path,help='read a captured copy of the actual robot source, without importing it')
    args=parser.parse_args()
    if not 0<args.cup_diameter_mm<100:parser.error('provide the measured suction-cup contact diameter in mm')
    root=Path(__file__).resolve().parents[1]
    source=args.source or root/'src/suat_mission/src/suat_mission/manipulation/arm_pick_place.py'
    tree=ast.parse(source.read_text(encoding='utf-8'))
    constants={};defaults={}
    for node in tree.body:
        if not isinstance(node,ast.Assign):continue
        for target in node.targets:
            if not isinstance(target,ast.Name) or target.id not in ('A','B','FX','FY','CX','CY'):continue
            value=node.value.args[0] if isinstance(node.value,ast.Call) else node.value
            constants[target.id]=ast.literal_eval(value)
    for node in ast.walk(tree):
        if isinstance(node,ast.Call) and isinstance(node.func,ast.Attribute) and node.func.attr=='add_argument':
            try:name=ast.literal_eval(node.args[0])
            except (ValueError,TypeError,IndexError):continue
            for key in node.keywords:
                if key.arg=='default':
                    try:defaults[name]=ast.literal_eval(key.value)
                    except (ValueError,TypeError):pass
    import numpy as np
    a=np.asarray(constants['A'],float);b=np.asarray(constants['B'],float);inverse=np.linalg.inv(a)
    ordinary=float(defaults['--block-z']);offset=float(defaults['--grasp-offset'])
    def xyz(value):return [float(v) for v in value.split(',')]
    result={'calibration_id':'legacy_dlt_online_translation_20261117','source_file':str(source),
        'source_status':'imported_baseline_not_new_physical_validation',
        'workspace_source':'controller_ik','arm_bounds_mm':None,'reliable_work_area_mm':None,
        'support_z_mm':ordinary-100.,'cup_offset_mm':offset,'cup_radius_mm':args.cup_diameter_mm/2.,
        'cup_diameter_source':'operator_provided_contact_diameter','arm_yaw_in_base_rad':0.,
        'arm_mount_source':'spark_pro_model.xacro uarm_base_link_joint rpy 0 0 0; physical application still field-validated',
        'legacy_camera_A':a.tolist(),'legacy_camera_B_mm':b.tolist(),
        'camera_to_arm_mm':np.column_stack((inverse,-inverse@b)).tolist(),
        'geometry_method':'affine_depth_baseline','observation_pose_mm':xyz(defaults['--observe']),
        'board_observation_pose_mm':xyz(defaults['--observe']),
        'empty_travel_pose_mm':xyz(defaults['--home']),
        'transport_pose_mm':[202.75,-33.82,100.66],
        'transport_pose_source':'previous manually accepted cake transport; current executor needs field acceptance',
        'calibration_view_poses_mm':[[200.,0.,150.],[230.,0.,150.]],
        'refine_translation_at_start':True,'require_visual_alignment':True,
        'approach_offset_mm':40.,'lift_offset_mm':50.,'release_bottom_clearance_mm':100.,
        'object_height_mm':{label:100. for label in ('cake','clock','teddybear','banana')},
        'object_width_mm':{label:100. for label in ('cake','clock','teddybear','banana')},
        'placement_mode':'visual_red_zone','plane_residual_mm':5.,'height_tolerance_mm':12.,
        'alignment_tolerance_mm':5.,'max_visual_correction_mm':30.,'max_top_tilt_deg':15.,
        'calibration_pair_residual_mm':10.,'holding_position_tolerance_mm':25.}
    filename=args.output or root/'data/calibration/field_20261117/pickup_geometry_active.yaml'
    if filename.exists():
        backup=filename.with_name(filename.stem+'_before_import_'+str(__import__('time').time_ns())+filename.suffix)
        backup.write_bytes(filename.read_bytes())
    filename.parent.mkdir(parents=True,exist_ok=True)
    # JSON is accepted by the existing YAML reader and needs no extra writer dependency.
    filename.write_text(json.dumps(result,ensure_ascii=False,indent=2)+'\n',encoding='utf-8')
    print(json.dumps({'written':str(filename),'source':str(source),'status':result['source_status'],
                      'motion_commands_sent':False},ensure_ascii=False))


if __name__=='__main__':main()
