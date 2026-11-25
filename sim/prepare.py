#!/usr/bin/env python3
"""Generate task-owned assets from the installed Humble UR7e description."""
import copy, json, shutil, subprocess, xml.etree.ElementTree as ET
from pathlib import Path
import cv2
import numpy as np
import yaml
from scene_geometry import TABLE_BOX, TABLE_LEGS, BRACKET_BOXES, GROUND_BOX
from ament_index_python.packages import get_package_share_directory
ROOT=Path(__file__).resolve().parents[1]
OUT=ROOT/'runs/isaac/assets'; OUT.mkdir(parents=True,exist_ok=True)
c=yaml.safe_load((ROOT/'config/lab.yaml').read_text())
c.update(execution_environment='isaac',use_sim_time=True)
c['verified']={k:True for k in c['verified']}
c['motion']['max_approach']=.25
c['world_boxes']=[GROUND_BOX, TABLE_BOX, *TABLE_LEGS]
c['attached_boxes'].extend(BRACKET_BOXES)
c['camera']['optical_frame']='camera_color_optical_frame'
# The simulation camera housing is a simplified box behind the optical origin.
for box in c['attached_boxes']:
    if box['id']=='doodle_camera':
        box.update(center=[-.025,.13,-.025], size=[.10,.08,.06])
    elif box['id']=='doodle_gripper':
        box.update(size=[.08,.065,.17])
(ROOT/'config/isaac.yaml').write_text(yaml.safe_dump(c,sort_keys=False))
share=Path(get_package_share_directory('ur_description'))
xml=subprocess.check_output(['xacro',str(share/'urdf/ur.urdf.xacro'),'ur_type:=ur7e','name:=ur','use_fake_hardware:=true'],text=True)
robot=ET.fromstring(xml)
for e in list(robot.findall('ros2_control')): robot.remove(e)
# Same generated UR model in MoveIt, controller_manager, TF, and Isaac importer.
control=ET.SubElement(robot,'ros2_control',name='IsaacUR7e',type='system')
hw=ET.SubElement(control,'hardware');ET.SubElement(hw,'plugin').text='doodle_isaac_hardware/System'
for name in c['joint_names']:
    j=ET.SubElement(control,'joint',name=name)
    ET.SubElement(j,'command_interface',name='position')
    for kind in ['position','velocity']: ET.SubElement(j,'state_interface',name=kind)
sensor=ET.SubElement(control,'sensor',name='speed_scaling');ET.SubElement(sensor,'state_interface',name='speed_scaling_factor')
# The imported fixed mount uses Lab5's approximate wrist->optical transform.
link=ET.SubElement(robot,'link',name='camera_color_optical_frame')
j=ET.SubElement(robot,'joint',name='doodle_camera_mount',type='fixed')
ET.SubElement(j,'parent',link='wrist_3_link');ET.SubElement(j,'child',link='camera_color_optical_frame')
ET.SubElement(j,'origin',xyz=' '.join(map(str,c['camera']['mount_position'])),rpy='0 0 0')
(OUT/'robot.urdf').write_text(ET.tostring(robot,encoding='unicode'))
physical=copy.deepcopy(robot); physical.remove(physical.find('ros2_control'))
# Copy meshes into task assets, not the host ROS installation.
for mesh in physical.iter('mesh'):
    uri=mesh.get('filename')
    if uri.startswith('package://ur_description/'):
        rel=uri.removeprefix('package://ur_description/')
        dst=OUT/'ur_description'/rel;dst.parent.mkdir(parents=True,exist_ok=True);shutil.copy2(share/rel,dst)
        mesh.set('filename',str(dst))
(OUT/'robot.import.urdf').write_text(ET.tostring(physical,encoding='unicode'))
controllers={'controller_manager':{'ros__parameters':{'update_rate':100,'use_sim_time':True,
 'joint_state_broadcaster':{'type':'joint_state_broadcaster/JointStateBroadcaster'},
 'scaled_joint_trajectory_controller':{'type':'ur_controllers/ScaledJointTrajectoryController'}}},
 'scaled_joint_trajectory_controller':{'ros__parameters':{'joints':c['joint_names'],'command_interfaces':['position'],
 'state_interfaces':['position','velocity'],'speed_scaling_interface_name':'speed_scaling/speed_scaling_factor',
 'state_publish_rate':50.,'action_monitor_rate':20.,'update_rate':100,
 'constraints':{'stopped_velocity_tolerance':.01,'goal_time':5.,**{j:{'trajectory':.05,'goal':.005} for j in c['joint_names']}}}}}
(OUT/'controllers.yaml').write_text(yaml.safe_dump(controllers))
(OUT/'robot_parameters.yaml').write_text(yaml.safe_dump({'/**':{'ros__parameters':{'robot_description':ET.tostring(robot,encoding='unicode'),'use_sim_time':True}}}))
# Board raster is rendered as actual scene geometry; export reference for QA.
canvas=np.full((800,800,3),255,np.uint8)
dictionary=cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
markers={}
for i,(x,y) in enumerate([(100,700),(700,700),(700,100),(100,100)]):
    marker=cv2.aruco.drawMarker(dictionary,i,100)
    canvas[y-50:y+50,x-50:x+50]=marker[:,:,None]
    markers[str(i)]=cv2.aruco.drawMarker(dictionary,i,6).tolist()
cv2.imwrite(str(OUT/'board.png'),canvas)
(OUT/'markers.json').write_text(json.dumps(markers))
# Deterministic smiley input goes through the existing image contour pipeline.
smile=np.full((400,400,3),255,np.uint8)
cv2.circle(smile,(200,200),150,(0,0,0),3)
for x in [145,255]: cv2.circle(smile,(x,155),13,(0,0,0),3)
cv2.ellipse(smile,(200,210),(85,75),0,15,165,(0,0,0),3)
cv2.imwrite(str(OUT/'smiley.png'),smile)
print(OUT)
mshare=Path(get_package_share_directory('ur_moveit_config'))
semantic=subprocess.check_output(['xacro',str(mshare/'srdf/ur.srdf.xacro'),'name:=ur','prefix:='],text=True)
kin=yaml.safe_load((mshare/'config/kinematics.yaml').read_text())
moveit={'robot_description':ET.tostring(robot,encoding='unicode'),'robot_description_semantic':semantic,
 'use_sim_time':True,'qos_overrides':{'/clock':{'subscription':{'durability':'volatile','reliability':'best_effort','history':'keep_last','depth':1}}},'publish_robot_description':True,'publish_robot_description_semantic':True,
 'robot_description_planning':yaml.safe_load((mshare/'config/joint_limits.yaml').read_text()),
 'planning_plugin':'ompl_interface/OMPLPlanner','request_adapters':'','start_state_max_bounds_error':.1,
 'moveit_controller_manager':'moveit_simple_controller_manager/MoveItSimpleControllerManager',
 'moveit_simple_controller_manager':yaml.safe_load((mshare/'config/controllers.yaml').read_text()),
 'trajectory_execution.execution_duration_monitoring':False,'moveit_manage_controllers':False,
 'publish_planning_scene':True,'publish_geometry_updates':True,'publish_state_updates':True,'publish_transforms_updates':True}
moveit.update(kin['/**']['ros__parameters'])
(OUT/'moveit_parameters.yaml').write_text(yaml.safe_dump({'move_group':{'ros__parameters':moveit}}))
