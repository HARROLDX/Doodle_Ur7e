#!/usr/bin/env python3
"""Isaac Sim 5.1 physics, rendered wrist RGB-D and actual-tip ink recorder."""
import argparse, json, os, time
from pathlib import Path
from array import array
from scene_geometry import TABLE_TOP, PAPER_SURFACE, MARKER_SURFACE, TABLE_BOX, TABLE_LEGS, BRACKET_BOXES
p=argparse.ArgumentParser();p.add_argument('--steps',type=int,default=2000000);args=p.parse_args()
# Persist actual measurements even if the controller process exits unexpectedly.
if os.environ.get('ROS_DOMAIN_ID')!='231': raise RuntimeError('simulation domain must be 231')
ROOT=Path(__file__).resolve().parents[1]; OUT=ROOT/'runs/isaac'; ASSETS=OUT/'assets'
os.chdir(ASSETS)
from isaacsim import SimulationApp
app=SimulationApp({'headless':True,'width':960,'height':720,'renderer':'RaytracedLighting', 'active_gpu':0,'multi_gpu':False,'anti_aliasing':0,'disable_viewport_updates':True})
import numpy as np
from scipy.spatial.transform import Rotation
import omni.kit.commands, omni.usd
from pxr import Usd, UsdGeom, UsdPhysics, UsdLux, Gf, PhysxSchema
from isaacsim.core.api import World
from isaacsim.core.prims import SingleArticulation, XFormPrim
from isaacsim.core.api.objects import VisualCuboid, FixedCuboid
from isaacsim.core.utils.types import ArticulationAction
from isaacsim.core.utils.extensions import enable_extension
from isaacsim.sensors.camera import Camera
from PIL import Image as PILImage

enable_extension('isaacsim.ros2.bridge');enable_extension('isaacsim.asset.importer.urdf');app.update()
import rclpy
from sensor_msgs.msg import JointState, Image, CameraInfo
from rclpy.qos import qos_profile_sensor_data
from rosgraph_msgs.msg import Clock
from builtin_interfaces.msg import Time
from std_msgs.msg import Bool
rclpy.init();node=rclpy.create_node('doodle_isaac_world')
feedback=node.create_publisher(JointState,'/doodle_sim/joint_feedback',qos_profile_sensor_data)
clock=node.create_publisher(Clock,'/clock',10)
rgb=node.create_publisher(Image,'/camera/camera/color/image_raw',qos_profile_sensor_data)
depth=node.create_publisher(Image,'/camera/camera/depth/image_rect_raw',qos_profile_sensor_data)
info=node.create_publisher(CameraInfo,'/camera/camera/color/camera_info',qos_profile_sensor_data)
commands={};recording=False;done=False

def command(msg):
    global commands
    if len(msg.name)==len(msg.position) and np.isfinite(msg.position).all(): commands=dict(zip(msg.name,msg.position))
def record(msg):
    global recording
    recording=msg.data
def finish(msg):
    global done
    done=msg.data
node.create_subscription(JointState,'/doodle_sim/joint_commands',command,1)
node.create_subscription(Bool,'/doodle_sim/record',record,10)
node.create_subscription(Bool,'/doodle_sim/finish',finish,10)
world=World(stage_units_in_meters=1.,physics_dt=.02,rendering_dt=.5)
stage=omni.usd.get_context().get_stage()
_,ic=omni.kit.commands.execute('URDFCreateImportConfig')
ic.merge_fixed_joints=False;ic.fix_base=True;ic.import_inertia_tensor=True;ic.self_collision=False
ic.default_drive_strength=10000.;ic.default_position_drive_damping=1000.
ok,root=omni.kit.commands.execute('URDFParseAndImportFile',urdf_path=str(ASSETS/'robot.import.urdf'),import_config=ic)
if not ok: raise RuntimeError('URDF import failed')
print('IMPORTED',root,flush=True)
roots=[str(pr.GetPath()) for pr in stage.Traverse() if pr.HasAPI(UsdPhysics.ArticulationRootAPI)]
print('ARTICULATIONS',roots,flush=True)
robot=world.scene.add(SingleArticulation(prim_path=roots[0],name='ur7e'))
wrist_path=next(str(pr.GetPath()) for pr in stage.Traverse() if pr.GetName()=='wrist_3_link')
wrist=XFormPrim(prim_paths_expr=wrist_path,name='wrist',reset_xform_properties=False)
# Robot base at z=0 on the ground; independent low drawing table has four legs.
world.scene.add(FixedCuboid('/World/Ground',name='ground',position=np.array([0.,0.,-.025]),scale=np.array([3.,3.,.05]),color=np.array([.35,.38,.40])))
for box in [TABLE_BOX,*TABLE_LEGS]:
    world.scene.add(FixedCuboid('/World/'+box['id'],name=box['id'],position=np.array(box['center']),scale=np.array(box['size']),color=np.array([.40,.29,.17]) if box is TABLE_BOX else np.array([.15,.17,.19])))
world.scene.add(VisualCuboid('/World/Paper',name='paper',position=np.array([0.,.5,TABLE_TOP+.0001]),scale=np.array([.205,.205,.0002]),color=np.ones(3)))
# Render real black marker cells on white paper, IDs and orientation match vision.py.
markers=json.loads((ASSETS/'markers.json').read_text())
for i,(cx,cy) in enumerate([(-.075,.425),(.075,.425),(.075,.575),(-.075,.575)]):
    for row,col in np.argwhere(np.array(markers[str(i)])==0):
        world.scene.add(VisualCuboid(f'/World/Markers/m{i}_{row}_{col}',name=f'm{i}_{row}_{col}',position=np.array([cx+(col-2.5)*.025/6,cy+(2.5-row)*.025/6,MARKER_SURFACE-.00005]),scale=np.array([.025/6,.025/6,.0001]),color=np.zeros(3)))
def box(name,center,size,color):
    return world.scene.add(VisualCuboid(wrist_path+'/'+name,name=name,translation=np.array(center),scale=np.array(size),color=np.array(color)))

def cylinder(name,center,radius,height,color):
    shape=UsdGeom.Cylinder.Define(stage,wrist_path+'/'+name)
    shape.CreateRadiusAttr(radius);shape.CreateHeightAttr(height);shape.CreateAxisAttr('Z')
    shape.AddTranslateOp().Set(Gf.Vec3d(*center));shape.CreateDisplayColorAttr([tuple(color)])
    return shape

# Fixed geometry is parented to the wrist, so bracket, closed gripper and pen
# follow actual physics poses together. The pen TCP remains [0,0,.22].
cylinder('ToolAdapter',[0,0,.012],.032,.024,[.45,.47,.49])
cylinder('GripperMotor',[0,0,.047],.030,.046,[.08,.09,.10])
box('GripperPalm',[0,0,.082],[.062,.055,.035],[.12,.13,.14])
for label,side in [("Left",-1),("Right",1)]:
    # Twin closed fingers taper inward, ending on opposite sides of the pen.
    obj=box(f'FingerLink_{label}',[side*.022,0,.120],[.013,.022,.054],[.18,.19,.20])
    angle=-side*.35
    obj.set_local_pose(translation=np.array([side*.022,0,.120]),orientation=np.array([np.cos(angle/2),0,np.sin(angle/2),0]))
    box(f'FingerPad_{label}',[side*.009,0,.155],[.009,.020,.030],[.045,.045,.045])
    cylinder(f'FingerPivot_{label}',[side*.024,0,.098],.008,.008,[.5,.5,.5])
cylinder('PenBarrel',[0,0,.178],.0045,.064,[.06,.22,.62])
cylinder('PenNib',[0,0,.215],.0015,.010,[.08,.08,.09])
for b in BRACKET_BOXES: box(b['id'],b['center'],b['size'],[.07,.08,.09])
box('CameraBody',[-.025,.13,-.025],[.09,.025,.025],[.57,.59,.60])
box('CameraFace',[-.025,.13,-.011],[.088,.023,.003],[.055,.06,.065])
for i,x in enumerate([-.058,-.025,.008]):
    cylinder(f'CameraLens{i}',[x,.13,-.0085],.005,.002,[.08,.14,.21])
light=UsdLux.DomeLight.Define(stage,'/World/Light');light.CreateIntensityAttr(1100.)
sun=UsdLux.DistantLight.Define(stage,'/World/Sun');sun.CreateIntensityAttr(1500.)
cam=Camera(wrist_path+'/WristCamera',name='wrist_camera',resolution=(1280,960),frequency=2)
cam.set_local_pose(translation=np.array([-.025,.13,0]),orientation=np.array([1.,0.,0.,0.]),camera_axes='ros')
cam.set_focal_length(1.93);cam.set_horizontal_aperture(3.84);cam.set_vertical_aperture(2.88);cam.set_clipping_range(.01,10.)
overview=Camera('/World/Overview',name='overview',resolution=(960,720),frequency=2)
def look_at(camera,position,target):
    f=np.array(target)-position;f=f/np.linalg.norm(f);right=np.cross(f,[0,0,1]);right/=np.linalg.norm(right);down=np.cross(f,right)
    q=Rotation.from_matrix(np.column_stack([right,down,f])).as_quat()
    camera.set_world_pose(position=np.array(position),orientation=q[[3,0,1,2]],camera_axes='ros')
look_at(overview,[1.00,1.25,.85],[0,.28,.25]);overview.set_focal_length(2.8);overview.set_horizontal_aperture(3.84);overview.set_vertical_aperture(2.88)
world.reset();robot.initialize();cam.initialize();overview.initialize();cam.add_distance_to_image_plane_to_frame()
names=list(robot.dof_names)
initial=json.loads((ASSETS/'initial.json').read_text())
q=np.array([initial[n] for n in names]);robot.set_joint_positions(q);robot.set_joint_velocities(np.zeros(len(q)));robot.apply_action(ArticulationAction(joint_positions=q))
robot.get_articulation_controller().set_gains(kps=np.full(len(q),100000.),kds=np.full(len(q),10000.))
# Disable gravity on arm links to isolate position-control tracking from unmeasured actuator dynamics.
for prim in stage.Traverse():
    if prim.HasAPI(UsdPhysics.RigidBodyAPI) and str(prim.GetPath()).startswith(str(root)):
        PhysxSchema.PhysxRigidBodyAPI.Apply(prim).CreateDisableGravityAttr(True)
trace=[]; frame_count=0;last_ink=None;stroke_id=-1;ink_points=[];curve=None
frames=OUT/'frames';frames.mkdir(exist_ok=True)
print('WORLD_READY',names,wrist_path,flush=True)
try:
    last_wall=time.monotonic(); profile=np.zeros(5)
    for step in range(args.steps):
        tick=time.monotonic()
        rclpy.spin_once(node,timeout_sec=0.)
        if done: break
        if commands and all(n in commands for n in names): robot.apply_action(ArticulationAction(joint_positions=np.array([commands[n] for n in names])))
        profile[0]+=time.monotonic()-tick;tick=time.monotonic()
        world.step(render=False,update_fabric=True)
        profile[1]+=time.monotonic()-tick;tick=time.monotonic()
        sec=world.current_time;stamp=Time(sec=int(sec),nanosec=int((sec%1)*1e9))
        clock.publish(Clock(clock=stamp))
        msg=JointState();msg.header.stamp=stamp;msg.name=names;msg.position=robot.get_joint_positions().astype(float).tolist();msg.velocity=robot.get_joint_velocities().astype(float).tolist();feedback.publish(msg)
        positions,quaternions=wrist.get_world_poses(usd=False);pos,quat=positions[0],quaternions[0];rot=Rotation.from_quat(quat[[1,2,3,0]]).as_matrix();tip=pos+rot@np.array([0,0,.22])
        contact=abs(tip[2]-PAPER_SURFACE)<.0008 and abs(tip[0])<.099 and abs(tip[1]-.5)<.099
        if recording:
            trace.append([float(sec),*map(float,tip),int(contact),*msg.position])
            if contact:
                point=np.array([tip[0],tip[1],PAPER_SURFACE+.0002])
                if last_ink is None:
                    stroke_id+=1;ink_points=[];curve=UsdGeom.BasisCurves.Define(stage,f'/World/Ink/s{stroke_id}')
                    curve.CreateTypeAttr('linear');curve.CreateWidthsAttr([.0006]);curve.GetWidthsAttr().SetMetadata('interpolation','constant');curve.CreateDisplayColorAttr([(.02,.03,.1)])
                if last_ink is None or np.linalg.norm(point-last_ink)>.00025:
                    ink_points.append(Gf.Vec3f(*point));curve.CreatePointsAttr(ink_points if len(ink_points)>1 else ink_points*2);curve.CreateCurveVertexCountsAttr([max(2,len(ink_points))]);last_ink=point
            else: last_ink=None
        profile[2]+=time.monotonic()-tick;tick=time.monotonic()
        if step%25==0:
            world.render()
            profile[3]+=time.monotonic()-tick;tick=time.monotonic()
            pixels=cam.get_rgba() if not recording or step%250==0 else None
            if pixels is not None and pixels.size:
                pixels=np.asarray(pixels[:,:,:3],dtype=np.uint8)
                im=Image();im.header.stamp=stamp;im.header.frame_id='camera_color_optical_frame';im.height=960;im.width=1280;im.encoding='rgb8';im.step=1280*3;im.data=array('B',pixels.tobytes());rgb.publish(im)
                K=np.asarray(cam.get_intrinsics_matrix());ci=CameraInfo();ci.header=im.header;ci.height=960;ci.width=1280;ci.distortion_model='plumb_bob';ci.d=[0.]*5;ci.k=K.reshape(-1).tolist();ci.r=np.eye(3).reshape(-1).tolist();ci.p=np.column_stack([K,np.zeros(3)]).reshape(-1).tolist();info.publish(ci)
                z=cam.get_current_frame().get('distance_to_image_plane')
                if z is not None and depth.get_subscription_count()>0:
                    dm=Image();dm.header=im.header;dm.height=960;dm.width=1280;dm.encoding='32FC1';dm.step=1280*4;dm.data=array('B',np.asarray(z,dtype=np.float32).tobytes());depth.publish(dm)
                if step%250==0:
                    PILImage.fromarray(pixels).save(OUT/'camera_latest.png')
            ov=overview.get_rgba()
            if ov is not None and ov.size:
                if step%250==0:
                    PILImage.fromarray(ov[:,:,:3].astype(np.uint8)).save(OUT/'overview_latest.png')
                if recording:
                    PILImage.fromarray(ov[:,:,:3].astype(np.uint8)).save(frames/f'{frame_count:06d}.jpg',quality=85);frame_count+=1
        profile[4]+=time.monotonic()-tick
        if step%1000==0:
            print('STEP',step,'SIM',sec,'TIP',tip.tolist(),'RECORD',recording,flush=True)
            print('PROFILE',profile.tolist(),flush=True)
finally:
    np.savetxt(OUT/'actual_tip.csv',np.array(trace),delimiter=',',header='sim_time,x,y,z,contact,'+','.join(names))
    stage.Flatten().Export(str(OUT/'scene.usda'))
    (OUT/'world_result.json').write_text(json.dumps({'frames':frame_count,'samples':len(trace),'ink_strokes':stroke_id+1,'physics_dt':.02,'finished':done},indent=2))
    node.destroy_node();rclpy.shutdown();app.close()
