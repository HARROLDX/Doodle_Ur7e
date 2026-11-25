"""Humble service/action adapter. Construction and planning never command motion."""
import copy
import os
import json
import time
from pathlib import Path
import numpy as np
import yaml
from scipy.spatial.transform import Rotation
import rclpy
from rclpy.node import Node
from rclpy.time import Time
from rclpy.qos import qos_profile_sensor_data
from rclpy.action import ActionClient
from action_msgs.msg import GoalStatus
from sensor_msgs.msg import JointState, Image, CameraInfo
from geometry_msgs.msg import Pose, TransformStamped
from moveit_msgs.srv import GetCartesianPath, GetPositionFK, ApplyPlanningScene, GetPlanningScene
from moveit_msgs.msg import PlanningScene, PlanningSceneComponents, CollisionObject, AttachedCollisionObject, DisplayTrajectory
from control_msgs.action import FollowJointTrajectory
from controller_manager_msgs.srv import ListControllers
from rcl_interfaces.srv import GetParameters
from shape_msgs.msg import SolidPrimitive
from tf2_ros import Buffer, TransformListener, StaticTransformBroadcaster
from cv_bridge import CvBridge
from .core import (transform, waypoints, approach, timing, check_workspace, paper_transform)
from .vision import estimate
from .shutdown import check_stop, cancellation_cleanup

GOAL_RESPONSE_TIMEOUT = 10.0


def matrix(msg):
    t = msg.transform if hasattr(msg, 'transform') else msg
    p = t.translation if hasattr(t, 'translation') else t.position
    q = t.rotation if hasattr(t, 'rotation') else t.orientation
    return transform([p.x,p.y,p.z], [q.x,q.y,q.z,q.w])


def pose(xyz, quat):
    out = Pose()
    out.position.x, out.position.y, out.position.z = map(float, xyz)
    out.orientation.x, out.orientation.y, out.orientation.z, out.orientation.w = map(float, quat)
    return out


class Robot(Node):
    def __init__(self, config):
        if config.get('execution_environment') == 'isaac' and os.environ.get('ROS_DOMAIN_ID') != '231':
            raise ValueError('Isaac simulation requires isolated ROS_DOMAIN_ID=231')
        super().__init__('doodle_ur7e', parameter_overrides=[
            rclpy.parameter.Parameter('use_sim_time', value=config.get('use_sim_time', False))])
        self.c = config
        self.joints = None
        self.received = 0.
        self.image = self.info = None
        self.buffer = Buffer()
        self.listener = TransformListener(self.buffer, self)
        self.subs = [self.create_subscription(JointState, config['joint_states'], self.on_joints, qos_profile_sensor_data),
                     self.create_subscription(Image, config['camera']['image_topic'], self.on_image, qos_profile_sensor_data),
                     self.create_subscription(CameraInfo, config['camera']['info_topic'], self.on_info, qos_profile_sensor_data)]
        self.display = self.create_publisher(DisplayTrajectory, '/display_planned_path', 1)
        self.action = ActionClient(self, FollowJointTrajectory, config['action'])
        self.service_clients = {}
        self.goal = None

    def on_joints(self, msg):
        if set(self.c['joint_names']).issubset(msg.name):
            self.joints = msg
            self.received = time.monotonic()

    def on_image(self, msg):
        self.image = msg

    def on_info(self, msg):
        self.info = msg

    def until(self, predicate, timeout=10.):
        end = time.monotonic()+timeout
        check_stop()
        while rclpy.ok() and not predicate():
            check_stop()
            if time.monotonic() > end:
                raise TimeoutError('ROS wait timed out')
            rclpy.spin_once(self, timeout_sec=.05)

    def result(self, future, timeout=30.):
        self.until(future.done, timeout)
        result = future.result()
        if result is None:
            raise RuntimeError('ROS request returned no result')
        return result

    def call(self, typ, name, req):
        if name not in self.service_clients:
            self.service_clients[name] = self.create_client(typ, name)
        cli = self.service_clients[name]
        if not cli.wait_for_service(timeout_sec=5.):
            raise RuntimeError(f'{name} unavailable; check MoveIt/environment')
        # Allow recovery from an intermittent read-only status response timeout.
        # Retry only this idempotent status query, never commands or actions.
        attempts = 2 if name in ['/controller_manager/list_controllers', '/get_planning_scene'] else 1
        for attempt in range(attempts):
            future = cli.call_async(req)
            try:
                return self.result(future, 120. if name == '/compute_cartesian_path' else 30.)
            except TimeoutError as exc:
                cli.remove_pending_request(future)
                future.cancel()
                if attempt + 1 == attempts:
                    raise TimeoutError(f'{name}: response timed out after {attempts} attempt(s); no retry of motion commands') from exc
                self.get_logger().warning(f'{name}: status response timed out; retrying read-only query once')

    def state(self):
        self.until(lambda: self.joints is not None)
        # Spin to obtain a newly received state, not just a cached valid timestamp.
        old = self.received
        self.until(lambda: self.received > old, 3.)
        msg = self.joints
        age = (self.get_clock().now()-Time.from_msg(msg.header.stamp)).nanoseconds/1e9
        if not 0 <= age < 1.0:
            raise ValueError(f'joint state timestamp stale/future: {age}')
        if len(msg.position) != len(msg.name) or not np.isfinite(msg.position).all():
            raise ValueError('invalid joint state')
        if msg.velocity and (len(msg.velocity) != len(msg.name) or not np.isfinite(msg.velocity).all() or max(abs(v) for v in msg.velocity) > .01):
            raise ValueError('robot must be stationary before planning/execution')
        return copy.deepcopy(msg)

    def fk(self, state):
        req = GetPositionFK.Request()
        req.header.frame_id = self.c['base_frame']
        req.fk_link_names = [self.c['ik_link']]
        req.robot_state.joint_state = state
        res = self.call(GetPositionFK, '/compute_fk', req)
        if res.error_code.val != 1 or len(res.pose_stamped) != 1:
            raise ValueError('FK failed')
        if res.pose_stamped[0].header.frame_id != self.c['base_frame']:
            raise ValueError('FK returned wrong frame')
        return matrix(res.pose_stamped[0].pose)

    def controller_check(self):
        res = self.call(ListControllers, '/controller_manager/list_controllers', ListControllers.Request())
        states = {x.name:x.state for x in res.controller}
        if states.get('scaled_joint_trajectory_controller') != 'active':
            raise ValueError('scaled controller inactive; finish freedrive using course procedure')
        if any(states.get(x) == 'active' for x in ['freedrive_mode_controller','forward_position_controller','forward_velocity_controller']):
            raise ValueError('conflicting motion controller active')
        if not self.action.wait_for_server(timeout_sec=5.):
            raise ValueError('trajectory action unavailable')

    def scene(self, apply=False):
        expected = PlanningScene(is_diff=True)
        expected.robot_state.is_diff = True
        for key in ['world_boxes', 'attached_boxes']:
            for item in self.c[key]:
                if np.any(np.array(item['size']) <= 0):
                    raise ValueError('collision box must have positive size')
                frame = self.c['base_frame'] if key == 'world_boxes' else self.c['ik_link']
                obj = CollisionObject(id=item['id'], operation=CollisionObject.ADD)
                obj.header.frame_id = frame
                obj.primitives = [SolidPrimitive(type=SolidPrimitive.BOX, dimensions=list(map(float,item['size'])))]
                obj.primitive_poses = [pose(item['center'], [0,0,0,1])]
                if key == 'world_boxes':
                    expected.world.collision_objects.append(obj)
                else:
                    expected.robot_state.attached_collision_objects.append(AttachedCollisionObject(
                        link_name=frame, object=obj, touch_links=item['touch_links']))
        if apply:
            res = self.call(ApplyPlanningScene, '/apply_planning_scene', ApplyPlanningScene.Request(scene=expected))
            if not res.success:
                raise ValueError('planning scene rejected')
        req = GetPlanningScene.Request()
        req.components.components = (PlanningSceneComponents.WORLD_OBJECT_GEOMETRY |
                                     PlanningSceneComponents.ROBOT_STATE_ATTACHED_OBJECTS)
        actual = self.call(GetPlanningScene, '/get_planning_scene', req).scene
        actual_world = {x.id:x for x in actual.world.collision_objects}
        actual_attached = {x.object.id:x for x in actual.robot_state.attached_collision_objects}
        for obj in expected.world.collision_objects:
            self.compare_box(obj, actual_world.get(obj.id))
        for item in expected.robot_state.attached_collision_objects:
            other = actual_attached.get(item.object.id)
            if other is None or other.link_name != item.link_name or set(other.touch_links) != set(item.touch_links):
                raise ValueError('attached collision geometry missing/mismatched')
            self.compare_box(item.object, other.object)
        return actual

    def compare_box(self, expected, actual):
        if actual is None or len(actual.primitives) != 1 or len(actual.primitive_poses) != 1:
            raise ValueError(f'scene object missing: {expected.id}; run scene command')
        T_expected = matrix(expected.pose) @ matrix(expected.primitive_poses[0])
        T_actual = matrix(actual.pose) @ matrix(actual.primitive_poses[0])
        if actual.header.frame_id != expected.header.frame_id:
            self.until(lambda: self.buffer.can_transform(expected.header.frame_id, actual.header.frame_id, Time()), 3.)
            T_actual = matrix(self.buffer.lookup_transform(expected.header.frame_id, actual.header.frame_id, Time())) @ T_actual
        if (actual.primitives[0].type != SolidPrimitive.BOX or actual.meshes or actual.planes
                or not np.allclose(actual.primitives[0].dimensions, expected.primitives[0].dimensions, atol=1e-6)
                or not np.allclose(T_actual, T_expected, atol=1e-6)):
            raise ValueError(f'scene object geometry differs: {expected.id}')

    def plan(self, lines, calibration, hover, output, execute=False):
        if execute:
            if not all(self.c['verified'].get(k) is True for k in ['pen_tcp','camera_mount','workspace','scene']):
                raise ValueError('complete measured config verification before execution')
            simulation = self.c.get('execution_environment') == 'isaac'
            if calibration.get('environment') == 'isaac' and not simulation:
                raise ValueError('simulation calibration cannot execute on hardware')
            if calibration.get('verified') is not True or (calibration.get('synthetic', False) and not simulation):
                raise ValueError('paper calibration must be physically verified, non-synthetic')
        if np.linalg.norm(self.c['tip_in_link']) > .4:
            raise ValueError('tool exceeds supported 0.4m timing bound')
        self.scene()
        start = self.state()
        T = self.fk(start)
        R = Rotation.from_quat(self.c['link_quaternion']).as_matrix()
        if Rotation.from_matrix(R.T@T[:3,:3]).magnitude() > .03:
            raise ValueError('start orientation differs >0.03rad; position robot manually first')
        tip = T[:3,3] + T[:3,:3]@self.c['tip_in_link']
        tips, _ = waypoints(lines, self.c, calibration, hover)
        prefix = approach(tip, tips[0], self.c, calibration)
        # Current orientation must match exactly enough to avoid a tip swing on approach.
        targets = np.vstack([prefix[1:], tips[1:]]) - R@self.c['tip_in_link']
        req = GetCartesianPath.Request()
        req.header.frame_id = self.c['base_frame']
        req.start_state.joint_state = start
        req.start_state.is_diff = True  # preserve attached tools from planning scene
        req.group_name, req.link_name = self.c['group'], self.c['ik_link']
        req.waypoints = [pose(v, self.c['link_quaternion']) for v in targets]
        req.max_step = self.c['motion']['max_step']
        req.jump_threshold = 2.
        req.avoid_collisions = True
        # These fields exist in the installed Humble schema; timing is rebuilt below.
        if hasattr(req, 'revolute_jump_threshold'):
            req.revolute_jump_threshold = self.c['motion']['max_joint_step']
        if hasattr(req, 'max_velocity_scaling_factor'):
            req.max_velocity_scaling_factor = .05
            req.max_acceleration_scaling_factor = .05
        res = self.call(GetCartesianPath, '/compute_cartesian_path', req)
        if res.error_code.val != 1 or not np.isfinite(res.fraction) or res.fraction < 1.0-1e-8:
            raise ValueError(f'incomplete path rejected: fraction={res.fraction}, error={res.error_code.val}')
        trajectory = res.solution.joint_trajectory
        if len(trajectory.joint_names) != 6 or set(trajectory.joint_names) != set(self.c['joint_names']):
            raise ValueError('trajectory joints differ from configured UR7e')
        positions_for_timing = np.asarray([x.positions for x in trajectory.points])
        bounds = None
        if self.c['motion'].get('geometry_tip_timing', False):
            from .kinematics import TipKinematics
            description = self.call(GetParameters, '/move_group/get_parameters',
                                    GetParameters.Request(names=['robot_description']))
            if len(description.values) != 1 or not description.values[0].string_value:
                raise ValueError('live URDF unavailable for geometry timing')
            model = TipKinematics(description.values[0].string_value, self.c['base_frame'],
                                  self.c['ik_link'], list(trajectory.joint_names), self.c['tip_in_link'])
            state_positions = dict(zip(start.name, start.position))
            model_T = model.wrist([state_positions[n] for n in trajectory.joint_names])
            if not np.allclose(model_T, T, atol=1e-6, rtol=0):
                raise ValueError('URDF timing FK differs from MoveIt FK')
            bounds = model.segment_bounds(positions_for_timing)
            # Driver URDF includes physical factory calibration; MoveIt may use
            # nominal geometry. Bound speed under BOTH, never silently substitute.
            hardware_description = self.call(GetParameters, '/robot_state_publisher/get_parameters',
                                             GetParameters.Request(names=['robot_description']))
            if len(hardware_description.values) != 1 or not hardware_description.values[0].string_value:
                raise ValueError('driver URDF unavailable for geometry timing')
            hardware_model = TipKinematics(hardware_description.values[0].string_value,
                                          self.c['base_frame'], self.c['ik_link'],
                                          list(trajectory.joint_names), self.c['tip_in_link'])
            bounds = np.maximum(bounds, hardware_model.segment_bounds(positions_for_timing))
        times = timing(positions_for_timing, self.c, bounds)
        initial = dict(zip(start.name, start.position))
        if max(abs(initial[n]-v) for n,v in zip(trajectory.joint_names, trajectory.points[0].positions)) > .01:
            raise ValueError('trajectory start differs from measured joints')
        for point, sec in zip(trajectory.points, times):
            ns = int(round(sec*1e9))
            point.time_from_start.sec, point.time_from_start.nanosec = divmod(ns, 10**9)
            point.velocities = [0.] * 6
            point.accelerations = []
            point.effort = []
        # Verify final FK against the intended lifted tip (detect wrong link/frame).
        final = copy.deepcopy(start)
        positions = dict(zip(trajectory.joint_names, trajectory.points[-1].positions))
        final.position = [positions.get(n,v) for n,v in zip(start.name,start.position)]
        final_T = self.fk(final)
        end_tip = final_T[:3,3] + final_T[:3,:3]@self.c['tip_in_link']
        if np.linalg.norm(end_tip-tips[-1]) > .002:
            raise ValueError('final FK misses requested tip by >2mm')
        out = Path(output); out.mkdir(parents=True,exist_ok=True)
        report = {'fraction':res.fraction, 'joint_names':list(trajectory.joint_names),
                  'points':len(times), 'duration_seconds':float(times[-1]), 'hover_m':hover,
                  'executed':False, 'tip_waypoints':np.vstack([prefix[:-1],tips]).tolist(),
                  'joint_positions':[list(x.positions) for x in trajectory.points], 'times':times.tolist()}
        (out/'plan.json').write_text(json.dumps(report, indent=2))
        self.display.publish(DisplayTrajectory(trajectory_start=req.start_state, trajectory=[res.solution]))
        print(f'Complete Cartesian path: {len(times)} points, {times[-1]:.1f}s minimum; report {out}/plan.json')
        if execute:
            self.controller_check()
            self.scene()
            now = self.state()
            current = dict(zip(now.name,now.position))
            if max(abs(current[n]-v) for n,v in zip(trajectory.joint_names,trajectory.points[0].positions)) > .01:
                raise ValueError('robot moved after planning; replan')
            self.execute(trajectory)
            report['executed'] = True
            (out/'plan.json').write_text(json.dumps(report, indent=2))

    def execute(self, trajectory):
        # No execution from saved plans: every command replans from live state.
        check_stop()
        goal = FollowJointTrajectory.Goal(trajectory=trajectory)
        pending = self.action.send_goal_async(goal)
        result_future = None
        try:
            self.goal = self.result(pending, GOAL_RESPONSE_TIMEOUT)
            if not self.goal.accepted:
                raise RuntimeError('controller rejected trajectory')
            result_future = self.goal.get_result_async()
            result = self.result(result_future, self.c['motion']['execution_timeout'])
            if result.status != GoalStatus.STATUS_SUCCEEDED or result.result.error_code != 0:
                raise RuntimeError(f'controller failed: {result.status}, {result.result.error_string}')
        except BaseException:
            # A sent goal belongs to us even before its acceptance reply arrives.
            # Preserve the context and drain that reply before canceling this goal.
            with cancellation_cleanup():
                try:
                    if self.goal is None:
                        self.goal = self.result(pending, 10.)
                    if self.goal.accepted:
                        if result_future is None:
                            result_future = self.goal.get_result_async()
                        if not result_future.done():
                            response = self.result(self.goal.cancel_goal_async(), 5.)
                            if response.return_code != 0 and not result_future.done():
                                raise RuntimeError('controller did not accept cancellation')
                        final = self.result(result_future, 5.)
                        if final.status not in (GoalStatus.STATUS_CANCELED, GoalStatus.STATUS_ABORTED,
                                               GoalStatus.STATUS_SUCCEEDED):
                            raise RuntimeError('goal is not terminal')
                        print(f'Goal terminal status confirmed: {final.status}.')
                    else:
                        print('Goal rejected; no trajectory accepted.')
                except Exception as exc:
                    print(f'CANCEL UNCONFIRMED: use physical E-stop now. {exc}')
            raise
        finally:
            self.goal = None

    def capture(self):
        self.until(lambda: self.image is not None and self.info is not None and
                   abs((Time.from_msg(self.image.header.stamp)-Time.from_msg(self.info.header.stamp)).nanoseconds)<100000000)
        msg, info = self.image, self.info
        age = (self.get_clock().now()-Time.from_msg(msg.header.stamp)).nanoseconds/1e9
        skew = abs((Time.from_msg(msg.header.stamp)-Time.from_msg(info.header.stamp)).nanoseconds/1e9)
        if not 0 <= age < .5 or skew > .1 or msg.header.frame_id != info.header.frame_id:
            raise ValueError('camera image/info stale, unsynchronized or mismatched frame')
        if (msg.width,msg.height) != (info.width,info.height) or info.k[0] <= 0:
            raise ValueError('invalid camera intrinsics/image size')
        if info.distortion_model not in ['plumb_bob','rational_polynomial']:
            raise ValueError('unsupported CameraInfo distortion model; use color image_raw')
        return CvBridge().imgmsg_to_cv2(msg, 'bgr8'), msg, info

    def calibrate(self, output):
        poses, errors = [], []
        last = None
        deadline = time.monotonic()+(120 if self.c.get('execution_environment') == 'isaac' else 30)
        while len(poses) < 12:
            check_stop()
            if time.monotonic() > deadline:
                raise TimeoutError('12 paper samples not obtained')
            rclpy.spin_once(self, timeout_sec=.1)
            if self.image is None or self.image.header.stamp == last:
                continue
            image, msg, info = self.capture()
            last = copy.deepcopy(msg.header.stamp)
            T_cam_paper, error = estimate(image, np.array(info.k).reshape(3,3), np.array(info.d), self.c['paper'])
            # Wait for the robot transform at exposure time; NEVER substitute latest.
            stamp = Time.from_msg(msg.header.stamp)
            self.until(lambda: self.buffer.can_transform(self.c['base_frame'], msg.header.frame_id, stamp), 10. if self.c.get('execution_environment') == 'isaac' else 2.)
            T_base_cam = matrix(self.buffer.lookup_transform(self.c['base_frame'],msg.header.frame_id,stamp))
            poses.append(T_base_cam@T_cam_paper); errors.append(error)
            if self.c.get('execution_environment') == 'isaac':
                print(f'Calibration sample {len(poses)}/12, RMS {error:.3f}px', flush=True)
        xyz = np.array([T[:3,3] for T in poses])
        rotations = Rotation.from_matrix(np.array([T[:3,:3] for T in poses]))
        mean = rotations.mean()
        if np.max(np.linalg.norm(xyz-xyz.mean(axis=0),axis=1)) > .002 or np.max((mean.inv()*rotations).magnitude()) > .02:
            raise ValueError('unstable paper pose; improve lighting/mount/marker visibility')
        cal = {'base_frame':self.c['base_frame'], 'paper_size':[self.c['paper']['width'],self.c['paper']['height']],
               'position':xyz.mean(axis=0).tolist(), 'quaternion':mean.as_quat().tolist(),
               'reprojection_rms_px':float(np.mean(errors)), 'verified':False, 'synthetic':False,
               'captured_unix':time.time()}
        if self.c.get('execution_environment') == 'isaac':
            cal.update(environment='isaac', synthetic=True)
        paper_transform(cal,self.c)
        Path(output).parent.mkdir(parents=True,exist_ok=True)
        Path(output).write_text(yaml.safe_dump(cal))
        print(f'Saved {output}; verify physical XY and paper height before setting verified: true.')

    def mount(self):
        c = self.c['camera']
        self.until(lambda: self.buffer.can_transform(c['root_frame'],c['optical_frame'],Time()), 60.)
        # RealSense owns root->optical. Bridge wrist->root without a second optical parent.
        root_optical = matrix(self.buffer.lookup_transform(c['root_frame'],c['optical_frame'],Time()))
        wrist_optical = transform(c['mount_position'], c['mount_quaternion'])
        T = wrist_optical @ np.linalg.inv(root_optical)
        msg = TransformStamped()
        msg.header.frame_id, msg.child_frame_id = c['mount_link'], c['root_frame']
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.transform.translation.x,msg.transform.translation.y,msg.transform.translation.z = map(float,T[:3,3])
        q = Rotation.from_matrix(T[:3,:3]).as_quat()
        msg.transform.rotation.x,msg.transform.rotation.y,msg.transform.rotation.z,msg.transform.rotation.w = map(float,q)
        self.broadcaster = StaticTransformBroadcaster(self)
        self.broadcaster.sendTransform(msg)
        print('Published wrist->camera root; do not also run Lab5/Lab7 camera TF publishers.')
        while rclpy.ok():
            check_stop()
            rclpy.spin_once(self, timeout_sec=.1)
