"""One-shot commands. Only explicit --execute --supervised commands can send trajectories."""
import argparse
import json
from pathlib import Path
import sys
import yaml
from .core import load_config, load_strokes, waypoints


def run(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--config', default='config/local.yaml')
    sub = parser.add_subparsers(dest='command', required=True)
    p = sub.add_parser('prepare'); p.add_argument('image'); p.add_argument('--output',default='runs/image')
    p = sub.add_parser('board'); p.add_argument('--output',default='runs/board.svg')
    p = sub.add_parser('preview'); p.add_argument('strokes'); p.add_argument('--paper',required=True); p.add_argument('--output',default='runs/preview'); p.add_argument('--hover',type=float,default=.10)
    for name in ['mount','scene','doctor']:
        sub.add_parser(name)
    p = sub.add_parser('calibrate'); p.add_argument('--output',default='runs/paper.yaml')
    p = sub.add_parser('capture'); p.add_argument('--output',default='runs/capture.png')
    p = sub.add_parser('draw'); p.add_argument('strokes'); p.add_argument('--paper',required=True); p.add_argument('--output',default='runs/plan'); p.add_argument('--hover',type=float,default=.10); p.add_argument('--execute',action='store_true'); p.add_argument('--supervised',action='store_true')
    p = sub.add_parser('baymax', help='Import an image or discuss/generate/confirm with GPT')
    source = p.add_mutually_exclusive_group(required=True)
    source.add_argument('--image'); source.add_argument('--chat',action='store_true')
    p.add_argument('--images-dir',default='images'); p.add_argument('--paper')
    p.add_argument('--hover',type=float,default=.10)
    p.add_argument('--execute',action='store_true'); p.add_argument('--supervised',action='store_true')
    arguments = list(sys.argv[1:] if argv is None else argv)
    ros_arguments = list(arguments)
    if '--ros-args' in arguments:
        from rclpy.utilities import remove_ros_args
        arguments = remove_ros_args(args=['doodle'] + arguments)[1:]
    args = parser.parse_args(arguments)
    try:
        if args.command == 'prepare':
            from .image import prepare
            prepare(args.image,args.output)
            return
        config = load_config(args.config)
        if args.command == 'baymax':
            if args.execute and (not args.supervised or not args.paper):
                raise ValueError('execution requires --paper and --supervised with an on-site E-stop operator')
            from .baymax import external, dialogue
            output = dialogue(config,args.images_dir) if args.chat else external(args.image,config,args.images_dir)
            if output is None or not args.paper:
                return
            args.command='draw'; args.strokes=str(output/'strokes.json'); args.output=str(output/'robot_plan')
        if args.command == 'board':
            from .vision import board
            Path(args.output).parent.mkdir(parents=True,exist_ok=True)
            board(config['paper'],args.output)
            print(args.output)
            return
        if args.command in ['draw','preview']:
            lines = load_strokes(args.strokes)
            calibration = yaml.safe_load(Path(args.paper).read_text())
            if args.command == 'preview':
                from .image import preview
                out = Path(args.output); out.mkdir(parents=True,exist_ok=True)
                tips,links = waypoints(lines,config,calibration,args.hover)
                preview([x.tolist() for x in lines],out/'preview.svg')
                (out/'waypoints.json').write_text(json.dumps({'tip':tips.tolist(),'ik_link':links.tolist()},indent=2))
                print(f'{len(tips)} Cartesian waypoints; no ROS/motion used.')
                return
            if args.execute and not args.supervised:
                raise ValueError('execution requires --supervised with an on-site E-stop operator')
        import rclpy
        from .ros import Robot
        from rclpy.signals import SignalHandlerOptions
        rclpy.init(args=['doodle']+ros_arguments if '--ros-args' in ros_arguments else [], signal_handler_options=SignalHandlerOptions.NO)
        robot = Robot(config)
        try:
            if args.command == 'mount':
                robot.mount()
            elif args.command == 'scene':
                robot.scene(apply=True); print('Configured collision boxes applied and verified.')
            elif args.command == 'calibrate':
                robot.calibrate(args.output)
            elif args.command == 'capture':
                import cv2
                img,_,_ = robot.capture()
                Path(args.output).parent.mkdir(parents=True,exist_ok=True)
                if not cv2.imwrite(args.output,img):
                    raise ValueError('image write failed')
                print(args.output)
            elif args.command == 'doctor':
                state = robot.state()
                T = robot.fk(state)
                import numpy as np
                from scipy.spatial.transform import Rotation
                tip = T[:3,3] + T[:3,:3] @ np.asarray(config['tip_in_link'])
                angle = (Rotation.from_quat(config['link_quaternion']).inv() *
                         Rotation.from_matrix(T[:3,:3])).magnitude()
                print(f'Current tip in {config["base_frame"]}: {tip.tolist()} m; '
                      f'orientation error {angle:.4f} rad (planning requires <=0.03).')
                robot.controller_check()
                robot.scene()
                image,msg,_ = robot.capture()
                stamp = rclpy.time.Time.from_msg(msg.header.stamp)
                robot.until(lambda: robot.buffer.can_transform(config['base_frame'],msg.header.frame_id,stamp), 3.)
                print('Live joint state, FK, controller, action, scene, camera and timestamped TF passed. No motion sent.')
            elif args.command == 'draw':
                robot.plan(lines,calibration,args.hover,args.output,args.execute)
        finally:
            robot.destroy_node()
            if rclpy.ok():
                rclpy.shutdown()
    except KeyboardInterrupt:
        print('Interrupted; verify stopped, use E-stop if needed.',file=sys.stderr)
        raise SystemExit(130)
    except (ValueError, RuntimeError, TimeoutError, OSError) as exc:
        print(f'ERROR: {exc}',file=sys.stderr)
        raise SystemExit(1)


def main(argv=None):
    from .shutdown import stop_signals
    with stop_signals():
        run(argv)


if __name__ == '__main__':
    main()
