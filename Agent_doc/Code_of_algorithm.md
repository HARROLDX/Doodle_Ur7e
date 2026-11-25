# 算法、适配依据与部署基线

## 来源与版本

参考 [DoodleDroid](https://github.com/HarrisonBounds/DoodleDroid/tree/1732b5be7e7e4ff19ab84d9195e61e29bc6a1eff)，其目标是 Franka／ROS 2 Jazzy。参考用户课程仓库 `HARROLDX/eecs106a-fa26` 的 Lab 5，revision `9c2c1d3efe087500c3c8abc0bbbb00dae268ea05`；课程代码只读取核对接口，没有复制整份作业到这里。

Lab 5 的真实入口 `planning/main.py` 使用 `/scaled_joint_trajectory_controller/follow_joint_trajectory`，IK 使用 `ur_manipulator`、`wrist_3_link`、`base_link`，默认朝下四元数 xyzw `[0,1,0,0]`。课程 PDF 规定先进入 Distrobox，`ur7e_utils enable_comms` 还会开合夹爪。照片只支持“腕部相机、夹爪向下”这一安装观察，不提供精确外参或纸高。

手册入口：

- [Humble GetCartesianPath](https://github.com/moveit/moveit_msgs/blob/humble/srv/GetCartesianPath.srv)：header、start_state、IK link、max_step、fraction 与碰撞检查。
- [UR scaled controller](https://docs.universal-robots.com/Universal_Robots_ROS2_Documentation/doc/ur_robot_driver/ur_controllers/doc/index.html)：速度缩放、暂停与 FollowJointTrajectory。服务器安装版本以 docs/validation.md 为准，现场课程版本尚未查询。
- [Humble tf2](https://docs.ros.org/en/humble/Tutorials/Intermediate/Tf2/Tf2-Main.html)：按曝光时刻查询动态变换。
- 本项目 README、docs/calibration.md 是部署步骤，不替代课程现场安全规程。

## 上游到新实现

| 上游 | 本项目 | 改变理由 |
|---|---|---|
| process.py + linedraw/linedraw.py | image.py | 保留 getdots/connectdots 扫描轮廓；去掉随机扰动、写入安装目录和弹浏览器；空图报错、保比例 |
| stroke_router_utils.py / route_planner.py | core.route / waypoints | 最近端点排序、允许反转，不建稠密距离矩阵；完整纸面旋转，抬笔按法向 |
| calibrator.py，四 AprilTag 位姿均值 | vision.estimate / Robot.calibrate | 自带四 ArUco 标记板、所有16角点联合 PnP；曝光时间 TF、多帧一致性 |
| motion_planner.py，Franka ready 等 | Robot.plan / execute | Humble Cartesian service + UR scaled action；无自动 home/抓笔 |

原版 `_route_callback(self, request, response)` 被注册为 subscription，签名与订阅回调不一致；上游 stroke sort 的反向距离更新还有疑点。本实现没有直接运行这套 ROS 封装；路由的反转行为有回归测试。需要评估的是交付代码，而非因为原库功能多就假定它能直接运行在 UR 上。

## 数据流与几何

prepare 读取 BGR 图片，灰度、自动对比度、Gaussian blur、Canny，再水平和垂直扫描连成笔画，approxPolyDP 简化。按整张图画布的长边标准化到 [0,1] 方形空间，保持长宽比，最近端点排序。输入被限制为最多2000笔、20000点；无有效边缘直接报错。

paper +X 朝右、+Y 朝上、+Z 离开纸面；图片 +Y 朝下，映射时反转一次。沿纸面法向抬笔，相邻笔画之间始终有抬笔、平移、落笔段。`T_base_paper` 来自联合PnP及曝光时刻 `T_base_camera`，不是把末端相机当固定外部相机。平面法向需向上、倾斜不超过10度。

笔尖与腕部：`p_wrist = p_tip - R_wrist * tip_in_link`。笔的 XYZ 偏心不能用单个 base Z 加常数替代。腕部姿态在绘画中固定，初始姿态应匹配。当前位置由实时 JointState + FK 得到，不使用示例关节角控制真机。

相机 TF：RealSense 原生 root→optical 链继续由驱动拥有，本项目只发布 wrist→camera_link，按已测量的 wrist→depth optical 与驱动 root→depth optical 组合。课程 Lab5 broadcaster 不应同时运行。

## 规划、时序与失败

读取当前场景和状态；所有配置中的世界/附着碰撞盒必须存在且几何匹配。先在当前安全高度横移，再垂直下降至第一笔上方。整个图一次 Cartesian 求解，要求 fraction≈1 和 SUCCESS；检查六关节名称、有限值、最大相邻关节差、起始状态、末端FK。`start_state.is_diff=True` 保留场景的附着工具。

控制器使用带位置和零速度、不带加速度字段的点，形成逐段三次平滑插值（每个采样点停止，故较慢）。每段时长满足峰值速度 `1.5*abs(dq)/dt`、峰值加速度 `6*abs(dq)/dt²`。另用每关节到笔尖距离不超过2m的 UR7e+≤0.4m工具保守 Jacobian 上界约束笔尖速度。没有增加力控制或接触压入量。

MoveIt 是离散碰撞与路径计算，不能保证无限分辨率的连续碰撞或真实接触力；2mm Cartesian max_step 限制离散间距。工具几何与相机/TCP/机器人校准误差必须由现场验证约束。保守插值仍可能产生细微 Cartesian 偏差，现场悬空和小图验收不可省略。

执行前复核场景、控制器和关节起点。只通过显式执行命令发送一次 action，失败/中断尝试取消；网络故障无法保证远端停止，现场物理急停负责最终中止。结束停在末笔抬起位置。机器人移动、纸面变化、换笔均须重新核验，不自动恢复任务。

## 部署

严格采用 README 的 Distrobox 工作站流程。服务器 Docker 仅用于复现 Humble 编译和离线规划，并不取代现场 Distrobox。按“通信先于夹笔→传感器/MoveIt→测量本地配置→场景→纸面定位→只规划→10mm悬空→小图接触→人像”推进，现场监督与具体动作授权是独立前提。
