# 重要错误与修复经验

- Humble `rclpy.node.Node` 已有只读 `clients` / `services` 属性。自定义成员不能复用这些名字；本项目改为 service_clients/service_handles，真实DDS测试验证实例化与调用。
- `launch_ros.actions.Node` 会追加 `--ros-args`。CLI 必须先调用 remove_ros_args，不能把 launch 的 ROS 参数直接交给 argparse。
- 离线 IK/FK 测试必须有 robot_state_publisher 提供模型根帧与 base_link 的变换；仅启动 MoveIt 和发 JointState 不等于 TF 完整。FRAME_TRANSFORM_FAILURE (-21) 不能当成不可达。
- 两个线程不能同时 spin 同一个默认 executor。测试状态发布使用独立 executor；生产 CLI 保持单线程顺序等待。

- MoveIt 会将 CollisionObject 转换到规划 frame 并把位移放到 object.pose。校验必须组合 object.pose 与 primitive_pose，并比较共同坐标系内的几何；不能要求返回消息表示与请求逐字段相同。真实 MoveIt 和消息归一化回归测试覆盖。

重要离线基线：149e2e0（main，已推送私人仓库）。21项测试及实际UR7e MoveIt完整路径/障碍拒绝通过，详见 docs/validation.md。只代表软件离线可回退版本，不能称为真机稳定版；现场参数与运行输出不在Git中。

- action 请求发出后，即使 goal handle 尚未返回也不能把客户端退出当作取消。修复为信号仅设置停止请求、保持 ROS context，等待接受后取消并确认终态。回归必须在真实CLI子进程发送OS信号；只测试执行超时无法覆盖该竞态。
- Lab5 自由拖动后的恢复由 main.py 的 SwitchController 完成，不是单靠退出 freedrive 就能在所有课程环境中假定已恢复。独立程序的运行文档必须给出自己的恢复和状态核验步骤。

## 2026-10-14：Isaac/Humble 集成要点

- Isaac 官方镜像 `/isaac-sim` 目录属于 UID/GID 1234 且权限 750。不能仅切到 UID1000直接运行；使用宿主用户 UID、附加 GID1234、任务自有可写 cache/data/log 挂载，并在任务资产目录运行导入器。ROS 与 Isaac 使用同一 UID 和独立共享 IPC；不同用户/host IPC 曾引发 DDS 和 Carb 共享内存问题，不删除共享服务器的未知 shm 对象。
- 已安装 MoveIt 在 use_sim_time 初始化期间曾因 `/clock` QoS 参数声明异常退出。生成配置明确预置 clock subscription QoS 后启动通过；实际 controller_manager 与 scaled controller无需替换。
- 多 MB 的 Image.data 直接赋 bytes 会触发 Python 每字节类型检查。使用 `array('B', buffer)` 避免这部分开销；无订阅者不发布深度，录像仍保留实际物理采样。
- 只做 physics step 后读取默认 USD XForm 可能得到旧渲染姿态。记录器使用 update_fabric=True 和 get_world_poses(usd=False)，墨迹不能由目标轨迹生成。
- 仿真编排入口也必须像正式 CLI 一样以 SignalHandlerOptions.NO 初始化 rclpy，避免 SIGINT 先销毁 context 导致无法取消目标。真实 scaled controller + 新硬件接口的独立 CPU 合约测试已确认取消终态。
