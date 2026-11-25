# 项目规则（全仓库）

UR7e／D435i Eye-in-Hand 绘画，ROS 2 Humble，实验室必须先 `distrobox enter ros2`。从 README 的流程和 Codex_tasks.md 当前状态开始。

- `src/doodle_ur7e/doodle_ur7e/core.py`：纯几何、笔画路由、轨迹时间；`image.py`：图像轮廓；`vision.py`：标记板/PnP；`ros.py`：ROS 服务/action；`cli.py`：真实用户入口。
- `config/lab.yaml` 是未测量模板，真实参数放忽略的 `config/local.yaml`。不要把现场待测项写成已验证。
- 不自动开合夹爪、启用通信、tuck、切换控制器或接受部分路径。实际动作需要现场监督及具体试验块授权；开发服务器禁止连接真实机器人。
- 运行产物在 `runs/`，不提交人像、现场标定、凭据、构建产物。测试环境 ROS_LOCALHOST_ONLY=1、ROS_DOMAIN_ID=231，默认不占用 GPU；Isaac 仿真仅使用用户明确分配的 GPU，本轮获准共享 GPU 0，不干预其他进程。
- 保留上游许可证与归属，不能把全部上游算法称为自行原创。新增功能应保持 geometry/vision/ROS 分离。
- 以 `scripts/test_humble.sh` 做回归，实际 MoveIt 服务检查用 `scripts/verify_moveit.py`。伪服务测试只证明协议与分支，不证明逆解/真机效果。
- 共享场景对象和其他项目进程不得删除。配置/标定/代码变动使原有规划失效，每次执行重新规划。
- Baymax入口在baymax.py；GPT仅讨论和出图，生成图须用户 `/confirm`。不得让模型文字充当确认或机械臂命令。图片通过不等于机器人可执行，仍需完整实时规划。images用户内容默认不提交，API密钥仅从环境变量读取。
