# Baymax：选图或先聊天再画

这是可选入口，不改变原有 `prepare` / `draw` 流程。GPT 只负责讨论和出图，不返回机械臂指令，也没有 ROS、终端或执行工具权限。现场仍先进入原来的环境：

```bash
distrobox enter ros2
source /opt/ros/humble/setup.bash
cd ~/ros_workspaces/Doodle_Ur7e
colcon build --symlink-install --packages-select doodle_ur7e
source install/setup.bash
```

先按 [START](../START.md) 完成通信、相机、夹笔、TCP/手眼/纸面标定和场景核对。现场用 `config/local.yaml`，不能用 `config/isaac.yaml` 或服务器仿真纸面标定。示例命令不会自动夹笔、tuck 或切换控制器。

## 不用 GPT：直接选择图片

只处理、保存预览，不连接 ROS、不联网：

```bash
ros2 run doodle_ur7e doodle --config config/local.yaml baymax \
  --image images/external/smiley.png
```

增加 `--paper` 会从实时关节状态做 MoveIt 规划，默认悬空 10 cm，不发送动作：

```bash
ros2 run doodle_ur7e doodle --config config/local.yaml baymax \
  --image images/external/smiley.png --paper runs/paper.yaml
```

完成现场准备、低速悬空验收后，在现场急停人员监督下执行接触绘画：

```bash
ros2 run doodle_ur7e doodle --config config/local.yaml baymax \
  --image images/external/smiley.png --paper runs/paper.yaml \
  --hover 0 --execute --supervised
```

把路径换成自己的 PNG/JPEG/WebP/BMP 即可。每次执行重新处理并规划；不会加载旧轨迹直接执行。单独使用 `--execute` 不会绕过现场监督或纸面校验。

## 用 GPT：讨论 → 出图 → 人确认 → 检查 → 绘画

在 **Distrobox 内的运行终端**设置官方 API key；不要粘贴到聊天里或提交到仓库。例如 Bash 中可隐藏输入：

```bash
read -rsp 'OpenAI API key: ' OPENAI_API_KEY; echo
export OPENAI_API_KEY
ros2 run doodle_ur7e doodle --config config/local.yaml baymax --chat
```

对话示例：

```text
你: 机器人baymax你好，请你帮我画一幅笑脸
Baymax: ……讨论简单线稿需求……
你: 只要圆脸、两只眼睛和弯弯的嘴巴
你: /generate
图片已生成，请打开查看: .../images/generated/<编号>/review.png
你: 嘴巴再大一点
你: /generate
你: /confirm
```

`/generate` 才调用图像生成；终端会给出图片位置，需用文件浏览器或图片查看器打开。`/confirm` 是你对当前图的明确确认，模型说“确认”不会触发它。继续对话会取消旧图的待确认状态；再次出图也要重新确认。`/quit` 或 Ctrl+C 退出，不会自动绘画。

上述命令确认后只处理并保存图片。若启动时加 `--paper runs/paper.yaml`，确认后还会规划但不执行。若已完成现场准备，希望确认图后自动检查并绘画：

```bash
ros2 run doodle_ur7e doodle --config config/local.yaml baymax --chat \
  --paper runs/paper.yaml --hover 0 --execute --supervised
```

即便启动时允许执行，GPT 模式也必须等你查看并 `/confirm`。确认后图片检查、标定/配置核对或路径规划任何一项失败，都不发送轨迹；要简化图片或修正现场配置后重新运行。模型理解、图像的语义/美观仍由你判断，几何检查不能替代该判断。

## 模型、保存和限制

使用官方 Responses API，主模型 `gpt-5.6-sol` 负责对话并调用 `gpt-image-2.5-flare` 图像工具；并非把文本模型直接作为 Images API 的模型。参数集中在 `doodle_ur7e/baymax.py`。只在 `--chat` 使用 API，费用/权限取决于你配置的 API 账户，不读取或复制 Codex 登录凭据。

图片保存在仓库根 `images/` 下；从其他工作目录启动时可显式指定 `--images-dir /绝对路径/Doodle_Ur7e/images`。目录说明见 [图片库](../images/README.md)。原图、对话、确认记录和处理结果本地保留，默认不推送 Git。API 请求设 `store: false`，这不等同于对供应商所有日志保留政策的承诺。

新入口检查文件格式/尺寸、空白或过密图像、有限且归一化的笔画坐标，限制 ≤120 笔画、≤3000 点、纸面笔迹总长 ≤2.5 m。复杂图片会被拒绝，不静默截断成残缺图案。`checks.json` 中图片通过时仍明确标记机器人可行性尚需实时规划；之后共用原有完整笛卡尔路径、碰撞、关节跳变、速度/加速度/时长、起点一致性及执行前场景复查。真实接触力、手眼标定精度和突发环境变化仍要现场保障。

接口依据（2026-10-14 核对）：[GPT-5.6 Sol](https://developers.openai.com/api/docs/models/gpt-5.6-sol)、[Responses 图像生成工具](https://developers.openai.com/api/docs/guides/tools-image-generation)。本次环境未配置 API key，因此联网模型调用尚未实测；已用固定 API 响应测试确认流程与接口解析，外来图片链路另有实际 Humble/MoveIt 验证。
