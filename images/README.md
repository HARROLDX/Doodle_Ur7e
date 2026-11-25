# 图片库

- `external/`：外来图片；随仓库提供 `smiley.png`。导入时会再存一份带唯一编号的原图。
- `generated/`：GPT 生成原图、`review.png`、对话和确认记录。每次出图是独立目录。
- `processed/`：对应编号的 `processed.png`、`preview.svg`、`strokes.json`、`checks.json`；请求规划时还会有 `robot_plan/plan.json`。

生成图和用户导入图默认不提交 Git，文件仍保存在服务器/工作站本地。确认只针对本次图片 SHA256；后续对话、重新生成或文件被修改都不能复用确认。API 密钥只从环境变量读取，不写进图片记录。

使用说明见 [Baymax 入口](../docs/baymax.md)。只通过图片检查不代表机器人已可执行，仍必须通过实测配置、纸面标定、工作空间、实时状态及 MoveIt 完整无碰撞路径检查。
