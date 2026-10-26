# SITP UE5 Spatial Lab

**目标：UE5 仿真中的语义空间分析与路径规划。** 本项目与 [gamescaling-agent-lab](https://github.com/Ziran-Ziran/gamescaling-agent-lab) 独立开发、独立评测、独立发布，共享小型工程工具，不共享运行时或必需模型。

## 研究边界与当前能力

- **已有实现：**UE5 RGB-D 采集 C++ 源码、独立标定与输入检查、离线 VO/COLMAP/深度融合、ORB-SLAM3 顺序回放适配与评测。
- **尚待实现与验证：**语义识别、语义场景图、可通行表示、路径规划与 UE5 内导航闭环；实时连续 SLAM 和地图生命周期仍需独立验收。
- **本轮完成：**源码分拆、来源清单、独立 CI、发布白名单、研究协议和配置检查；不代表新增算法、训练或游戏现场验收完成。

本项目使用 UE5 提供可控场景与独立真值，研究空间语义、可通行性和路径规划。规划可接规则控制器或引擎导航接口；不依赖 GameScaling。

## 离线开始

推荐 Python 3.12；先确认本机 Git、Python 路径和版本。只克隆本仓库即可运行以下检查，不需要兄弟仓库、GPU、游戏、模型或 UE。

```powershell
git clone https://github.com/Ziran-Ziran/sitp-ue5-spatial-lab.git
cd sitp-ue5-spatial-lab
python -m venv .venv
.venv\Scripts\python.exe -m pip install -r requirements/dev.txt
.venv\Scripts\python.exe -m unittest discover -s tests -v
.venv\Scripts\python.exe -m unittest discover -s ue5/tools -p "test_*.py" -v
.venv\Scripts\python.exe scripts/validate_project.py
.venv\Scripts\python.exe scripts/sync_engineering.py
.venv\Scripts\python.exe scripts/check_repository.py
git diff --check
```

Linux/macOS 将解释器路径换成 `.venv/bin/python`。部分链接权限或真实会话回归会明确跳过；UE 与 ORB C++ 编译需独立环境。

`experiments/*.json` 是**设计阶段的研究配置**，尚未绑定数据或训练/导航执行器；`validate_project.py` 只检查配置与数据边界，不启动实验。

## 可选的本地场景

已提供[两项目本地场景接入](docs/local-scenes.md)：共用源素材登记，各自维护独立 UE 实验副本；素材与本机绑定不上传 Git，也不构成训练或导航已验收。

## 工程入口

- [架构与模块边界](docs/architecture.md)、[开发与平台限制](docs/development.md)
- [研究问题与实验设计](docs/research-plan.md)、[阶段路线图](docs/roadmap.md)
- [输入、时间、模态和评测协议](docs/data-contract.md)
- [分拆来源与本地数据处理](docs/migration.md)、[共享工程工具维护](docs/shared-engineering.md)
- [贡献指南](CONTRIBUTING.md)、[发布范围](docs/repository-policy.md)、[授权说明](NOTICE.md)

原始混合项目保留为历史来源，本仓库不复制其私人日志、论文资料、资产、数据和模型；公开源码按固定提交导出。
