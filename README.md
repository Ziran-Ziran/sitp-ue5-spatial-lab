# SITP UE5 Spatial Lab

**目标：UE5 仿真中的语义空间分析与路径规划。** 本项目与 [gamescaling-agent-lab](https://github.com/Ziran-Ziran/gamescaling-agent-lab) 独立开发、独立评测、独立发布，共享小型工程工具，不共享运行时或必需模型。

## 研究边界与当前能力

- **已有实现：**UE5 RGB-D 采集 C++ 源码、独立标定与输入检查、离线 VO/COLMAP/深度融合、ORB-SLAM3 顺序回放适配与评测；另有独立场景副本中的引擎 NavMesh/oracle 任务、局部监督语义目标搜索闭环，以及与搜索观测关联的静态位姿 RGB-D 回放。
- **已做的本地验收：**UE 5.8.2 原生模块编译、合成地面/墙体的导航与碰撞控制案例，以及本地静态场景的固定 oracle 回合。资产与原始结果不随仓库发布，其他设备和场景仍需独立验收。
- **尚待实现与验证：**视觉语义识别、观测估计的可通行地图与规划、语义场景关系；实时连续 SLAM、地图生命周期、渲染采集与导航的实时联动仍待验收。引擎名称、标签、位置和 NavMesh 仅用于显式 oracle 或监督条件。

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

可执行的本地 oracle 任务使用独立入口 `python scripts/navigation_experiment.py --help`，见[导航基线复现](docs/navigation-baseline.md)。它需要本机 UE 5.8、C++ 工具链和已登记的独立场景副本；不依赖 GameScaling、模型或真实客户端输入。

局部监督搜索入口为 `python scripts/semantic_search_experiment.py --help`。它在当前观测/记忆中选择语义目标，通过引擎路径实际移动；探索点、位姿和标签仍是明确的引擎监督，尚未使用视觉模型或观测估计的几何地图。

RGB-D 位姿回放入口为 `python scripts/render_replay_experiment.py --help`。它先验收颜色、深度、位姿和曝光控制，再按固定搜索帧导出独立视觉输入；这是离线重渲染，未把图像接入在线搜索策略。

## 可选的本地场景

已提供[两项目本地场景接入](docs/local-scenes.md)：共用源素材登记，各自维护独立 UE 实验副本；素材与本机绑定不上传 Git，也不构成训练或导航已验收。

## 工程入口

- [架构与模块边界](docs/architecture.md)、[开发与平台限制](docs/development.md)
- [研究问题与实验设计](docs/research-plan.md)、[阶段路线图](docs/roadmap.md)
- [输入、时间、模态和评测协议](docs/data-contract.md)
- [分拆来源与本地数据处理](docs/migration.md)、[共享工程工具维护](docs/shared-engineering.md)
- [贡献指南](CONTRIBUTING.md)、[发布范围](docs/repository-policy.md)、[授权说明](NOTICE.md)

原始混合项目保留为历史来源，本仓库不复制其私人日志、论文资料、资产、数据和模型；公开源码按固定提交导出。
