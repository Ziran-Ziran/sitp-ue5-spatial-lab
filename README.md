# SITP UE5 Spatial Lab

**目标：UE5 仿真中的语义空间分析与路径规划。** 本项目与 [gamescaling-agent-lab](https://github.com/Ziran-Ziran/gamescaling-agent-lab) 独立开发、独立评测、独立发布，共享小型工程工具，不共享运行时或必需模型。

## 研究边界与当前能力

- **已有实现：**UE5 RGB-D 采集 C++ 源码、独立标定与输入检查、离线 VO/COLMAP/深度融合、ORB-SLAM3 顺序回放适配与评测；另有独立场景副本中的引擎 NavMesh/oracle 任务、局部监督语义目标搜索闭环，以及与搜索观测关联的静态位姿 RGB-D 回放。
- **已做的本地验收：**UE 5.8.2 原生模块编译、合成地面/墙体的导航与碰撞控制案例，以及本地静态场景的固定 oracle 回合。资产与原始结果不随仓库发布，其他设备和场景仍需独立验收。
- **观测几何与执行：**校准深度生成局部地面支持/障碍/未知网格，经独立 A*、连续足迹检查和实际 CharacterMovement 执行；已实现停止采图重规划与固定红/蓝视觉标记目标搜索。眼高、相机外参与引擎位姿的重置/动作监督均明确声明，保留全部未知、危险、规划和动作失败。
- **尚待实现与验证：**家具类别视觉识别、语义场景关系与跨场景泛化；实时连续 SLAM、地图生命周期和连续采样导航仍待验收。颜色标记控制不代表一般语义能力，引擎名称、标签、位置和 NavMesh 仅用于显式 oracle 或监督条件。

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

深度几何离线探针入口为 `python ue5/tools/depth_grid_replay.py --help`，只读取隔离的 input 序列，不读取相邻的引擎评测目录。固定局部起终点的路径可用性与导航成功率分别报告。

`scripts/pitch_sweep_experiment.py` 为同一冻结源帧采集新的水平/45°/垂直向下静态视图；`ue5/tools/pitch_sweep_report.py` 比较单视图与三视图的局部路径。视角是明确的相对外参，不是假定历史相机曾实际转动。

`scripts/depth_path_experiment.py` 将冻结的深度局部路径经连续足迹检查后交给真实 CharacterMovement，保留全部规划/检查/执行失败。几何只读隔离深度，重置与动作反馈明确使用引擎位姿；该入口专门评测离线观察到实际运动的局部探针。

`scripts/online_depth_experiment.py` 进一步在本次实际运动后停下采 RGB-D，并比较初始路线与每至多一米重新观察/规划。它仍是引擎位姿监督下的几何局部任务，已验收受控观察反馈；没有视觉语义识别、估计定位或连续实时采样。

受控视觉类别入口 `scripts/visual_marker_experiment.py` 使用固定 HSV 红/蓝标记检测、光轴深度局部目标、深度路径和同一直接动作模块。类别来自 RGB，任务请求不含目标坐标；真值只在结束后独立评分。它面向可核验的颜色标记控制，家具识别与跨场景泛化仍未实现。

`scripts/city_marker_experiment.py` 将同一颜色目标链路放入独立城市覆盖场景，保留所有源位姿、颜色误选与几何/动作失败。当前两规划条件都为 0/96 到达，零接触；原始失败和构造诊断可复查，不能用中性控制场景成绩替代城市结果。

观测支撑面入口 `ue5/tools/support_plane_replay.py` 与 `depth_path_experiment.py --ground-model-pair` 比较同图水平/坡面几何。当前完整冻结路径实测为 8/48 与 10/48 到达、测量中均零接触；两项增量来自一个相同深度输入组，不能当独立样本或城市语义成功。直立胶囊净空、失败拟合与未知区域均保留，详见导航基线。

保存观测还可用 `scripts/marker_perception_diagnostics.py` 区分颜色、组件过滤与几何可见性代理；真值只留在评测侧。已知色标尺寸的离线消融损失了原有效关联，保留负结果，未接入在线策略。

## 可选的本地场景

已提供[两项目本地场景接入](docs/local-scenes.md)：共用源素材登记，各自维护独立 UE 实验副本；素材与本机绑定不上传 Git，也不构成训练或导航已验收。

## 工程入口

- [架构与模块边界](docs/architecture.md)、[开发与平台限制](docs/development.md)
- [研究问题与实验设计](docs/research-plan.md)、[阶段路线图](docs/roadmap.md)
- [输入、时间、模态和评测协议](docs/data-contract.md)
- [分拆来源与本地数据处理](docs/migration.md)、[共享工程工具维护](docs/shared-engineering.md)
- [贡献指南](CONTRIBUTING.md)、[发布范围](docs/repository-policy.md)、[授权说明](NOTICE.md)

原始混合项目保留为历史来源，本仓库不复制其私人日志、论文资料、资产、数据和模型；公开源码按固定提交导出。
