# 本地导航与语义搜索基线

入口 `scripts/navigation_experiment.py` 将独立静态场景推进为固定目标任务，提供冻结清单、目标选择、引擎路径、Game-world 移动和独立评分。引擎名称、位置和 NavMesh 都是明确的特权信息；当前没有视觉语义识别，也没有从 RGB-D 估计可通行地图。

## 前置条件

只在已登记的 `local/scenes/<scene-id>/workspace/UE5/` 独立副本工作。先确认设备、工具和进程，运行 `python scripts/local_scene.py inspect`；不得修改共享源素材或 catalog。此入口暂支持 Windows UE 5.8 与匹配 C++ 工具链。离线单元测试不需要 UE、资产或 GPU。

在当前机器给变量赋真实路径，避免复用其他设备的路径：

```powershell
$ueRoot = '<本机 UE_5.8 安装目录>'
$sceneProject = 'local/scenes/mondstadt-city-7-1-0/workspace/UE5/MondstadtCityDetailed.uproject'
$resourceLock = '<两对话约定的共享重任务锁文件>'
```

## 复现

每次使用新的输出目录和实验地图名，避免覆盖资产或历史结果：

```powershell
# 编译 -> 实测碰撞/导航 -> 保存独立实验地图 -> 冻结任务 -> 规划；尚未执行移动。
python scripts/navigation_experiment.py prepare --engine-root $ueRoot --project $sceneProject --lock $resourceLock --experiment-map /Game/SpatialLab/OracleCity_run1 --out output/nav-prepare-run1

# 读取原冻结清单和计划 -> 实际 Game-world 执行 -> 全回合独立评分。
python scripts/navigation_experiment.py run --engine-root $ueRoot --project $sceneProject --lock $resourceLock --tasks output/nav-prepare-run1/tasks.json --plans output/nav-prepare-run1/plans.json --out output/nav-execution-run1

# 内置地面/墙体：绕墙到达、无效出生点、不可达目标、直行碰墙四个控制案例。
python scripts/navigation_experiment.py synthetic --engine-root $ueRoot --project $sceneProject --lock $resourceLock --experiment-map /Game/SpatialLab/Synthetic_run1 --out output/nav-synthetic-run1

# 独立离线评分，不重跑 UE；输出必须为新文件。
python ue5/tools/navigation_report.py output/nav-prepare-run1/tasks.json output/nav-execution-run1/execution.json --out output/nav-execution-run1/score-recheck.json
```

有时间边界时，追加带时区的 `--start-before <ISO时间>` 与 `--finish-before <ISO时间>`。每个 UE/编译任务有超时、PID、日志、退出码和进程树清理；共享锁覆盖整个重任务生命周期。不会按年龄删除锁，也不会结束其他对话的进程。超时或锁忙后先查看该输出目录的 spec/status/log，不反复盲重跑。

## 如何解释结果

prepare 只证明编辑器查询与任务定义，不证明移动成功。run 检查完整地图标识、冻结清单哈希、`finished` 和固定回合，再生成 report。synthetic 的三个预期失败是控制案例通过，不是三个导航任务成功。

报告按策略保留全部六个任务、成功率、失败原因、碰撞事件及其可观测分母、执行路长、选择与执行 wall 时间、路径查询数。欧氏最近策略每回合一次路径查询；最短可达策略扫描所有有效接近点，因此不能称等算力比较。固定输入复现时同时比较终止状态、逐回合路长和失败记录，不能只比较成功率。

详细坐标、脚底参考点、预算、成功与碰撞定义见 [数据合同](data-contract.md)。这是单一静态场景、引擎标签监督/oracle 的端到端最小环境；固定回合不代表视觉识别、场景泛化、动态避障或实时 RGB-D 闭环已完成。

所有地图、素材、清单、实际轨迹、原始报告、构建文件和日志保留在本地忽略目录。公开内容只包含通用源码、合成测试和复现说明。

## 局部语义监督和记忆回放

在 `run` 或 `synthetic` 命令追加 `--record-observations`，额外生成真实角色位姿下的 `observations.jsonl` 和与本次执行关联的检查摘要。synthetic 同时增加可见椅子、初始被墙遮挡且绕行后可见的路灯控制物；物体使用引擎内置立方体，不依赖城市资产。四个移动控制案例与可见性控制分别验收。

先导出策略允许的目标类别，再单独回放；示例路径按实际输出替换：

```powershell
python ue5/tools/semantic_goal_replay.py export-goals output/nav-prepare-run1/tasks.json --out output/goals-only-run1.json
python ue5/tools/semantic_goal_replay.py replay --goals output/goals-only-run1.json --observations output/nav-execution-run1/observations.jsonl --ttl-s 5 --out output/semantic-replay-run1.json
```

回放比较当前帧与 5 秒 TTL 记忆的目标可用率，不读全场景实体表或未来帧。空帧、零帧失败和缺失回合单独报告。范围、眼高、三个包围盒射线样本及引擎标签均显式声明；完整字段与时钟见数据合同。这种保守采样会漏掉未被射线命中的物体。已有到达回合也可能没有目标类别观测，应保留并诊断，不能按终点真值选择参数。

路线仍由原有全局引擎 oracle 规划；离线选择不会改变角色运动。此结果不能表示观测驱动导航、视觉语义识别或记忆改善了导航成功率。`inspect_visibility_failures.py` 只在事后读取目标与几何作诊断，其输出不能作为策略输入。

## 观测驱动目标搜索

独立入口 `scripts/semantic_search_experiment.py` 使用同一传感器，令当前观测/记忆影响实际目标与移动；未发现目标时沿固定几何探索点搜索。需要场景启用 PythonScriptPlugin；同步执行项目内的 CPU 策略，不调用模型或外部服务。地图、构建和在线 Python 源码均在重任务锁内核验哈希。

```powershell
# 同步编译并创建仅依赖内置立方体的闭环控制案例，使用新地图/输出。
python scripts/semantic_search_experiment.py synthetic --engine-root $ueRoot --project $sceneProject --lock $resourceLock --experiment-map /Game/SpatialLab/SearchControls_run1 --out output/search-controls-run1

# 用既有冻结任务的起点生成固定几何网格，输出不含目标 catalog 的 missions。
python scripts/semantic_search_experiment.py prepare --engine-root $ueRoot --project $sceneProject --lock $resourceLock --tasks output/nav-prepare-run1/tasks.json --out output/search-prepare-run1

# 引擎只收到 missions；原标注在引擎结束后单独评分。
python scripts/semantic_search_experiment.py run --engine-root $ueRoot --project $sceneProject --lock $resourceLock --missions output/search-prepare-run1/missions.json --truth output/nav-prepare-run1/tasks.json --out output/search-execution-run1
```

合成控制包括：可见椅子到达、先探索绕墙再发现路灯并到达、首帧已在目标范围内的零移动到达、无效出生点和不允许探索时的预期失败。逐帧和时序记忆两种策略都执行。控制案例通过不等于城市任务全部成功。

报告保留所有回合的成功/失败、碰撞、路长、模拟与 wall 时间、移动请求数以及动作审计。实际城市回合可能因观测漏报、不可达目标或探索范围耗尽失败；两种记忆策略也可能结果相同。不得根据真值调参、删回合或把回放目标可用率写成导航收益。当前路径和探索点仍使用引擎几何，未实现视觉语义或观测估计的可通行地图。

## 与搜索关联的 RGB-D 位姿回放

回放先在内置几何上验收当前 DLL 的同步像素/深度，再从既有搜索输出冻结采集计划。采集在当前静态场景重渲染，不让历史源模拟时间随渲染 wall 时间前进。先依据当前场景的独立照明条件设置 `$captureExposure`，不要把别的设备或场景的曝光当成事实。

```powershell
# 首次使用新地图；若明确复用自己的既有控制地图，可加 --reuse-fixture。
python scripts/render_replay_experiment.py controls --engine-root $ueRoot --project $sceneProject --lock $resourceLock --experiment-map /Game/SpatialLab/RgbdControls_run1 --out output/render-controls-run1

# $captureExposure 为预先确定的 EV100 数值；源 job 必须对应这份搜索观测。
python ue5/tools/rendered_replay.py --observations output/search-execution-run1/observations.jsonl --source-job output/search-execution-run1/game.job.json --frames-per-episode 4 --exposure-ev100 $captureExposure --out output/render-plan-run1.json

python scripts/render_replay_experiment.py run --engine-root $ueRoot --project $sceneProject --lock $resourceLock --plan output/render-plan-run1.json --observations output/search-execution-run1/observations.jsonl --source-job output/search-execution-run1/game.job.json --controls output/render-controls-run1/capture --controls-job output/render-controls-run1/render.job.json --out output/render-execution-run1
```

输出包含 `input-report.json`、`capture/input/` 和独立的 `capture/evaluation/`。只把 input 下各 sequence 的像素/内参/源时间转交未来视觉模块；不要连同位姿或完整采集索引传入算法。当前输入检查拒绝真值字段、缺配对、串时间/位姿、换图及变化的源地图。控制验收也核对当前 DLL、脚本和实际输出目录；原生重编译后需对应控制证据。

相机 heading 由过去运动推导，初始为 0，不是假定恢复了实际游戏相机。图像尚未驱动在线策略；曝光、深阴影与有效深度比例也不能当作语义识别成绩。详细坐标、深度、曝光与时间限制见数据合同。所有图像、标注、位姿和日志仍保留本地。

## 冻结深度路径的实际运动

先完成 development.md 的 pitch sweep 采集，再冻结全部帧的单/三视角局部探针和连续足迹检查。prepare 为 CPU 工作，不启动 UE；另两步使用共享锁、进程树超时和新的地图/输出。

```powershell
python scripts/depth_path_experiment.py prepare --engine-root $ueRoot --project $sceneProject --lock $resourceLock --capture output/pitch-execution-run1/capture --capture-job output/pitch-execution-run1/render.job.json --out output/depth-prepare-run1
python scripts/depth_path_experiment.py synthetic --engine-root $ueRoot --project $sceneProject --lock $resourceLock --experiment-map /Game/SpatialLab/DepthControls_run1 --out output/depth-controls-run1
python scripts/depth_path_experiment.py run --engine-root $ueRoot --project $sceneProject --lock $resourceLock --capture output/pitch-execution-run1/capture --capture-job output/pitch-execution-run1/render.job.json --plan output/depth-prepare-run1/plan.json --controls output/depth-controls-run1 --out output/depth-execution-run1
```

当前控制含绕墙到达、故意直行撞墙、初始墙内出生、不生成代理的规划失败与零运动到达。所有控制预期均通过才允许城市执行。首次深墙出生的 overlap 查询曾漏报，实际贴地漂移被重置上限拦下；增加初始穿透 sweep 后控制通过，失败原始记录保留。

本机单场景一次冻结实验含 48 个源帧、两条件共 96 条记录。水平条件 48 条均因起点未知未执行；三视角有 12 条离线路径，连续 gate 拒绝其中 2 条连接段/路径足迹、2 条起点足迹，批准的 8 条均实际到达，0 个非地面碰撞事件，总实际路长 2660.94 cm。三视角全部帧成功为 8/48；其余 40 条规划或连续检查失败保留。相关源帧不构成 48 个独立搜索任务。

8 条运动的平均执行 wall 为 0.08925 秒，使用加速固定模拟步长，排除 UE 启动、重置、离线采集和规划；不能解释为实时导航延迟。路线由深度估计几何生成，世界坐标转换、重置与动作反馈使用显式引擎位姿。它不使用 NavMesh 路径，但尚未连接移动中采图和重规划，也不等同于椅子/路灯搜索成绩。全部原始证据仍在本地忽略目录。

## 停止观察后的在线重规划

独立入口使用本次 Game world 的实际相机图像，在每段移动后停止并重新观察；不复用原离线图像作在线输入。任务仍从完整源位姿冻结为前方三米探针，位姿/相对目标与动作反馈明确使用引擎监督。

```powershell
python scripts/online_depth_experiment.py prepare --engine-root $ueRoot --project $sceneProject --lock $resourceLock --capture output/pitch-execution-run1/capture --capture-job output/pitch-execution-run1/render.job.json --out output/online-prepare-run1
python scripts/online_depth_experiment.py synthetic --engine-root $ueRoot --project $sceneProject --lock $resourceLock --experiment-map /Game/SpatialLab/OnlineControls_run1 --out output/online-controls-run1
python scripts/online_depth_experiment.py run --engine-root $ueRoot --project $sceneProject --lock $resourceLock --capture output/pitch-execution-run1/capture --capture-job output/pitch-execution-run1/render.job.json --tasks output/online-prepare-run1/tasks.json --controls output/online-controls-run1 --out output/online-execution-run1
```

首次受控验证的 10 个回合通过全部 13 项检查，并从保存像素重算动作。平地两条件均到达：一次观察 1 次采图、332.62 cm；重观察 4 次采图、365.28 cm，均无碰撞。固定新墙在行走 50 cm 后启用，一次观察沿旧路线受阻并记录 20 次接触；重观察在第二次采图后因目标区域未知停下，0 次接触。静态墙、墙内出生和零运动控制也分别保留预期结果。这证明受控条件的观察反馈，并不证明城市成功率提高。

回合报告区分 whole-episode、capture、Python bridge wall 和模拟时间。重观察消耗更多图像、进程调用和路径折点，不是等观察预算比较；局部几何任务也不等同于椅子/路灯搜索。具体字段和时钟限制见 data-contract.md。运行仍须实际 GPU 渲染、共享锁和进程树清理；不得加 NullRHI。

本机城市停止采图的完整 96 回合已现场执行并独立重算像素/动作/轨迹：一次观察和一米重观察均到达 6/48、0 次非地面接触。实际总路长分别 2004.75/2191.71 cm，观察次数 48/66，平均有测量回合 wall 为 0.59072/0.78213 秒。本轮未显示重观察提高静态城市到达率；失败全部保留，未调用引擎路径。

该在线结果与前述冻结路径的 8/48 是不同条件。对额外两次连续路径拒绝的同像素交叉诊断发现：世界目标换算产生约 −4.22×10⁻¹⁵ m 的 right 值，网格 floor 索引落入零的相邻单元；旧/新深度图均复现相同差异。此诊断没有改写旧原始记录，也不把反事实路线算作现场成功；独立数值协议的后续现场对照见下文。

## 可核验的视觉标记目标

`scripts/visual_marker_experiment.py` 在独立内置几何场景运行固定 HSV 红/蓝标记任务。类别来自 RGB，目标相对位置来自光轴深度；世界坐标只用于重置与执行，真值仅在运行后评分。复现先以当前原生源码完成 online_depth_experiment.py synthetic 共同分支控制，再按 development.md 的视觉入口创建新地图/输出。

一次本机真实运行的 12 条固定任务完整结束，独立像素/动作/真值审计后成功 8/12，11 个有测量回合共 0 个非地面接触事件，总实际路长 3025.20 cm。正面与背向红/蓝搜索均有实际到达；首帧近距离目标经新像素确认后零移动结束。另保留一个连续足迹拒绝、两次四朝向搜索未发现目标、一次墙内出生受阻。

连续足迹失败发生在一个蓝标记任务：目标已从 RGB 检出且深度定位有效，格心 A* 找到路线，但一段保守方形足迹与未知格相交，因此没有执行。它不是类别缺失，也不能按真实空地把未知格补为可走。其他缺失/遮挡任务的预期失败、控制检查通过和实际到达数量分别解释，不删除失败分母。

共 60 次三视角停止观察，平均有测量回合 wall 为 2.18968 秒，其中累计采图 3.98601 秒、Python bridge 16.73021 秒；wall 排除 UE 启动与重置，固定步长模拟仍为加速运行。虚拟相机扫描没有机械转动时间模型。这一结果限于固定颜色标记和受控合成场景；家具语义、城市场景泛化与实时性能仍未验收。源图、轨迹、真值与失败产物只保留本地。

## 同一足迹约束下的规划消融

格心圆距离余量与后续连续方形足迹并不等价。独立合成最小复现是在平地加入一个未知格；旧 A* 路径经过它的方形扩张角而被 gate 拒绝，按相同方形约束生成的格心支持可绕开该未知格。`continuous_grid_planner.py` 对齐这两者，并提供通过同一 gate 的有界直连简化；没有放宽未知、胶囊、安全余量或任务预算。

同一原生 DLL、同一场景生成器的三个现场变体均执行全部十二项任务，并从保存像素/动作/独立真值重新核验：

| 变体 | 实际成功 | 接触 | 实际总路长 cm | 三视角观察次数 | 平均 episode wall 秒 |
|---|---:|---:|---:|---:|---:|
| legacy_centers | 8/12 | 0 | 3025.20 | 60 | 2.95866 |
| continuous_square | 8/12 | 0 | 3219.94 | 62 | 2.83396 |
| continuous_square_shortcuts | 8/12 | 0 | 3634.68 | 68 | 2.46146 |

三个条件都有十一项实际运动测量；墙内出生没有测量，不计作零接触样本。原蓝色任务在三个条件分别因连续路径足迹、后续局部目标余量、十二次观察预算结束。新规划器允许安全地继续探索，但本次没有提高成功率，额外移动与观察成本完整保留。wall 受顺序运行、缓存与进程开销影响，单次结果不能证明计算性能提高。

同图 CPU 重放中，城市三视角三种变体均为 8/48 连续路线批准、水平视图均为 0/48；六十次原标记观测中的可执行几何路线由 32 次变为 33 次。它们是相关历史观测的离线可用性，不能算反事实导航成功或独立试验。真实固定任务对照的重复入口和统一审计见 development.md。

## 查询边界数值协议的独立现场对照

原始坐标不动、连续 gate 不变，只显式选择查询格索引协议。旧协议在同图 114 次历史观测中完全重现原动作；新协议在其中 80 次改变至少一个规范查询索引，连续路线由 30 次变为 34 次，这只是离线动作可用性。

同一新 DLL、原城市地图和全部原位姿/世界目标下，两种协议各实际执行 96 回合并独立重算。每个协议的合成控制均通过全部十三项；同构建的视觉原分支回归保留 8/12、0 接触和全部原失败。

| 查询协议 | 模式 | 实际到达 | 接触 | 实际总路长 cm | 三视角观察 | 平均 episode wall 秒 |
|---|---|---:|---:|---:|---:|---:|
| legacy_floor | once | 6/48 | 0 | 2004.75 | 48 | 0.60092 |
| legacy_floor | receding | 6/48 | 0 | 2191.71 | 66 | 0.81608 |
| boundary_snap_1e-9_cell | once | 8/48 | 0 | 2660.94 | 48 | 0.59291 |
| boundary_snap_1e-9_cell | receding | 6/48 | 0 | 2767.12 | 72 | 0.91371 |

所有 192 回合都有实际测量，失败和零移动保留。新 once 在两条相关位姿上修复了初始索引引发的路线拒绝；新 receding 初始能走，但之后仍被新观测的连续足迹 gate 拒绝，未算成功。部分原目标格失败也只是换成更晚的连续检查失败。结果支持数值稳定性修复，不证明重规划更优、不同数据独立、语义能力或跨场景收益；单次 wall 差异也不作为性能结论。

## 城市几何中的受控像素目标

在新的城市副本添加六个红/蓝色标，构造的八次候选查询含一次胶囊阻挡、一次高度拒绝，全部保留；原地图字节未变。首次脚本导入失败与随后 commandlet 未初始化导致 24 次地面查询未命中、零色标的构造产物也保留，没有启动导航。已知原点的前后对照确认初始化问题，复用现有有界原生初始化后再构造，不改变地面、摆放环、颜色或任务预算。

全部 48 个原位姿各配两种颜色类别，每个规划条件 96 项任务，在同一覆盖地图和 DLL 下完整执行并独立审计：

| 规划器 | 到达 | 接触 | 实际总路长 cm | 三视角观察 | 平均 episode wall 秒 |
|---|---:|---:|---:|---:|---:|
| legacy_centers | 0/96 | 0 | 379.48 | 210 | 1.01721 |
| continuous_square_shortcuts | 0/96 | 0 | 381.92 | 210 | 1.12998 |

全部 192 项均有实际测量，零移动、几何拒绝和未观察到目标保留。两组各有 24 次四朝向扫描后仍未观察到所需类别；其余主要在起点/目标余量、负空间或连续足迹处拒绝。两组各有 76 次目标选择，按事后独立 50 cm 关联，42 次对应真色标，34 次未关联；各 40 个回合至少一次选择了真色标，仍未到达。相邻源位姿相关，不能当作独立泛化样本。

这证明同一像素类别→深度目标→规划→动作/失败反馈链路已在城市几何上实际评测，也显示当前基线未完成这些城市目标任务。平面地面先验、观测覆盖与城市颜色误检均有可复查失败；不得把控制场景的 8/12 或“观察到色标”替代这里的 0/96。构造使用引擎信息，视觉动作请求仅接收像素/类别/扫描状态/规划器，色标位置只在结束后评分；没有运行期 NavMesh 路径请求。


## 观测支撑面：冻结路径的物理对照

固定模型 `horizontal_v1` 和 `observed_support_plane_v3` 使用全部原始 48 帧、同一个 `continuous_square_shortcuts`、局部三米目标及原身体/连续足迹/动作预算。坡面只从隔离深度拟合近脚底观测支持；失败拟合保持未知，不读场景几何、引擎标签或目标真值。具体有界假设与直立胶囊净空修正见 data-contract.md。两种早期 CPU 候选的低顶反例、修复前失败测试和旧报告均保留，没有把候选离线批准写成验收。

同一当前原生 DLL 的五项 supplied-route 控制通过全部八项检查后，新的 schema 2 完整执行 96 行，并独立重建像素计划、原始生产者、控制快照、地图和实际轨迹：

| 地面模型 | 固定任务到达 | 实际启动测量 | 非地面接触事件 | 实测总路长/cm | 平均执行 wall/s |
|---|---:|---:|---:|---:|---:|
| horizontal_v1 | 8/48 | 8 | 0 | 2363.84 | 0.03634 |
| observed_support_plane_v3 | 10/48 | 10 | 0 | 2954.90 | 0.03321 |

配对为八项均到达、两项仅坡面到达、三十八项均失败，无退步。原四十八帧实际只有十八个不同三视角深度输入组；新增两项共享完全相同深度与 5.067876° 拟合，属于一个相关输入组，不能当作两次独立收益或显著性证据。失败仍全部进入分母：坡面有十八项起点未知、十二项目标障碍、六项起点余量失败、两项目标负空间；拟合本身有三十支持、八次 refinement 拒绝、两次周围支持不足、八次无有界共识。未执行项的碰撞/路长为 null，不当作零碰撞运动。

Game 进程本次 13.469 秒完成、退出码零、进程树清理；表中 wall 仅含实际运动，排除 UE 启动、重置和 CPU 几何计算，不能说明实时性能。实际路长为三维轨迹累计，探针之间的总长不用于比较路线效率。零接触只是本次有测量运动的记录，不是普遍物理安全保证。

本分支保持世界路径高度为原起点，不外推拟合坡高，原垂直/终点限制仍可拒绝坡路。它没有改变在线策略、重采样或颜色检测，该次转移未包含独立坡道/低顶套件；后续独立合成物理控制见下文，城市颜色目标仍为 0/96。同一旧颜色观测的模型回放未显示路线批准增益。CPU replay 与实际冻结转移的复现/统一审计入口见 development.md。


## 同图感知负结果与评测侧可见性代理

已有中性控制的全部 60 次保存观测先通过十四项投影/颜色/隐藏/遮挡/无效出生检查：41 次有 requested-category 深度支持，均有原检测/选择支持和正确点关联。分析只使用原观测，不新增 UE 回合或成功记录。城市两规划条件各 210 次保存观测均重新核验其原执行、不可变历史 worker、控制、地图、像素与动作后诊断：

| 原条件 | 代理支持≥9 像素的实体/观测对 | HSV 支持≥9 像素 | 原检测接受且选中支持 | 原选择总数/点关联/非关联 | 尺寸候选选择总数/点关联/非关联 |
|---|---:|---:|---:|---:|---:|
| legacy_centers | 52 | 50 | 42 | 76/42/34 | 50/28/22 |
| continuous_square_shortcuts | 52 | 50 | 42 | 76/42/34 | 48/28/20 |

每组原始四十八位姿×两类别分母继续为 96；210 次观测是相关轨迹中的观测，不是独立实验。这里的“代理支持”是已知盒体 front-depth 与保存深度的一致性，不是精确实例 mask 或可见目标 recall。约 97% 的 requested proxy 像素已有相应 HSV 颜色；仍有十个 proxy-eligible 对未被原检测接受。重叠拒绝原因包括八次高度先验、四次组件像素不足，不能相加当十次互斥失败；本组未出现处理预算遗漏，独立解析用例覆盖了此情形。

尺寸上界规则在回放前冻结为已知盒体尺寸及 2 mm 深度量化余量，没有针对城市真值放宽。它在中性保存观测中保留全部 41 次选择，但在每组城市观测额外排除了十四次有代理支持的原检测，点关联从 42 降到 28；非关联数量也下降，选择质量未显示实质改善。因此仅保留为明确的离线负结果，不新增运行协议或推广成识别能力。原城市两组导航仍是 0/96；这些反事实选择不替代原动作、到达或失败。

## 固定目标规划视距的现场负结果

原策略规划最远三米目标、执行至多一米前缀。对照把规划目标改成已有的一米执行视距，保持原检测、地面、legacy planner、连续足迹和全部预算不变；不尝试多个候选距离。独立解析用例同时覆盖三米未知而一米可走，以及一米目标落在墙上、三米路线原可绕行的退步。全部 480 次原观测的 CPU 重放中，中性动作批准 32→33、两组城市各 4→20；这些仅是旧像素上的动作，不是到达。

新构建先通过共同几何控制十三项检查，再各通过颜色控制十项；两组各完整十二任务到达 8/12、十一项实际测量、零接触。三米/一米分别走 3025.20/3790.05 cm、采图 60/68 次，四项失败相同，额外动作没有改善中性成功率。

同一新城市覆盖地图保留原六个色标完全相同的位置和原基础地图字节，两条件执行全部四十八原位姿×两类别：

| 规划视距 | 实际到达 | 实际测量 | 非地面接触 | 实测总路长/cm | 三视角观察 | 平均 episode wall/s |
|---|---:|---:|---:|---:|---:|---:|
| fixed_three_meters | 0/96 | 96 | 0 | 379.48 | 210 | 0.91310 |
| execution_prefix_one_meter | 0/96 | 96 | 0 | 2880.53 | 236 | 1.21939 |

三米结果重现原基线；一米增加移动和观察后仍没有完成城市任务。两组各二十四项四朝向搜索未发现目标，另有起点余量/负空间、目标余量和连续足迹拒绝；未因有移动或检测而算成功。原生路径请求均为零，CPU 比较器核验当前同一 DLL、地图、匹配控制及移除视距字段后相同的规范任务 manifest，并逐步重算全部像素、动作、轨迹和独立真值评分。原始全部失败保留，单次顺序运行的 wall 不能说明实时性能或统计显著差异。复现入口见 development.md 的目标规划视距对照。

三米/一米分别有76/102次目标选择，事后50cm点关联为42/58次、非关联为34/44次；两组仍各40个回合至少选择过真色标。增多的关联观测属于同一相关轨迹，不是更多独立检测成功或可见目标 recall，误选也随移动增加。

## 独立坡面与低顶的物理控制

在自有基础几何地图预先固定十种用例：平地，正负5°/10°/15°坡面，170cm低顶、220cm高顶，以及10°坡面上的176.5cm平行低顶。两模型只接收新三俯角深度，使用同一一米查询、legacy planner、身体/未知/连续 gate；世界路线高度仍固定为起点，原20cm终点高度容差不变。低/高/斜顶另各有一条独立 supplied reference，不能与感知规划混记。

| 条件 | 原执行合同成功 | 规划批准/实际测量 | 非地面接触 | 实测总路长/cm | 平均运动 wall/s |
|---|---:|---:|---:|---:|---:|
| horizontal_v1 | 2/10 | 2 | 0 | 249.59 | 0.02488 |
| observed_support_plane_v3 | 5/10 | 8 | 0 | 1010.19 | 0.01877 |
| supplied_reference（单列） | 1/3 | 3 | 42 | 201.46 | 0.07766 |

平地、高顶两模型均成功；正负5°和负10°的支撑面路线成功。正10°、正负15°均获批准并实际运动，但因原终点高度合同失败，实际终点Z分别为20.594、30.580、−23.882cm；不能把八次批准写成八次成功，也不按已看到的结果放宽20cm阈值。水平模型拒绝全部六个坡面。净空两负例两模型均拒绝，未测量行保留null；三条直接参考中，高顶到达、低顶和斜顶各实际阻挡并记录21次接触。这验证了本次净空负例确有物理阻挡；全部十三项 instrumentation 检查通过，没有批准后接触的模型路线。

当前构建先重新通过五项直接运动控制及其八项检查，故意穿墙参考产生的二十次接触另记。新套件采集进程15.400秒、Game进程11.248秒，均退出零、清理完整进程树并释放本任务的锁；表中运动wall排除采图、模型计算、启动和重置，不是实时性能。十个用例是合成控制而非城市或独立分布；路线高度保持起点的限制仍未解决，支撑面未接入在线颜色任务。

首次采集完成后发现标定字典接口不匹配，在进入Game前拒绝；原产物保留，修复有完整标准标定/像素回归。审查随后补齐斜顶参考验收、危险批准清单及原生产者导入闭包/模型/像素/控制快照和唯一60Hz参数守卫，用新地图输出重新完成全部二十三行，没有改物理参数或原记录。CPU audit 重建全部像素计划并重算实际轨迹、接触、失败与参考，复现命令见 development.md。
