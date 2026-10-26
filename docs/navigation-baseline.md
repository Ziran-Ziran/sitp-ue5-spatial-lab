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
