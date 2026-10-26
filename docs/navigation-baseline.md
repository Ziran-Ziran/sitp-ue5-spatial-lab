# 本地引擎 oracle 导航基线

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
