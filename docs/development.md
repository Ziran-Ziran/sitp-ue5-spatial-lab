# 开发与复现

## 离线环境

推荐 Python 3.12；依赖以项目现有环境为起点，并更新已知有安全修复的版本，不表示任意平台组合已验证。先在当前机器确认 Git、Python 路径，再创建虚拟环境并安装 `requirements/dev.txt`。本仓库只安装空间感知依赖，不安装 GameScaling 或控制器依赖。

```powershell
python -m unittest discover -s tests -v
python -m unittest discover -s ue5/tools -p "test_*.py" -v
python scripts/check_repository.py
git diff --check
```

这些命令只做离线验证。SLAM 中有一项可选的真实会话回归，未提供本地数据时跳过。CI 不替代官方模型或真实数据集成测试。CI 运行 Windows/Linux 几何回归和独立工程检查，不编译 UE、不运行游戏或模型。

## UE5 采集开发

项目描述文件当前关联 UE 5.8。开发者需独立安装有合法授权的对应 UE 版本及 C++ 工具链。公开版只包含项目描述、Source 和三个 Default 配置，**不包含 Content 中的地图和蓝图**；原配置仍引用 First Person 模板资源，因此直接双击不能保证进入可用场景。

可用官方 First Person 模板在本地创建或补齐对应资源，或自行建立带有 Pawn 与 CameraComponent 的测试关卡，并在 Project Settings 中设置启动地图与 GameMode。保留 `GSVSLAM` 模块名称，生成工程文件后编译。自行创建场景时优先在本地覆盖配置，提交前审查与他人场景的兼容性。

采集子系统可向带相机的 Pawn 挂载采集组件，现有入口包含 F9/F10 和 `VisionRec.Start` / `VisionRec.Stop`。默认数据写入根目录 `output/ue5-datasets`，不纳入 Git。记录前核对分辨率、内参、深度编码、渲染设置与时钟；保留完整元数据以便重建。

尚未在全新克隆和空资产环境验证 UE 编译/采集。资产初始化自动化是明确的后续任务，不把缺失资源视为已解决。

## ORB-SLAM3 无真值顺序回放

适配器位于 `ue5/tools/orbslam3/`，链接开发者已有的 ORB-SLAM3、OpenCV、Eigen 和 Pangolin；不会安装依赖或替换上游库。先核实本机与 WSL 的身份和实际路径，再执行；上游库/源码/词袋不随本仓库分发，授权见 NOTICE。

```powershell
python ue5/tools/orbslam3_input.py <capture-session> <new-sensor-input>
```

预处理只从元数据提取固定独立标定、尺寸、深度单位和名义频率，输入目录只含图像对、`sensor.json`、`settings.yaml` 和 `frames.tsv`。不复制或打开真值位姿、地标，也不转交整份元数据。BGRA PNG 显式转换为 BGR，uint16 光轴毫米深度中的饱和 65535 转为无效 0，不翻转图像。缺配对或变化标定会报错。默认保留至少 40 GiB 空闲，单次输入复制上限 10 GiB。

在 Linux/WSL 中，`ORB_ROOT` 必须指向本机真实存在的构建；示例中的变量先按本机情况赋值。依赖路径需与既有库一致，避免误用另一套 Eigen。

```bash
cmake -S ue5/tools/orbslam3 -B "$BUILD_DIR" -DORB_ROOT="$ORB_ROOT" \
  -DEigen3_DIR="$EIGEN_CONFIG" -DPangolin_DIR="$PANGOLIN_CONFIG" \
  -DOpenCV_DIR="$OPENCV_CONFIG" -DCMAKE_BUILD_TYPE=Release
cmake --build "$BUILD_DIR" -j2
"$BUILD_DIR/gsv_rgbd" "$ORB_ROOT/Vocabulary/ORBvoc.txt" \
  "$SENSOR_INPUT/settings.yaml" "$SENSOR_INPUT" "$NEW_OUTPUT" 10
```

最后参数是回放节流 Hz，0 表示尽快按顺序处理；每次只读当前帧。旧采集的时钟是 `frame_id / target_hz`，明确属于名义时间，不是精确采样时间或端到端延迟。Viewer 与闭环均关闭，输入深度因子固定为 1000。默认虚拟基线 0.1 m、近点阈值 4 m 是初始参数；近点阈值还控制新 RGB-D 地图点创建，需要按传感器工作距离独立核查，不能根据真值误差选择。

输出 `tracking.jsonl` 包含即时状态和 `Twc`（CV 相机到原生地图，米）；仅 OK 且有限的位姿有效，失效为 null。`map_id` 来自公开的已跟踪地图点接口；每张图的原点独立。`sparse_snapshot.jsonl`、`keyframes_snapshot.jsonl` 保留图编号；单图时另存上游异步轨迹。当前本地上游 Shutdown 不等待线程，导出只称为加锁异步快照，不声称静止的最终优化图。地图导出不代表地图重载定位已验收。

原生产物完整落盘后，可单独执行：

```powershell
python ue5/tools/orbslam3_report.py <new-output> --session <capture-session>
python ue5/tools/summarize.py
```

不传 `--session` 只做原生状态统计，不读真值。传入后才读取真值作事后评分；原生产物先保存 SHA-256 并核对评分前后不变。按连续有效段分别报告覆盖率、米制刚体 ATE 与仅供评测的 Sim(3)，多图不拼接为全程轨迹。该适配器当前只提供文件回放，UE 实时通道另行验收。

## ORB 跟踪内部诊断

`orbslam3_diagnostics.py` 在新目录生成本地上游源码副本，保留上游版权头，并复制、哈希已有构建的目标文件。补丁必须精确匹配唯一锚点；源码不兼容时先失败，不修改原安装。诊断构建只重编 Tracking/LocalMapping 两个上游编译单元，加上本项目的日志组件；不修改类布局、匹配阈值或跟踪决策。须使用与原对象一致的编译器、依赖和构建参数，不把复制对象视为通用二进制。

```bash
python3 ue5/tools/orbslam3_diagnostics.py prepare "$ORB_ROOT" "$DIAGNOSTIC_SOURCES"
cmake -S ue5/tools/orbslam3 -B "$DIAGNOSTIC_BUILD" \
  -DORB_ROOT="$ORB_ROOT" -DEigen3_DIR="$EIGEN_CONFIG" \
  -DPangolin_DIR="$PANGOLIN_CONFIG" -DOpenCV_DIR="$OPENCV_CONFIG" \
  -DGSV_BUILD_DIAGNOSTICS=ON -DGSV_DIAGNOSTIC_SOURCES="$DIAGNOSTIC_SOURCES" \
  -DCMAKE_BUILD_TYPE=Release
cmake --build "$DIAGNOSTIC_BUILD" --target gsv_rgbd_diagnostics -j1
"$DIAGNOSTIC_BUILD/gsv_rgbd_diagnostics" "$ORB_ROOT/Vocabulary/ORBvoc.txt" \
  "$SENSOR_INPUT/settings.yaml" "$SENSOR_INPUT" "$NEW_OUTPUT" 10
python3 ue5/tools/orbslam3_diagnostics.py report "$NEW_OUTPUT"
```

`diagnostics.jsonl` 记录特征/有效深度、投影与参考关键帧匹配、优化后地图内点、关键帧创建/入队/处理、局部 BA、重定位候选、重置与新图事件。前台使用源帧号；后台线程的帧号为 null，按关键帧 ID 和时间戳关联，避免误记到当前帧。`diagnostic_report.json` 解释失效分支与入队等待，未观测的分支明确保留未知，不把特征总数当地图匹配内点数。

诊断会改变运行时开销和线程调度，不能拿它的速度作实时验收。日志在 Shutdown 请求后关闭，之后不再写入；本地上游不等待后台线程，runner 关闭本地输出流后结束整个独立进程，避免尚存线程与析构的同步对象竞争。地图仍是加锁异步快照，未声称最终静止或重载定位。

本地 ORB 默认可能屏蔽文字重置消息：无内部事件时报告重置次数未知，另存文字日志出现次数；只有覆盖重置入口的诊断 schema 2 才报告精确计数。旧报告里的零条重置消息不能直接当成零次重置。

## 新数据输入检查

```powershell
python ue5/tools/inspect_slam_input.py <image-directory-or-video-or-map> --out <new-report.json>
# 默认最多解码300帧；完整检查用 --max-frames 0
```

此工具只读源数据，不提取视频、不建图、不改时间、不读取 poses.jsonl/landmarks；拒绝输入及父目录的符号链接/junction。报告包括帧编号、配对、坏帧、尺寸变化、重复画面、独立标定、时钟来源和深度有效比例。`offline_ready` 只表示已完整检查的输入合同通过，不表示 SLAM 成功；局部检查为 false。退出0表示合同通过，退出2表示未就绪，原因在 errors/warnings。显式输出必须为新文件。

- RGB 图像序列或视频建议单目路线，但本项目当前 runner 仍只实现 RGB-D。视频 FPS 只支持名义回放，不证明采样时钟或米制尺度。
- RGB-D 当前检查合同为 uint8 三通道 BGR 解码、同名 uint16 光轴毫米深度，0/65535无效。BGRA/灰度明确提示需转换；UE 原始 BGRA 可先运行 `orbslam3_input.py` 准备独立输入，再检查。其他深度单位/含义需显式适配，不能猜测换算。
- 单独的 PLY/OBJ/GLB 等地图只做文件类型/大小识别，不解析几何，不判为可跟踪序列；定位还需要连续图像、相机标定和地图格式适配。

图像文件名需带数字帧号，建议 `frames/frame_000000.png`，深度同名放 `depth/`。标定从独立 `sensor.json` 或显式 `--calibration` 读取；既有 UE 元数据只提取白名单 calibration 字段，不把 start_pose 转交算法。标定字段为 fx/fy/cx/cy/w/h，工具能检查数值与尺寸，不能代替真实相机标定。

旧数据可声明 `target_hz` 和 `time_source: nominal_frame_id_over_target_hz`；这只支持名义回放。新采集若有独立的 `timestamps.jsonl`，每行使用 `{"frame_id":0,"t_sensor_s":100.0}`（秒），并在 sensor.json 声明 `time_source: sensor_capture_monotonic`。只有真实采样时记录的独立时钟才应这样声明；工具检查声明与单调性，不会独立证明物理时钟同步。不能从真值文件提取时间，也不能把容器FPS或文件保存时间冒充采样时间。


## 工程配置与共享工具

`python scripts/validate_project.py` 校验设计配置、评测划分及特权信息声明，不读取真实数据，不检查实际模型行为。
`python scripts/sync_engineering.py` 检查本仓库的共享文件与锁定哈希一致；更新流程见 shared-engineering.md。
`python scripts/check_repository.py` 只检查 Git 暂存区实际内容。全新仓库应在暂存源码后运行；正常克隆已存在完整 index。

这些检查都不启动训练、GPU、云资源、UE、游戏输入或模型下载。实验输出留在忽略目录 output/，不得把旧工程的绝对路径当本机事实。

本地蒙德城区素材接入与独立 UE 副本见 [场景接入说明](local-scenes.md)。

## 独立 oracle 导航实验

见[导航基线复现](navigation-baseline.md)。`scripts/navigation_experiment.py` 提供 prepare/run/synthetic；它将本仓库的 SpatialLab 源码暂存到独立场景副本，使用本机 UE 5.8 C++ 工具链编译。不会把 GSVSLAM 的资产缺失问题隐式当成已解决，也不依赖该采集模块或另一仓库。

UE/build 子进程必须使用显式共享锁和超时，Windows Job Object 覆盖整个进程树；锁忙最多等待 60 秒，随后保留状态并退出，不夺锁。完成截止在取得锁后重算，并保留 20 秒清理预算。CPU 离线模块无需该锁。普通 CI 不运行 UE 集成验收；本机合成 UE 和场景结果需独立报告。

## 搜索位姿的离线 RGB-D 回放

`scripts/render_replay_experiment.py` 需要实际渲染、PythonScriptPlugin 和对应 GPU，使用 `-AllowCommandletRendering -RenderOffscreen`，不能加 NullRHI。首次编译材质可能较慢；查看当前有界任务的 shader/log/PID，不重复启动。局部环境曾在未使用的 SM5 着色器路径出现 Niagara 编译错误，而 D3D12 控制渲染完成；退出码与像素验收必须同时检查，不能只忽略日志。

渲染前先通过内置彩色几何、光轴深度、位姿平移/重复、空视野和曝光控制。若原工程使用旧亮度单位，采集进程明确切为 EV100；不更改磁盘配置。场景已有曝光值是独立依据，光照退化、阴影或裁剪仍需报告，不能为了导航分数调整。复现命令与算法/评测目录边界见 navigation-baseline.md 和 data-contract.md。

## 深度几何离线探针

```powershell
python ue5/tools/depth_grid_replay.py --input output/render-execution-run1/capture/input --out output/depth-grid-run1
```

输入只传隔离的 input 目录，输出目录必须不存在。该 CPU 工具保存每帧网格、余量、路径或失败与源文件哈希，不启动 UE/GPU、不读引擎评测目录。默认眼高、相机姿态与格子假设见 data-contract.md；不能把未知脚边区域补成空地，或把局部路径探针当作实际目标导航。

## 固定向下视角的近场覆盖

```powershell
python scripts/pitch_sweep_experiment.py controls --engine-root $ueRoot --project $sceneProject --lock $resourceLock --map-prefix /Game/SpatialLab/PitchControls_run1 --out output/pitch-controls-run1
python scripts/pitch_sweep_experiment.py run --engine-root $ueRoot --project $sceneProject --lock $resourceLock --base-plan output/render-plan-run1.json --observations output/search-execution-run1/observations.jsonl --source-job output/search-execution-run1/game.job.json --controls output/pitch-controls-run1/capture --controls-job output/pitch-controls-run1/render.job.json --out output/pitch-execution-run1
python ue5/tools/pitch_sweep_report.py --input output/pitch-execution-run1/capture/input --out output/pitch-grid-run1
```

控制地图/输出均须新建；来源计划和搜索任务需满足原始状态哈希快照协议。启动器核对控制脚本及全部导入源码、DLL 和实际输出目录。重任务仍使用共享锁与进程树超时；几何报告只用 CPU。新分支不改写旧水平图像或原始搜索结果，也不把离线探针写成已执行路线。

## 深度路径的物理转移

`scripts/depth_path_experiment.py` 的 prepare/synthetic/run 复现命令见 navigation-baseline.md。prepare 重算所有帧并保存确定性的冻结计划；run 再核对源像素、原采集状态快照、地图和几何源码。几何输出之后才由独立层读取源位姿做重置/动作坐标转换。当前 DLL 的内置无 NavMesh 控制必须通过；物理执行用 NullRHI，无需重新渲染图像。若源码或源输入改变，另建计划和输出，不改写旧记录。

直接动作执行器会在墙体阻挡、无进展、轨迹偏离、重置异常或预算耗尽时失败。先检查 execution.json 的实际轨迹/反馈及 game.job.log/status，再针对原因修复；不要按城市真值放宽足迹检查或只执行已知成功帧。冻结探针的转移结果与语义搜索、在线 RGB-D 闭环分别报告。

## 在线停止观察与重规划

`scripts/online_depth_experiment.py` 提供 prepare/synthetic/run，见 navigation-baseline.md。prepare 只用 CPU 冻结源位姿任务；synthetic 编译并创建独立无 NavMesh 控制地图；run 实际 GPU 采图与直接动作。数值 worker 使用启动器当前 Python，需已有 numpy/opencv；UE 内嵌 Python 只负责有界子进程。每次观察单独保存严格 input、request/response 和原生相机/动作/轨迹证据，审计不读取全场景几何。

新任务输出和地图必须不存在。当前原生 DLL 与策略源码控制必须通过，子进程也受共享锁下的父进程树约束。观察期间不修改冻结源码或输入；任何失败先查本次日志/状态/像素/动作，不复用旧响应或补写原始证据。该模式是停走观察，不是连续实时传感器或完整语义目标导航。

## 受控 RGB 标记目标

`visual_marker.py`、`visual_marker_depth_policy.py` 只用 BGR 图像/光轴深度与已知局部外参。固定类别规则、缺失处理、四朝向搜索与距离/预算见 data-contract.md。算法请求不包含目标世界坐标；`visual_marker_report.py` 在像素/动作审计后才读取单独的 fixture truth。

先以当前源码运行 online_depth_experiment.py synthetic，完成编译并验收共同采图/动作分支，再使用新地图和输出：

```powershell
python scripts/visual_marker_experiment.py --engine-root $ueRoot --project $sceneProject --lock $resourceLock --geometry-controls output/online-controls-run1 --experiment-map /Game/SpatialLab/MarkerControls_run1 --out output/marker-controls-run1
```

入口创建自己的中性地面、unlit 色标与遮挡墙，保存独立 truth 后执行固定全部任务。truth 的路径不会传入 UE 游戏或数值 worker；其哈希由外层任务守卫保护，评分器只在结束后读取。日志、图像、地图、轨迹和 truth 均保留忽略目录。合成 CPU 用例、真实控制场景与城市研究结果须分别报告。

同一当前构建可比较预先固定的三种规划器，各使用完整十二项任务和新地图/输出，参数与预算保持不变：

```powershell
python scripts/visual_marker_experiment.py --engine-root $ueRoot --project $sceneProject --lock $resourceLock --geometry-controls output/online-controls-run1 --planner-variant legacy_centers --experiment-map /Game/SpatialLab/MarkerLegacy_run1 --out output/marker-legacy-run1
python scripts/visual_marker_experiment.py --engine-root $ueRoot --project $sceneProject --lock $resourceLock --geometry-controls output/online-controls-run1 --planner-variant continuous_square --experiment-map /Game/SpatialLab/MarkerSquare_run1 --out output/marker-square-run1
python scripts/visual_marker_experiment.py --engine-root $ueRoot --project $sceneProject --lock $resourceLock --geometry-controls output/online-controls-run1 --planner-variant continuous_square_shortcuts --experiment-map /Game/SpatialLab/MarkerShortcuts_run1 --out output/marker-shortcuts-run1
python scripts/compare_visual_marker_runs.py --project $sceneProject --runs output/marker-legacy-run1 output/marker-square-run1 output/marker-shortcuts-run1 --out output/marker-comparison-run1.json
```

比较器只使用 CPU，重新核对全部实际像素/动作/独立真值、原 job/status 快照、唯一实际生产参数、当前源码闭包和共同 DLL/场景生成器；旧构建或更改后的策略不能直接冒充当前比较。原地图、输入和失败结果不覆盖。预算失败依旧进入分母；离线连续路线长度与实际三维路长分别保存。

## 数值查询索引对照

在线几何任务可显式选择 `--query-cell-policy legacy_floor` 或 `boundary_snap_1e-9_cell`，生成 schema 2；不传参数继续生成原 schema 1。新策略只规范极近内部网格线的查询索引，原坐标仍交给原连续足迹检查，详见 data-contract.md。原始图像/任务/轨迹不回填。

对每个条件设置不同的 `$indexTag`（仅字母数字/下划线），以相同源采集和当前源码运行；第二个条件把 tag 改为 `SnapRun1`、policy 改为 `boundary_snap_1e-9_cell`：

```powershell
$indexTag = 'FloorRun1'
$indexPolicy = 'legacy_floor'
python scripts/online_depth_experiment.py synthetic --engine-root $ueRoot --project $sceneProject --lock $resourceLock --experiment-map "/Game/SpatialLab/Index${indexTag}" --out "output/index-$indexTag-controls" --query-cell-policy $indexPolicy
python scripts/online_depth_experiment.py prepare --engine-root $ueRoot --project $sceneProject --lock $resourceLock --capture output/pitch-execution-run1/capture --capture-job output/pitch-execution-run1/render.job.json --out "output/index-$indexTag-prepare" --query-cell-policy $indexPolicy
python scripts/online_depth_experiment.py run --engine-root $ueRoot --project $sceneProject --lock $resourceLock --capture output/pitch-execution-run1/capture --capture-job output/pitch-execution-run1/render.job.json --tasks "output/index-$indexTag-prepare/tasks.json" --controls "output/index-$indexTag-controls" --out "output/index-$indexTag-execution" --query-cell-policy $indexPolicy

python scripts/compare_online_query_runs.py --project $sceneProject --capture output/pitch-execution-run1/capture --capture-job output/pitch-execution-run1/render.job.json --runs output/index-FloorRun1-execution output/index-SnapRun1-execution --controls output/index-FloorRun1-controls output/index-SnapRun1-controls --out output/index-comparison-run1.json
```

新比较器从原采集重建全部任务，核对当前同 DLL、两套同策略控制、原始生产参数和源码闭包，并重新从实际像素审计完整运动。两条件规范化后必须仅有索引协议差异；不能删除失败位姿或把不同世界目标混成同一任务。每次 run 的锁内输入也冻结其控制 tasks/job/status 快照。

## 城市受控颜色目标

先具备原始 pitch 采集和当前 DLL 的完整视觉控制证据（`visual_marker_experiment.py --planner-variant legacy_centers`）。只给构造器原位姿来源与新地图名，不提供目标结果；两组实际任务复用同一覆盖地图：

```powershell
python scripts/city_marker_experiment.py prepare --engine-root $ueRoot --project $sceneProject --lock $resourceLock --capture output/pitch-execution-run1/capture --capture-job output/pitch-execution-run1/render.job.json --experiment-map /Game/SpatialLab/CityMarkers_run1 --out output/city-marker-scene-run1
python scripts/city_marker_experiment.py run --engine-root $ueRoot --project $sceneProject --lock $resourceLock --capture output/pitch-execution-run1/capture --capture-job output/pitch-execution-run1/render.job.json --scene output/city-marker-scene-run1 --controls output/marker-legacy-run1 --planner-variant legacy_centers --out output/city-marker-legacy-run1
python scripts/city_marker_experiment.py run --engine-root $ueRoot --project $sceneProject --lock $resourceLock --capture output/pitch-execution-run1/capture --capture-job output/pitch-execution-run1/render.job.json --scene output/city-marker-scene-run1 --controls output/marker-legacy-run1 --planner-variant continuous_square_shortcuts --out output/city-marker-shortcuts-run1
python scripts/city_marker_experiment.py compare --project $sceneProject --capture output/pitch-execution-run1/capture --capture-job output/pitch-execution-run1/render.job.json --scene output/city-marker-scene-run1 --controls output/marker-legacy-run1 --runs output/city-marker-legacy-run1 output/city-marker-shortcuts-run1 --out output/city-marker-comparison-run1.json
```

prepare 的引擎查询、初始化与 NavMesh 重建仅为显式场景构造，受共享锁/超时/进程树守卫保护；原地图哈希必须保持不变。查看 placement-plan、fixture 和状态日志，空目标构造不能开始正向评测。run 才执行真正的城市像素目标任务；compare 只用 CPU 重建原完整任务，核对构造/控制/实际生产者/源码/DLL/地图并重算全部像素、动作、轨迹和事后真值关联。真实家具类别仍不在此入口的能力声明中。


## 观测支撑面模型的独立比较

CPU 同图查询只使用 input，不接收评测位姿；输出目录必须新建：

```powershell
python ue5/tools/support_plane_replay.py --input output/pitch-execution-run1/capture/input --out output/support-plane-replay-run1
```

物理转移先用当前 DLL 验收独立 supplied-route 控制，再冻结全部原帧与两个模型；它沿用固定局部三米查询和原执行预算，不是在斜坡上放宽验收。所有重任务继续传共享锁、超时及截止时间参数，见本节前述约定。

```powershell
python scripts/depth_path_experiment.py synthetic --engine-root $ueRoot --project $sceneProject --lock $resourceLock --experiment-map /Game/SpatialLab/SupportControls_run1 --out output/support-controls-run1
python scripts/depth_path_experiment.py prepare --ground-model-pair --engine-root $ueRoot --project $sceneProject --lock $resourceLock --capture output/pitch-execution-run1/capture --capture-job output/pitch-execution-run1/render.job.json --out output/support-prepare-run1
python scripts/depth_path_experiment.py run --ground-model-pair --engine-root $ueRoot --project $sceneProject --lock $resourceLock --capture output/pitch-execution-run1/capture --capture-job output/pitch-execution-run1/render.job.json --plan output/support-prepare-run1/plan.json --controls output/support-controls-run1 --out output/support-execution-run1
python scripts/compare_support_plane_transfer.py --project $sceneProject --capture output/pitch-execution-run1/capture --capture-job output/pitch-execution-run1/render.job.json --controls output/support-controls-run1 --run output/support-execution-run1 --out output/support-audit-run1.json
```

最后一条只使用 CPU，重建两模型完整计划并重算实际轨迹/反馈，核对原始生产者、当前模型源码、DLL、地图、控制快照和所有源像素。源码或输入改变时另建计划/输出，不改原始记录；离线 approved 数、物理到达数和颜色目标成功率分别报告。


## 保存观测的颜色/可见性失败诊断

尺寸候选仅作 CPU 同图消融，不运行新动作：

```powershell
python ue5/tools/marker_extent_replay.py --execution output/city-marker-legacy-run1/execution.json --out output/marker-extent-replay-run1.json
```

评测诊断先重建原始 native/控制/任务/像素和动作，再读独立真值。此次历史 worker 对应以下完整提交；自己的保存运行需使用与其原 guarded 字节匹配的提交，不能用当前分支名或回填旧哈希。输出文件必须新建。

```powershell
$workerCommit = 'c3bd856bf6d938934770d8d7f629299b91421e51'
python scripts/marker_perception_diagnostics.py --run output/marker-legacy-run1 --project $sceneProject --worker-source-commit $workerCommit --out output/marker-proxy-controls-run1.json
python scripts/marker_perception_diagnostics.py --run output/city-marker-legacy-run1 --scene output/city-marker-scene-run1 --controls output/marker-legacy-run1 --capture output/pitch-execution-run1/capture --capture-job output/pitch-execution-run1/render.job.json --project $sceneProject --worker-source-commit $workerCommit --out output/city-marker-proxy-run1.json
```

第一条诊断必须先通过保存中性场景的可见/隐藏/遮挡/无效出生控制。第二条仍只做 CPU 评测，没有 UE 启动、目标位姿输入策略或新的导航成功。`city_marker_experiment.py audit/compare` 也可显式传 `--worker-source-commit`；不传时继续严格核对当前 worker。历史验证不修改工作区、旧输出或原始 status/job；缺失依赖/代码变化/错误源提交直接失败。


## 目标规划视距对照

先做同图 CPU 对照；它严格重算旧 schema 2 响应，保留原观测/heading/搜索计数，只报告新动作可用性：

```powershell
python ue5/tools/marker_horizon_replay.py --execution output/city-marker-legacy-run1/execution.json --out output/marker-horizon-replay-run1.json
```

实际两条件均使用 `legacy_centers`。先以当前构建重新验收共同几何控制，再各运行完整颜色控制；为两个 `$horizonTag` 分别设 `ThreeRun1`/`OneRun1`，policy 分别为 `fixed_three_meters`/`execution_prefix_one_meter`，输出与地图须新建：

```powershell
$horizonTag = 'ThreeRun1'
$horizonPolicy = 'fixed_three_meters'
python scripts/visual_marker_experiment.py --engine-root $ueRoot --project $sceneProject --lock $resourceLock --geometry-controls output/online-controls-run1 --planner-variant legacy_centers --goal-horizon-policy $horizonPolicy --experiment-map "/Game/SpatialLab/Horizon$horizonTag" --out "output/horizon-$horizonTag-controls"
```

用同一当前构建 prepare 一张新的城市覆盖地图，两条件复用它；原构造规则和全部源位姿不变：

```powershell
python scripts/city_marker_experiment.py prepare --engine-root $ueRoot --project $sceneProject --lock $resourceLock --capture output/pitch-execution-run1/capture --capture-job output/pitch-execution-run1/render.job.json --experiment-map /Game/SpatialLab/HorizonCity_run1 --out output/horizon-city-scene-run1
python scripts/city_marker_experiment.py run --engine-root $ueRoot --project $sceneProject --lock $resourceLock --capture output/pitch-execution-run1/capture --capture-job output/pitch-execution-run1/render.job.json --scene output/horizon-city-scene-run1 --controls "output/horizon-$horizonTag-controls" --planner-variant legacy_centers --goal-horizon-policy $horizonPolicy --out "output/horizon-$horizonTag-city"
python scripts/compare_marker_horizon_runs.py --project $sceneProject --scene output/horizon-city-scene-run1 --capture output/pitch-execution-run1/capture --capture-job output/pitch-execution-run1/render.job.json --runs output/horizon-ThreeRun1-city output/horizon-OneRun1-city --controls output/horizon-ThreeRun1-controls output/horizon-OneRun1-controls --out output/horizon-city-comparison-run1.json
```

新控制必须通过原完整验收，实际城市运行前核验相同视距的控制。所有重任务继续传原共享锁及截止时间参数；CPU 比较器重建全部原任务、来源和实际像素/动作/轨迹，不补写历史原始输出。

## 独立坡面/低顶物理控制

当前 DLL 先重新通过既有 supplied-route 运动控制，再在全新自有地图采集固定合成几何的 RGB-D，并执行两种地面模型的冻结路线与三条独立参考。它不修改城市、在线策略、原生协议或任何高度预算：

```powershell
python scripts/depth_path_experiment.py synthetic --engine-root $ueRoot --project $sceneProject --lock $resourceLock --experiment-map /Game/SpatialLab/SupportRouteControls_run1 --out output/support-route-controls-run1
python scripts/support_physics_experiment.py run --engine-root $ueRoot --project $sceneProject --lock $resourceLock --controls output/support-route-controls-run1 --experiment-map /Game/SpatialLab/SupportPhysics_run1 --out output/support-physics-run1
python scripts/support_physics_experiment.py audit --project $sceneProject --controls output/support-route-controls-run1 --run output/support-physics-run1 --out output/support-physics-audit-run1.json
```

实际重任务继续附加本节的共享锁和开始/结束截止参数；每次输出及地图必须新建。最后一条仅用 CPU，核对原完整生产者守卫、固定步长参数和全部像素，再重算二十项模型路线及三项参考的实际轨迹/评分。`report.json` 的控制检查与各模型完整分母分别阅读，尤其保留高度合同失败、危险批准、未测量 null 与刻意参考接触。原始私有产物不上传仓库。

原生高度失败的探索性事后诊断只用 CPU，先执行同一完整原始审计，再比较端点与已验证构造地面的位置。它保留原成功/失败、接触、null和所有预算，不启动新运动：

```powershell
python scripts/support_physics_goal_diagnostics.py --project $sceneProject --controls output/support-route-controls-run1 --run output/support-physics-run1 --out output/support-goal-diagnostics-run1.json
```

这里的查询是水平前向一米处的构造地面，而非沿坡面弧长一米；端点接近度不能替代原生执行合同成功率。构造真值只进入此评测模块，不进入像素策略。

## 初始视觉输入的重复与一致性

对完整城市视距对照先重跑原严格审计，再按六幅PNG与固定类别/扫描计数/规划器/视距条件分组。缺少初始观察的任务仍保留，响应只忽略身份、时间和CPU计时元数据，其他动作字段冲突须显式报告：

```powershell
python scripts/marker_repeatability_report.py --project $sceneProject --scene output/horizon-city-scene-run1 --capture output/pitch-execution-run1/capture --capture-job output/pitch-execution-run1/render.job.json --runs output/horizon-ThreeRun1-city output/horizon-OneRun1-city --controls output/horizon-ThreeRun1-controls output/horizon-OneRun1-controls --out output/marker-repeatability-run1.json
```

该分析只用CPU，完整任务/成功失败与原始分母不变。重复像素不是独立试验，首个同输入动作一致也不能证明后续运动确定性或泛化。
