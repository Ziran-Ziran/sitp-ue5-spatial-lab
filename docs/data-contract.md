# 空间语义与规划数据合同

当前可执行的 RGB-D 输入以 development.md 和 inspect_slam_input.py 为准。可执行的本地 `engine_oracle` 任务/计划/执行/评分合同见下文；其余语义/规划字段仍是设计合同，尚未实现统一在线传输。

- 观测：schema_version、session_id、frame_id、单调采样时间与时钟来源、RGB、可选深度、独立内参、深度单位/含义/无效值。记录模态缺失和同步误差；不得从 poses.jsonl 提取时间或标定。
- 估计空间：坐标系/左右手系、单位、Twc 方向、map_id、地图版本、跟踪状态、置信度及延迟。新图原点独立，不能拼接不连续轨迹。
- 语义实体（规划）：实体 ID、类别/文本描述、三维范围、关系、来源帧、置信度与最后更新时间。区分传感器估计和引擎标签；标签是评测真值时不进入估计分支。
- 规划输入/输出（规划）：目标语义/位置、代理尺寸与动作约束、可通行/未知/障碍表示、地图版本；输出路点、预计代价、失败原因及重规划条件。几何点云不是可通行表示。
- 评测：真实路径/语义标签独立保管，明确 oracle 基线。报告成功率、碰撞、路长比、覆盖率、语义指标、定位有效率与端到端延迟，不只报几何误差。

experiments 用按 scene 的 train/validation/test 互斥标识；不能从同一场景相邻帧随机划分后称跨场景泛化。实验条件、路径和真值边界固定后再统计结果。

## 本地引擎 oracle 合同（schema_version 1）

`navigation_tasks.py` 生成冻结任务清单；`plan_navigation.py` 生成计划；原生 `SpatialLabRuntime` 输出执行；`navigation_report.py` 在执行后单独评分。所有产物留在忽略目录。清单采用排序键、紧凑 JSON、UTF-8 的 SHA-256；计划和执行必须引用同一哈希与完整 `/Game/SpatialLab/...` 地图 package 标识。执行器和评分器都拒绝错地图，不接受地图短名。执行记录不能省略、重复或更改任务。

- 坐标是 UE 左手系 X 前/Y 右/Z 上，单位厘米。任务 `start_cm`、`goal_cm` 为地面/NavMesh 点；原生代理胶囊中心从 start 上移 90 cm，半高 88 cm。执行 `final_position_cm`、`trajectory_cm` 为胶囊脚底位置（中心减当前半高），不代表相机位姿。
- 固定代理半径 34 cm、高 176 cm、速度上限 300 cm/s、台阶 35 cm、坡度 45°。目标为 `chair` 与 `lamp`，实体来源 `engine_actor_name_oracle`，含引擎位置和实测可用接近点；这些信息只在 oracle 条件使用。
- 清单记录 seed、起点、类别、成功半径、时间与路长预算、策略和代理。固定种子从实测可用网格选三个起点，距所有目标至少 500 cm，距任一标注楼层高度不超过 100 cm；不按目标可达性筛选，不删除孤立区域。每起点两类别、两策略，共 12 回合。
- 欧氏最近策略只查询最近实体的最近接近点；最短可达策略查询所有有效接近点并选择最短完整路径。`path_queries` 和选择耗时单独记录；部分路径不是完整路径，两者查询预算不同。
- 原生执行采用 Game world、AIController、CharacterMovement，每回合独立出生和清理。控制入口不发送桌面键鼠。`direct_line` 仅用于合成墙体碰撞控制案例；城市比较使用引擎 NavMesh 执行。
- 执行保存 status、所选实体、脚底轨迹、连续路长、模拟时间、进程内 wall 时间，以及碰撞事件。碰撞定义为法线 Z 小于 0.7071 的非地面阻挡事件，按 0.1 模拟秒去抖；不是唯一障碍数、穿透深度或真实伤害。未生成代理的回合碰撞数为 null，不能当零。
- Game world 以固定 1/60 秒步长加速运行。模拟耗时与 wall 耗时分别记录；延迟汇总为选择调用加回合执行的 wall 时间，排除 UE 启动，不据此声称实时采集或传感器到动作延迟。
- 成功要求执行到达且脚底终点距所选、类别正确的标注 Actor 原点不超过 300 cm。评分从独立实体表核对类别/位置和实际终点，不信任单独的 success 标志；不衡量视觉分类准确率、物体接触或真实可交互性。所有固定回合进入成功率分母，缺失执行也是失败。

完整 `finished` 与回合数检查通过才算运行结束；UE 进程退出码 0 不能代替产物验收。合成 UE 控制案例、静态场景回合、离线合成单元测试分别标注，原始报告不改写。

## 局部引擎可见性监督与选择回放（schema_version 1）

`--record-observations` 在 Game world 启用 `observations.jsonl`。条件必须为 `engine_local_visibility_supervision`，语义来源 `engine_actor_label`，位姿来源 `engine_ground_truth`，路线来源 `global_engine_oracle_selected_routes`。这是碰撞几何监督，没有渲染图像、像素分类或检测置信度；观察与到达判据也不等价。

- 首行为 stream：完整地图、冻结清单哈希、同次原生执行生成的 UUID `attempt_id`、厘米坐标系和传感器参数。执行 JSON 使用同一 attempt；关联检查还核对有帧回合的起终脚底位置（5 cm 内）、初始模拟时刻 0 与末帧/执行时长（误差 1e-4 秒内）。`initial_position_cm` 记录控制器接管并自动贴地后的实际脚底，独立于任务计划起点；轨迹与距离累计也从实际脚底开始，排除初始化贴地位移。早期本地产物曾以计划点作为轨迹首点，原始文件保留，不与修正后的路长混用。不同尝试不能混用。旧的无 attempt 流不由此合同接受，原始文件不改写。
- 固定全景范围 2000 cm，按眼点到 Actor 原点距离筛选，眼点在脚底上方 150 cm。每 0.1 模拟秒采样，另强制首末帧；同一模拟时刻不重复采样。向 Actor 包围盒中心及上下半个 Z extent 共三个点发复杂碰撞射线；首个阻挡命中必须是该 Actor。射线落入空隙不算检测，因此细杆、空心或碰撞不完整的物体可能漏报。它不是相机视锥、像素覆盖或真实可见性真值。
- episode_start/end 包含 run_id；未生成代理的失败也保留边界和零帧。frame 包含连续 frame_id、回合内 `t_sim_s`、从记录器启动起单调的 `t_monotonic_s`、脚底和眼点、当帧 entities。模拟时间每回合重置；wall 时钟不重置，也不等于实时采样频率。
- 每个 entity 仅有 entity_id、chair/lamp 类别、引擎原点 position_cm 和命中样本数 1..3。流中没有全 catalog、真值目标、未来帧或未观察实体。严格字段白名单拒绝额外特权字段；读取器逐条消费，核验连续帧号、时钟、范围、回合和最终计数/footer。
- 策略的 goal-only JSON 仅提供地图、清单哈希、run_id 和目标类别；清单导出与回放是分开的命令，回放不接收原始任务 catalog、起点或坐标。goal-only 可跨同一冻结任务的多次 attempt 复用；回放报告保留当前 attempt。
- `frame_only` 只保留当前观测；`temporal_memory` 默认 TTL 5 模拟秒，可预先显式指定 0..120 秒，边界包含到期时刻。每回合独立清空；按当前脚底至已观察实体原点的距离选择，平局按 ID。所有位置和类别均是明确的引擎监督，没有从未来补写历史记忆。
- 报告目标可用帧比例、首次目标观测、选择变化、策略 CPU wall 耗时和失败原因；没有观测的回合仍在回合分母中。路线受全局 oracle 偏置，回放只衡量选择/记忆可用性，不能将其转写成反事实导航成功率、路长收益或视觉语义准确率。

## 观测驱动目标搜索（schema_version 1）

`semantic_search_experiment.py` 将 `semantic_search_policy.py` 同步接入 Game world。条件仍为 `engine_local_visibility_supervision`；观测 stream 的路线来源改为 `observation_dependent_engine_navmesh_geometry`。传感器仍使用相同的三个包围盒射线样本，没有因终点真值改变采样参数。

- mission bundle 仅包含完整地图、来源标注清单的哈希、几何来源声明和 missions，并独立计算 `manifest_sha256`（排序键、紧凑 UTF-8 JSON，排除哈希字段自身）。每条 mission 的白名单为 run_id、category、policy、start_cm、时间/路长/到达/成功预算和 exploration_points_cm。没有目标实体、真实目标坐标或 oracle 计划。独立评分核对原标注 episodes × 两种策略的完整集合及类别、起点、预算，重算哈希不能合法化删回合。
- 城市探索点只从导出的起点查询生成：X/Y 偏移各为 -1500、-750、0、750、1500 cm，排除中心；NavMesh 投影范围 150/150/100 cm，拒绝高度变化超过 100 cm 或 Pawn 胶囊受阻点。没有按目标、路径连通性或评测结果筛选。几何来源是引擎 NavMesh，不是观测估计地图。
- 策略接收当前帧及对应 action_id 的 movement feedback（none/moving/arrived/rejected/blocked/cancelled），不接收世界 API。`frame_only` TTL 为 0，`temporal_memory` 为 5 模拟秒。最近的已观察目标可打断探索；目标失效则取消；移动失败的实体在本回合不再尝试。未发现目标时按当前距离及索引选择尚未尝试的固定探索点，每点只尝试一次，耗尽则保留 `exploration_exhausted`。
- 观察目标的移动点为其原点沿当前观察者方向退让成功半径的 0.6 倍，避免要求角色站到物体原点。它是预先固定的规则，没有读取标注接近点。是否真正可达仍由引擎路径反馈决定，部分路径不允许作为成功路径。实际观测目标在成功半径内时输出 `candidate_reached`，供事后独立核验。
- 原生执行保留相同胶囊、速度、步长、碰撞定义、模拟时间/路长预算和 120 秒 wall 上限，策略另有 64 次命令预算。每个 episode 保存全轨迹和逐帧 policy_trace（frame_id、t_sim_s、输入 feedback、输出 action、桥接 wall 时间）。`navigation_requests` 统计 MoveTo 调用，不能当作内部 NavMesh 查询数，不能与 oracle 的显式 path_queries 声称等算力。
- 评分仅接受 `ue_game_world_observation_dependent_search` 的实际执行和对应路线声明。成功必须为 `candidate_reached`、观测来源与记忆时效有效、全动作能逐帧重现，且独立标注中的类别和实际终点距离通过。oracle 的 arrived/success 状态不能绕过搜索来源检查。失败、零帧及缺失执行进入分母；未知碰撞不当作零。模拟与 wall 时间分开，episode wall 包含同步 Python 桥和移动，排除引擎启动。
- 原生模块和两个在线 Python 源文件的哈希在拿到共享锁后检查；原生构建记录源码与 DLL 对应关系。响应文件每帧独占创建，校验 attempt/run/frame 后删除；旧响应或 Python 异常使回合失败，不沿用上一次动作。

## 与搜索关联的 RGB-D 静态位姿回放（schema_version 1）

`rendered_replay.py` 仅接受已完成的观测驱动搜索流，默认按每回合源帧索引均匀选取 4 帧（可预先指定 2..8，含首末帧）；零帧失败仍保留。回放相机位置采用记录眼点，yaw 只由当前与上一帧 XY 位移推导，静止时保留朝向，初始 yaw=0，pitch/roll=0。它没有恢复实际游戏相机，也不从未来帧或目标坐标选择视角。

- 采集计划绑定源 stream、原 search job 与 execution 的 SHA-256、attempt UUID、冻结地图文件字节哈希和回放计划哈希。源 job 必须实际输出对应观测/执行，状态、首末脚底与模拟时间匹配；同名地图改内容也会被拒绝。bounded status 保存原始任务文件字节的 `spec_sha256` 和锁内实际核验的 `verified_input_sha256` 快照；来源与控制检查均核对二者，不能修改任务哈希后复用旧状态。无快照的历史状态不补填，须新执行。只绑定源 `.umap` 字节，外部引用资产与光照按当前加载状态重渲染，不声称恢复全部历史引擎状态。
- 原生 `CapturePose` 使用独立 RGB/Depth SceneCapture，在同一静态相机变换发起渲染、同步 flush 后读回。RGB 为 FinalColorLDR、BGRA8 经 PNG 编码，不翻转；通常解码成 BGR。深度为 SceneDepth R32 float 厘米，以 RCM_MinMax 原值读取，四舍五入为 uint16 光轴毫米 PNG。0 无效，超过 30 m 或非有限值写 0；读取器也将历史 65535 视为无效，原生输出不生成有效 65535。
- 固定尺寸 320×240、水平视角 90°，无畸变，fx=fy=160、cx=159.5、cy=119.5，像素中心采用整数坐标。局部相机反投影约定为 CV 右手系 X 右/Y 下/Z 前；评测位姿保留 UE 左手系厘米、Pitch/Yaw/Roll 角度。合成彩色方块的投影/尺寸检验是像素级粗检查，不是亚像素标定或真实传感器标定。
- 曝光预先显式冻结；可按场景已有独立光照/后处理设置确定，不按导航成绩或语义真值挑选。采集进程临时设置扩展亮度范围为 EV100，bias=0，固定 min/max 为指定曝光，并记录原工程亮度模式及 override scope。原生拒绝亮度单位不符的调用；不写项目配置、素材或系统设置。控制案例验证 EV1 与 EV13 改变 RGB 而不改变深度。
- `capture/input/sequence_NNN/` 是算法侧，只允许 sensor.json、timestamps.jsonl、frames/、depth/。不含引擎位姿、地图名、源回合标签或实体表。timestamps 为连续新 frame_id 与 `t_source_sim_s`，来源 `replay_source_simulated_seconds`；它是保存的回合模拟时间，既不是渲染吞吐率，也不是物理传感器采样时钟。当前 SLAM 输入工具未适配此专用回放时钟，不据此声称 SLAM 已验收。
- `capture/evaluation/` 独立保存计划、源回合/源帧映射与实际引擎相机变换。采集器写出后立即记录每对 PNG 的哈希；消费端核对计划、实际位姿、源时间、数量、captured 状态与文件哈希，拒绝换图、串回合和任意单调时间。控制像素目录还必须对应同一次 bounded job 的实际 `--out`、原生 DLL 和控制脚本。
- 输入检查报告逐帧有效深度比例、RGB 空间变化、白色裁剪/暗区/有色像素比例及哈希，不丢弃暗帧或空深度。`input_contract_ready` 只表示配对与来源合同完整，不能当作视觉识别、可通行估计、实时采集或 SLAM 成功。早期过曝或缺少关联字段的本地产物保留，不按新协议补写原始数据。

## 深度局部网格与路径探针（schema_version 1）

`depth_grid_replay.py` 的条件为 `depth_geometry_upright_eye_height_prior`，只接收算法侧 `sequence_NNN` 目录。每帧独立处理，不读取评测目录、引擎标签/全局位姿或 NavMesh，也不融合跨帧位姿。内参、深度单位和源时钟沿用上述 RGB-D 回放合同。输入文件哈希在处理前后核对；报告路径可用性，不能转写成导航成功。

- 先验固定相机水平、眼高 1.5 m；光轴深度反投影到米制 `[right,forward,up]`，参考地面 `up=0`。默认格宽 0.25 m，right 范围 `[-5,5)`、forward 范围 `[-2,10)`；每格状态 -1 未知、0 观测地面支持、1 障碍、2 观测负空间危险。
- `abs(up)<=0.1m` 的点提供地面支持；`0.1<up<=1.76m` 的点标记代理身体高度内障碍，`up<-0.1m` 标记相对参考地面的落差危险。障碍和负空间都覆盖同格地面支持，二者同时存在时记为障碍。高于身体的点不创造可走区域。不沿空射线推断地面、不填充脚边盲区。地面支持是离散采样估计，不能证明整格地面、坡度、台阶、动态障碍或物理安全。
- 只在地面支持格中规划。障碍、未知与边界外区域均用 `radius/resolution + sqrt(2)/2` 格的中心距离作保守余量，默认半径 0.34 m；格子外接圆近似会拒绝部分实际足够宽的通道。A* 使用 8 邻域、欧氏启发与米制边长，对角移动还要求两个正交邻格可走；默认最多扩展 250000 格。
- 默认探针是固定局部 `(right=0,forward=0)` 到 `(0,3m)`，与语义目标和评测真值无关。状态包括 path_found、start/goal_unobserved、obstacle、negative_hazard、clearance_blocked、outside、no_path 与 expansion_budget。路径经过格心，路径长度不包含任意连续查询点到格心的偏移。报告地面/障碍/负空间/未知/余量格数和 CPU 耗时，所有失败保留。
- 水平相机通常看不到脚边地面，当前静态回放的实际起点可能全部未知。不得偷偷换成已观察的起点或假设脚边走廊可通行。合成已支持区域内的路径检查应与真实起点失败分别标注；新增向下观察必须生成新传感器数据及明确的相对外参合同。

## 固定相对俯仰静态观察（schema_version 1）

`pitch_sweep_protocol.py` 在未修改的基础回放计划外包装固定 views 与 `sweep_sha256`。三视角固定为 level/0°、down45/45°、nadir/90°，向下为正；每个源帧的位置和 heading 一致，曝光与基础计划一致。源任务、地图字节、原任务状态快照和帧选择继续使用原来源校验。它采集新静态图像，不表示历史搜索相机实际转动过，也没有测量转动耗时。

- 算法输入为 `input/sequence_NNN/{level,down45,nadir}/`；每个 view 只含 sensor.json、timestamps.jsonl、frames/、depth/。modality 为 `rgbd_offline_calibrated_pitch_view`，传感器外参仅含 `camera_mount=coincident_eye_pure_pitch_relative_to_level_heading` 和 `relative_pitch_down_deg`，不含世界位置、全局朝向、源 run/实体标签或地图。三 view 帧数、源时刻和曝光须完全一致。
- 相机光轴深度向量的局部 right 为 `x`、forward 为 `z`、相对 up 为 `-y`。俯角 θ 后 `forward=z*cosθ+up*sinθ`、`up=-z*sinθ+up*cosθ+eye_height`；只在同一个静态源帧融合，不跨源时刻融合位姿。仍使用原格宽、眼高、地面带、身体高度与半径。三幅深度带来额外观测和计算，不能归因于同等观测预算下的策略改进。
- 评测侧保存每 view 的实际相机位置与旋转、源帧/时间对应及每对 PNG 哈希。位置误差上限 0.001 cm，旋转用 UE 基向量比较，允许垂直向下时等价的欧拉角表示。输入检查继续核对全部图像、源时刻、视角和原来源；不能交换 view 文件或补写时间。
- 真实内置几何控制包含固定高度平地、中心墙体、两米宽且降低一米的通道；检查已知光轴深度、脚边观察、平地路径以及墙/落差阻挡。控制通过是渲染与离线几何验收，不是 CharacterMovement 导航验收。
- `pitch_sweep_report.py` 保留同帧水平/三视角路径结果、危险与未知格子、输入哈希及 CPU 耗时。固定局部探针仍为 `(0,0)` 到 `(0,3m)`；控制案例单独使用 `(0,4m)`。这些是相关帧上的几何可用性，不能当作独立导航回合成功率。眼高/参考地面先验在坡地上仍可能保守拒绝起点、终点或路径，不按真值放宽。

## 冻结深度路径的物理转移（schema_version 1）

条件为 `depth_grid_pose_supervised_frozen_path_execution`。`depth_execution_plan.py` 从完整三视角采集重算单/三视角全部帧，几何模块只接收隔离的 input。完成规划后，独立桥接层才读取源眼点/heading，把眼高减去 150 cm 得到计划脚底，并将局部 right/forward 米制路径转为 UE 世界厘米。此位姿是明确的重置和动作监督，不是视觉定位。

- 连续 gate 对实际起终点、格心连接段及每条路径段检查。将所有非地面格和边界外区域按轴对齐方形半径 `0.34+0.10m` 扩张，线段与扩张格接触也拒绝；0.10 m 固定余量来自 5 cm 到点容差和 5 cm 重置 XY 容差。它比圆形胶囊保守，也不能证明格内连续地面、坡度或动态安全。默认 10 cm 执行走廊用于监测偏离，不额外宣称整条该走廊通过物理认证。
- 冻结 plan 绑定地图字节、全部源帧、采集计划/位姿文件、像素和几何源码哈希，以及原采集 job/status 快照和历史渲染 DLL 哈希。新动作 DLL 可另行编译，但必须通过当前物理控制；历史渲染 DLL 不得冒充当前动作 DLL。run 从当前源重算完整 bundle，拒绝删失败帧或修改参数的缓存计划。
- 每条规划/检查失败保留记录，不出生代理。批准路线在 Game world 使用 34/88 cm 胶囊、300 cm/s、35 cm 台阶和 45° 坡度，先查询出生碰撞，静止贴地 0.25 模拟秒后核对实际 XY 漂移不超过 5 cm、Z 不超过 10 cm。初始化贴地不计移动路长。深层初始穿透另由 0.01 cm query-only sweep 检查，不移动角色探测。
- `SpatialLabDepthRunner` 直接 `AddMovementInput`，没有 MoveTo、NavMesh 路径/投影调用；位置反馈来源 `engine_ground_truth_for_actuation`。按固定 1/60 秒运动，逐点 5 cm 容差；限制 30 模拟秒、30 执行 wall 秒、20 m 路长、10 cm 跟踪偏差、50 cm 垂直偏离与 2 模拟秒无进展。每点停止速度后转向，保存全部 post-physics 脚底轨迹、到点事件、重置与运动耗时。碰撞沿用 0.1 模拟秒去抖的非地面阻挡事件。
- 独立评分核对原生 evidence、UUID、冻结完整集合/原输入、真实轨迹首尾、累计三维路长、固定步长、速度、到点次序/时刻/位置、预算及到达后的走廊。失败与缺失仍在成功率分母；未开始测量的碰撞/路长/时延为 null，零有效测量时对应汇总也为 null，另报实际测量数。
- 执行 scope `frozen_static_depth_transfer` 是冻结静态观察的局部探针转移，不是移动中 RGB-D 采集、在线重规划或原语义任务。`synthetic_supplied_route_controls` 使用显式给定的路线验证动作/墙体/出生/零运动/不执行案例；不能当作深度估计成绩。执行 wall 不含 UE 启动、重置和离线规划，不能声称端到端实时延迟。

## 停止采图的在线深度局部任务（schema_version 1）

条件为 `online_stopped_depth_geometry_pose_supervision`。任务从原冻结采集计划的全部源位姿生成：每个源帧固定原起点、heading 和前方三米世界目标，各执行 `initial_sweep_once` 与 `receding_sweep`。只用源位姿定义重复任务，不给算法历史图像或按旧规划成功筛选起点。它是相关位姿上的局部几何任务，不是原来的语义目标搜索。

- 首次观察和后续观察都来自本次 Game world 的实际角色脚底上方 150 cm。角色停止后同步采集水平/45°/垂直向下三视图，heading 固定为任务方向；已知虚拟相机外参不等于估计相机姿态或测量机械转动成本。像素、内参、眼高/俯仰先验沿用原几何参数，曝光来自预先冻结的独立场景设置。采集临时 EV100 设置只影响当前进程。
- 单步 `input/` 仅含 observation.json 与 level/down45/nadir 各自的 rgb.png、depth.png。PNG 哈希在采集后立即封存；RGB 以 BGR8 解码，深度为 uint16 光轴毫米。元数据仅有固定标定/相对视角、眼高先验与本回合 `t_sim_s`，时钟来源 `game_world_simulated_seconds_at_stopped_capture`。三幅图之间不推进游戏模拟；这是停止观察，不是连续采样或真实传感器时钟。
- 策略请求白名单为 schema_version、attempt_id、run_id、step_id、input_dir、goal_right_forward_m、mode、t_sim_s。当前相对目标由执行侧用引擎实际位姿与冻结世界目标换算，属于显式位姿监督；请求不包含世界坐标、地图、标签、引擎网格或未来图像。数值策略不导入 unreal；UE 内嵌 Python 只用标准库启动当前外部 Python 数值进程，20 秒超时且属于父 Job Object 进程树。
- 每步以当前三幅深度重建原局部网格/A* 和连续足迹 gate。一次观察条件执行整条初始路线；重观察条件只执行沿该路线至多 1 m 的连续前缀，保持折点，停止后重新采图和规划。没有跨时刻地图融合或语义记忆。最多八次三视角观察，额外观察与计算预算必须披露。
- 直接动作、胶囊、地形与到点/重置/跟踪限制沿用冻结路线执行器；在线回合上限 30 模拟秒、60 wall 秒、20 m 路长。whole-episode wall 包含同步渲染、数值子进程、等待和移动，单列 capture/bridge wall 与重置耗时；排除 UE 启动。固定步长仍为 1/60 秒加速模拟，不能声称实时性能。
- 独立审计从保存像素重算每步动作，核对实际相机、脚底与轨迹模拟时刻、相对目标换算、步号/attempt、精确响应与到点反馈，再核验终点、路长和预算。失败、未出生、无观测和缺失回合保留；未开始测量的运动/碰撞为 null。合成控制中的新墙按同一规则在两条件实际移动 50 cm 后启用，不把此干预当作静态城市数据。
