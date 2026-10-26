# 架构与能力边界

目标链路：UE5 可控场景 → RGB/深度/时间/标定 → 定位和几何 → 语义识别/场景关系 → 可通行表示 → 路径规划 → 规则或引擎执行 → 反馈。

已有：ue5/GSVSLAM/Source 中的采集与场景逻辑；ue5/tools 中的输入检查、VO、重建、融合、ORB 回放与独立评测。另有 `ue5/tools/native/SpatialLab` 原生导航桥与 Game-world 执行器，由 `scripts/stage_navigation.py` 安装到独立本地场景副本。

可执行的 oracle 链路为：引擎标签/位置与碰撞查询 → 冻结目标和起点 → 欧氏最近或最短可达目标选择 → 引擎 NavMesh 路径 → AIController/CharacterMovement → 逐帧位置与非地面阻挡事件 → 独立终点评分。输入和全部条件显式标注 `engine_oracle`；不实现视觉语义识别。完整流程见 navigation-baseline.md。

尚未实现：语义模型、场景图、观测几何规划的在线目标导航与统一在线接口。现有 oracle 不消费 RGB-D、SLAM 输出或评测真值文件。

局部监督分支可以在实际移动时按胶囊脚底位姿记录有限距离的碰撞射线观测。记录器内部读取引擎标签，但只输出当帧命中的实体；离线策略只接收逐帧流和目标类别白名单。逐帧选择与有 TTL 的语义记忆可比较目标可用率及选择变化。采集路线仍由全局 oracle 选择，因此回放不是观测驱动闭环导航，也不能证明视觉识别能力。

独立的 `SpatialLabSearchRunner` 将局部监督观测接到实际搜索：固定起点/类别/几何探索点 → 当前帧或 TTL 记忆选择 → AIController/CharacterMovement → 路径反馈和下一帧。策略是 CPU 测试与 UE PythonScriptPlugin 共用的 `semantic_search_policy.py`，没有引擎 API、全实体表或评测标注输入。几何探索点和路径使用引擎 NavMesh，属于明确的几何对照条件。执行结束后才读取独立标注评分，并逐帧重现保存的动作决策；这种审计不模拟反事实运动。

`SpatialLabRgbd` 与 `render_replay_experiment.py` 在已保存的搜索眼点上同步重渲染 RGB-D。朝向只从过去运动推导，未记录实际第一人称相机朝向；此分支不反馈在线控制。采集计划、源任务/地图哈希和引擎位姿只留在采集/评测侧；算法输入目录只包含像素、内参、曝光说明与明确标注的源模拟时间。结构检查通过仍不代表图像适合识别、SLAM 或跨场景学习。

`local_depth_grid.py` 将光轴深度反投影到局部右/前/上坐标；使用显式水平相机与眼高先验，把观测地面支持与代理高度内障碍分开，未观测区域保持未知。`grid_path.py` 在保守余量后的网格运行独立 A*。`depth_grid_replay.py` 仅接收隔离序列，报告固定局部探针路径或失败；既不读取引擎几何/标签/全局位姿，也不推进 UE。稀疏地面采样和固定高度模型不构成物理可通行证明。

固定俯仰观察分支 `pitch_sweep_experiment.py` 在同一源眼点/运动 heading 上生成新的 0°/45°/90° 向下视图。传感器侧只公开已知相对俯仰外参，世界位姿留在评测侧；`pitch_sweep_report.py` 在同一局部参考系融合三幅深度后运行相同网格与 A*。这补充近场覆盖，未实现历史相机恢复、在线主动观察或跨时刻位姿融合。

`depth_path_experiment.py` 在几何模块完成后，由独立桥接层使用评测位姿把局部路径转成世界坐标。连续检查覆盖实际查询点到格心的连接段和所有路径段；未观测区域继续阻挡。`SpatialLabDepthRunner` 使用直接 AddMovementInput 和真实碰撞，完全不调用 MoveTo/NavMesh；引擎位姿只用于明确的重置/执行反馈。它转移冻结静态深度路径，未在移动中重新采图、建图或规划，也不衡量原语义目标任务成功。

`SpatialLabOnlineDepthRunner` 则在当前 Game world 停止角色并采三幅 RGB-D，将隔离像素和受监督的相对目标交给外部 CPU 数值策略。`online_depth_policy.py` 每次只重建当前局部几何，没有跨时刻地图或引擎网格；重观察条件执行至多一米前缀后重复此循环。采集位姿/任务世界目标单独保存，`online_depth_report.py` 从每次实际像素重算动作并核对全部成功与失败轨迹。停止观察的传感器成本计入回合 wall，未实现连续实时传感器、视觉定位或语义目标搜索。

视觉标记分支使用不同 condition：`visual_marker.py` 从当前 RGB 分割明确红/蓝颜色类别，再由光轴深度估计局部目标；`visual_marker_depth_policy.py` 只收到类别、像素与搜索计数，不接收相对真值目标。局部保持距离目标随后进入原深度路线模块；未发现类别时执行有界虚拟相机转向。原生公共执行器仅用引擎位姿重置/执行，不给视觉策略目标位置。受控场景真值只在 `visual_marker_report.py` 事后评分；颜色标记类别不代表家具语义或场景泛化。

GameScaling 不是任何一环的必需依赖。未来语义模块输入观测/估计位姿，输出带类别、关系、置信度和时间的实体；规划模块接收任务目标与可通行表示，输出路径或失败原因。模块合同先见 data-contract.md，再随实现版本化。

引擎真值独立保存，只在事后评测或明确标注的 oracle 对照使用。引擎 NavMesh 可以作为产品基线，但不能与观测估计混为同一条件。稀疏点图不等于可通行地图，几何精度、语义正确率和导航成功分别评测。


`support_plane_grid.py` 是独立的观测地面模型：只在已反投影点中拟合近脚底局部斜面，把实际脚底高度固定为零，保留未知区域和观测障碍。直立胶囊的净空按坡度跨足迹变化保守扩展，不能把斜面残差直接当作物理高度。`support_plane_replay.py` 对同一隔离像素比较水平/坡面模型；`support_plane_execution.py` 在像素规划后才读取原位姿生成冻结动作。新 schema 2 明确模型与 scope，旧在线/颜色策略和 DLL 不改变；冻结路径转移、在线观察、语义任务及物理坡道验收分别报告。


`marker_extent.py`/`marker_extent_replay.py` 单独评估已知受控盒体的尺寸上界；默认颜色策略保持原输出，尺寸先验没有接入 Game 任务或 worker 请求。`marker_visibility.py`/`marker_visibility_report.py` 只在评测侧使用原相机、盒体真值与保存深度，诊断投影、颜色、组件处理与选择阶段，不能作为精确实例可见性真值。`marker_perception_diagnostics.py` 先核验原执行再进行这项分析；评测模块不进入视觉 worker 导入闭包。
