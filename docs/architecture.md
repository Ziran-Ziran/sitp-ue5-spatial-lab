# 架构与能力边界

目标链路：UE5 可控场景 → RGB/深度/时间/标定 → 定位和几何 → 语义识别/场景关系 → 可通行表示 → 路径规划 → 规则或引擎执行 → 反馈。

已有：ue5/GSVSLAM/Source 中的采集与场景逻辑；ue5/tools 中的输入检查、VO、重建、融合、ORB 回放与独立评测。另有 `ue5/tools/native/SpatialLab` 原生导航桥与 Game-world 执行器，由 `scripts/stage_navigation.py` 安装到独立本地场景副本。

可执行的 oracle 链路为：引擎标签/位置与碰撞查询 → 冻结目标和起点 → 欧氏最近或最短可达目标选择 → 引擎 NavMesh 路径 → AIController/CharacterMovement → 逐帧位置与非地面阻挡事件 → 独立终点评分。输入和全部条件显式标注 `engine_oracle`；不实现视觉语义识别。完整流程见 navigation-baseline.md。

尚未实现：语义模型、场景图、观测估计到可通行地图的转换、观测条件下的路径规划与统一在线接口。现有 oracle 不消费 RGB-D、SLAM 输出或评测真值文件。

GameScaling 不是任何一环的必需依赖。未来语义模块输入观测/估计位姿，输出带类别、关系、置信度和时间的实体；规划模块接收任务目标与可通行表示，输出路径或失败原因。模块合同先见 data-contract.md，再随实现版本化。

引擎真值独立保存，只在事后评测或明确标注的 oracle 对照使用。引擎 NavMesh 可以作为产品基线，但不能与观测估计混为同一条件。稀疏点图不等于可通行地图，几何精度、语义正确率和导航成功分别评测。
