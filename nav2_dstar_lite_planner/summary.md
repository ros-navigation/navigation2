# D* Lite 全局规划器 — 工作报告

## 一、背景与目标

为 Nav2（ROS 2 导航框架）实现一个独立的 D* Lite 增量启发式搜索规划器插件。D* Lite 的核心优势是：当代价地图发生局部变化时，不需要从头重新规划，而是复用之前的搜索空间，仅更新受影响的部分。这使其在需要高频重规划（如 100Hz）的动态环境中特别有价值。

本工作对应 Nav2 Issue #5770 的讨论。

## 二、D* Lite 算法原理

### 2.1 核心思想

D* Lite 是一个**反向搜索**（目标 → 起点）的增量算法。它维护两组值：

- **g(s)**：从目标到状态 s 的当前最短路径代价
- **rhs(s)**：一步前瞻值 = min_{s' ∈ Succ(s)} (c(s, s') + g(s'))

状态一致性判断：
- 若 g(s) = rhs(s)，状态是**一致的**（consistent）
- 若 g(s) > rhs(s)，状态是**过一致的**（overconsistent）—— 意味着找到了更短的路径
- 若 g(s) < rhs(s)，状态是**欠一致的**（underconsistent）—— 意味着之前更短的路径被堵死了

算法维护一个优先队列 U，存放所有不一致状态，按 Key 排序。

### 2.2 Key 的计算

Key 是字典序对 (k1, k2)：

```
k1 = min(g(s), rhs(s)) + h(start, s)
k2 = min(g(s), rhs(s))
```

其中 h(start, s) 是启发函数，在反向搜索中用 s 到起点的欧几里得距离。Key 保证了算法优先扩展最有希望通往起点的状态。

### 2.3 主循环 computeShortestPath

```
while U 非空 AND (U.top.key < key(start) OR rhs(start) ≠ g(start)):
    u = U.pop()
    if u 是过一致的 (g > rhs):
        g(u) = rhs(u)
        更新 u 的所有前驱
    else (欠一致):
        g(u) = ∞
        更新 u 的所有前驱和 u 自身
```

关键技巧是**懒惰删除**：更新状态时不从队列中删除旧条目，而是插入新条目。弹出时通过比较条目中的 key 和当前计算的 key 来判断条目是否过期。

### 2.4 增量更新

首次规划：完整运行 initialize() + computeShortestPath()。

后续规划：
1. 对代价地图做快照（snapshot），下次对比差异
2. 找出所有代价发生变化的格点
3. 对这些格点及其前驱调用 UpdateVertex()
4. 再次运行 computeShortestPath()（只扩展受影响的部分）
5. 通过贪心下降法从起点沿最小 g 值提取路径

为防止陷入局部最优，每 N 次增量更新后（replan_interval 参数）强制做一次完全重规划。

### 2.5 路径提取

从起点出发，在 8 连通邻居中每次选择 `edge_cost + g(succ)` 最小的后继，直到到达目标。这与在已标注的搜索空间中执行贪心策略一致。

## 三、架构设计

### 3.1 三层结构

```
DStarLitePlanner (GlobalPlanner 适配器)
  ├── ParameterHandler (参数管理)
  └── DStarLite (核心算法引擎)
```

**DStarLitePlanner** — 实现 `nav2_core::GlobalPlanner` 接口：
- `configure()` / `activate()` / `deactivate()` / `cleanup()` — 生命周期管理
- `createPlan(start, goal)` — 对外接口，处理坐标转换、代价地图加锁、异常处理、路径滞后

**DStarLite** — 纯算法引擎：
- 管理状态池、优先队列、代价地图快照
- 与 ROS 解耦，可独立测试

**ParameterHandler** — 6 个动态参数：
- `allow_unknown` — 是否允许穿越未知区域
- `max_iterations` — computeShortestPath 最大迭代次数
- `replan_interval` — N 次增量后强制完全重规划
- `hysteresis_factor` — 路径切换滞后因子（≥1.0）
- `terminal_checking_interval` — 取消检查间隔
- `use_final_approach_orientation` — 终点朝向策略

### 3.2 文件结构

```
nav2_dstar_lite_planner/
├── CMakeLists.txt              # ament_target_dependencies 构建
├── package.xml                 # format=3, Apache-2.0
├── dstar_lite_planner.xml      # pluginlib 注册描述符
├── README.md
├── include/nav2_dstar_lite_planner/
│   ├── dstar_lite.hpp          # 核心算法数据结构与类声明
│   ├── dstar_lite_planner.hpp  # GlobalPlanner 适配器
│   └── parameter_handler.hpp   # 参数结构体与处理器
├── src/
│   ├── dstar_lite.cpp          # 核心算法实现 (~350 行)
│   ├── dstar_lite_planner.cpp  # 插件适配器 (~230 行)
│   └── parameter_handler.cpp   # 参数实现 (~130 行)
└── test/
    └── test_dstar_lite.cpp     # 9 个测试用例
```

### 3.3 关键数据结构

```cpp
// 地图格点坐标
struct CellIndex { int x; int y; };

// D* Lite 字典序 Key
struct Key { double k1; double k2; };

// 每个格点的搜索状态
struct DStarLiteState {
    double g{INF};      // 从目标到当前格点的最短距离
    double rhs{INF};    // 一步前瞻值
    Key key{};          // 队列中的 Key（用于懒惰删除检测）
    bool in_queue{false};
};

// 状态存储：用 unordered_map 做 O(1) 查找，vector 做紧凑存储
std::vector<DStarLiteState> state_pool_;      // 紧凑数组
std::unordered_map<CellIndex, int> state_lookup_;  // 坐标 → 数组索引
```

## 四、开发过程中的关键修复

### 4.1 HUMBLE 兼容性适配

主分支的 Nav2 代码使用 `nav2_ros_common`（Kilted+ 新增包），但目标环境是 Humble。做了以下适配：

| 主分支写法 | Humble 兼容写法 |
|-----------|----------------|
| `nav2::LifecycleNode` | `rclcpp_lifecycle::LifecycleNode` |
| `nav2_util::ParameterHandler<Params>` | 自实现的 ParameterHandler |
| `createPlan(start, goal, viapoints, cancel_checker)` | `createPlan(start, goal)` |
| `nav2_core/planner_exceptions.hpp` | `nav2_core/exceptions.hpp` |
| `nav2_core::GoalOccupied` 等专用异常 | `nav2_core::PlannerException` |
| `nav2_core::nav2_core` CMake 目标 | `ament_target_dependencies` |

### 4.2 遍历代价为零的 Bug（最关键的修复）

**原始代码：**
```cpp
// Free space cost = 0 → edge cost = distance * 0 = 0
return costmap_cost / MAX_NON_OBSTACLE;
```

**问题：** 空旷区域的代价为 0，导致所有边代价为 0。算法中所有 g 值都变为 0，路径提取时无梯度可循，陷入循环/失败。

**修复：**
```cpp
// Free space = 1.0, max non-obstacle = 2.0 → 始终正代价
return 1.0 + costmap_cost / MAX_NON_OBSTACLE;
```

D* Lite（和 A*）要求边代价严格为正才能保证正确性。这是经典的"零代价边"陷阱。

### 4.3 CMake 链接方式

Humble 的 nav2 包使用传统的 ament 导出变量（`${nav2_util_LIBRARIES}`），而非现代 CMake 目标（`nav2_util::nav2_util_core`）。使用 `ament_target_dependencies()` 自动处理了这些差异。

## 五、测试结果

```
[==========] 9 tests from 2 test suites ran. (39 ms total)
[  PASSED  ] 9 tests.

DStarLiteAlgorithm:
  test_initialize          - 验证目标 rhs=0，在队列中
  test_isSafe_with_unknown - 验证 allow_unknown 对未知/致命代价格点的判断
  test_withinBounds        - 边界检查
  test_simple_path         - 空旷空间中找路径
  test_path_around_obstacle- 绕障碍物找路径
  test_no_path             - 完全阻塞时正确返回无路径
  test_incremental_replan  - 代价地图变化后增量重规划

DStarLitePlanner:
  test_lifecycle           - configure/activate/deactivate/cleanup 完整周期
  test_reconfigure         - 动态参数更新端到端验证
```

## 六、使用方式

在 Nav2 配置中加入：

```yaml
planner_server:
  ros__parameters:
    planner_plugins: ["DStarLite"]
    DStarLite:
      plugin: "nav2_dstar_lite_planner/DStarLitePlanner"
      DStarLite.allow_unknown: true
      DStarLite.max_iterations: 100000
      DStarLite.replan_interval: 10
      DStarLite.hysteresis_factor: 1.05
```

## 七、已知局限性

1. **全向运动模型**（8 连通网格），不支持非完整约束（如 Dubins/Reeds-Shepp）
2. **无碰撞检测** — 仅检查格点代价，未考虑机器人足迹在格点间的碰撞
3. **仅全局规划** — 不替代局部规划器，适合高频重规划场景的全局路径更新
4. **Humble API** — 异常类型为通用 `PlannerException`，在新版 ROS 2 中可细化为专用异常
5. **未做性能优化** — 状态存储用 `unordered_map` 而非直接数组索引，适用于原型阶段

## 八、未来方向

- 在 Smac Planner 中探索 "D*-mode"，将非完整约束搜索邻域与增量搜索结合
- 使用直接数组索引替代 hash map 提升性能
- 支持 Field D*（在格点间做线性插值，生成更平滑的路径）
