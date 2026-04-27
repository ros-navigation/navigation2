# D* Lite Benchmark Report

**System:** 32-core Intel, 5752 MHz, Google Benchmark 1.6.1

## 1. Open Space — First Plan (no obstacles, 8-connected diagonal)

| Map Size | Time (ms) | Nodes Expanded | Path Length |
|----------|----------|---------------|-------------|
| 100×100 | 0.118 | 98 | 98 |
| 200×200 | 0.260 | 198 | 198 |
| 400×400 | 0.602 | 398 | 398 |

## 2. Static Obstacles — First Plan (200×200)

| Obstacle % | Time (ms) | Nodes Expanded |
|-----------|----------|---------------|
| 10% | 6.10 | 18555 |
| 20% | 6.26 | 19248 |
| 30% | 7.03 | 21978 |

## 3. Incremental Replan — D* Lite Advantage (200×200)

After establishing baseline path, add N random obstacles and replan incrementally.

| Changes | Incremental (ms) | Full Replan (ms) | Speedup | Avg Nodes |
|---------|-----------------|-----------------|---------|-----------|
| 10 | 0.112 | 0.258 | 2.3× | 0.0 |
| 50 | 0.112 | 0.684 | 6.1× | 0.2 |
| 100 | 0.114 | 0.578 | 5.1× | 0.2 |

## 4. Scale Analysis

Open space first-plan time vs map size:

| Map Size | Cells | Time (ms) | ms per 10k cells |
|----------|-------|----------|-----------------|
| 100×100 | 10000 | 0.118 | 0.1175 |
| 200×200 | 40000 | 0.260 | 0.0649 |
| 400×400 | 160000 | 0.602 | 0.0376 |

## 5. Key Observations

1. **Incremental replan is fast**: Even with 100 obstacles changed, incremental time stays at ~0.11 ms, while full replan costs 0.58 ms — a **5× speedup**.
2. **Random obstacles barely affect D* Lite**: When obstacles are placed away from the optimal path, D* Lite expands ~0 nodes — it detects the path is still optimal and does nothing.
3. **Linear scaling**: 100→200→400 cells shows time scaling proportional to path length (diagonal), which is expected for D* Lite on open ground (it greedily follows the heuristic).
4. **Static obstacle density**: 10%→30% obstacles increases time from 6.1 to 7.0 ms (15% increase), as the search must explore around blocked cells.
