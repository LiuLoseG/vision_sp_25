# 用 rm_aim_sim 仿真器测试本仓库自瞄

[rm_aim_sim](https://github.com/ForgetWW520/rm_aim_sim) 是独立的 RoboMaster 自瞄闭环仿真器（Godot 渲染 + 共享内存 IPC）。本仓库通过 `build/standard_sim` 接入它：**检测、PnP、跟踪、瞄准、开火决策全部使用本仓库的 `auto_aim` 模块**，只把“相机输入”和“云台通信”换成共享内存。

## 一、准备

```bash
# 1. 仿真器（首次需要下载 Godot 4.7.2 并导入资源）
cd ~/rm_aim_sim
python3 tools/setup_godot.py
./run.sh --headless --editor --import --quit     # 只做一次

# 2. 本仓库
cd ~/vision_sp_25
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build -j8 --target standard_sim
```

若 GitHub 直连不通，可用镜像下载 Godot：
`curl -fL -o godot.zip https://ghproxy.net/https://github.com/godotengine/godot-builds/releases/download/4.7.2-stable/Godot_v4.7.2-stable_linux.x86_64.zip`

## 二、跑一次

两个终端，都从各自仓库根目录启动：

```bash
# 终端一：启动仿真器（脚本会加 --max-fps 120 --disable-vsync）
./scripts/run_sim.sh --config=$PWD/configs/sim/static.cfg

# 终端二：接入本仓库自瞄
./build/standard_sim configs/sim.yaml --duration=30
```

一键跑完整场景矩阵（自动起停仿真器）：

```bash
./scripts/sim_test.sh all 30          # 静止 / 平移 1m/s / 平移 2m/s / 自转
./scripts/sim_test.sh translate 40    # 只测一个场景
./build/standard_sim configs/sim.yaml --show   # 打开检测/瞄准窗口
```

`standard_sim` 选项：`--channel=<名>` 覆盖通道、`--no-fire` 只跟踪、`--duration=<秒>` 运行时长、`--show` 显示窗口。

## 三、场景配置

`configs/sim/*.cfg` 是仿真器侧的 ConfigFile 场景。为了让长时间统计不中断，都把
`[target] max_health` 设为 10000（这是仿真允许的上限；按 20 伤害/发，500 发命中就会
打爆，长时间扫描仍须重启，见 `tools/sim_sweep.py`）。

| 文件 | 场景 |
|---|---|
| `static.cfg` | 4 m 静止靶 |
| `translate_slow.cfg` | 平移 1 m/s、加速度 3 m/s² |
| `translate.cfg` | 平移 2 m/s、加速度 6 m/s²（半程 1.5 m 往返）|
| `spin.cfg` | 原地自转 4 rad/s |
| `combined.cfg` | **平移 1 m/s + 自转 4 rad/s**（最接近实战）|
| `combined_hard.cfg` | 平移 2 m/s + 自转 6 rad/s（极限工况）|

`configs/sim.yaml` 是本仓库侧配置，`sim:` 段是本接入特有的项：

| 键 | 含义 |
|---|---|
| `channel` | 共享内存通道名，需与仿真 `[ipc].channel` 一致 |
| `detector` | `traditional`（纯灯条几何）或 `yolo` |
| `aim_delay` | 图像采样 → 下发指令的额外延迟补偿（s）|
| `feedforward` | 是否下发目标角速度前馈 |
| `ff_horizon` | 计算前馈用的未来角差分步长（s）|
| `ff_acc` | 是否下发角加速度前馈（默认关闭）|
| `fire_acc_limit` | 角加速度超过该值时不开火，0 关闭 |
| `high_rate_control` | 是否在帧间持续下发（默认关闭）|

## 四、实现要点

### 4.1 时间域必须统一

仿真时间 `sim_time` 不是墙钟。`standard_sim` 用一个**严格线性**的映射把 `sim_time` 换成 `steady_clock::time_point`：

```text
t = anchor_wall + (sim_time - anchor_sim)
```

不要向墙钟“收敛”或重锚，否则仿真掉帧时 Tracker 会看到 0.5 s 的 dt 跳变并误判目标丢失。同时 `Aimer::aim(..., to_now=false)`：`to_now=true` 会用 `steady_clock::now()`，在仿真里属于跨时钟域预测。

### 4.2 云台前馈

仿真云台是二阶伺服（`response_hz=6`，即 τ≈26 ms）。只下发目标角时，对匀速/匀速旋转目标会一直滞后 $\frac{1}{\omega}\dot\theta$。对 2 m/s 平移目标，这只是“平移靶命中率 0%”。

因此 `standard_sim` 用 Aimer 副本预测两个未来瞄准角，差分出角速度和（可选）角加速度一并下发。实车云台下发链路同样支持这两个前馈量，这里的代码可以整体搬过去。

### 4.3 开火容差按装甲尺寸折算

`Shooter` 新增 `aim_tolerance_ratio`：把「装甲半宽 × 比例 / 距离」折算成角度容差，并与固定容差取较小者。

$$ \theta_{tol} = \min\left(\theta_{fixed},\ \arctan\frac{w/2 \cdot r}{d}\right) $$

这样近距离不会过松（打板出界），远距离不会过紧（漏打）。`0` 表示退回原来的固定容差行为。

**自转目标要用更严的容差**（`aim_tolerance_ratio_spin`，默认 0.4）。原因不是"自转更准"，
而是换板时云台要横向跨越整块装甲的间距（4 m 处约 1.9°）：此时"云台已对上上一条指令"
很容易成立，但云台实际停在两块板之间，打出去必然偏出装甲、落地。复合靶实测：

| `aim_tolerance_ratio_spin` | 出膛 | ground | 命中率 |
|---|---:|---:|---:|
| 0.35 | 142 | 30 | 74.5% |
| **0.40** | 178 | 22 | **82.5%** |
| 0.45 | 209 | 32 | 81.3% |

纯平移**不能**收紧：云台存在稳态滞后，收紧后 `|gimbal - last_command|` 几乎永不满足，
直接打不出弹（2 m/s 平移 62% → 26%）。所以按目标转速 `|wz| > aim_spin_speed`（默认 3 rad/s）
自动切换两套值。

### 4.4 每帧一次控制

实测在仿真里“每帧一次 + 角速度前馈”比帧间高频重发更稳：高频重发会让 `Shooter` 的“上一条指令与实际姿态已稳定”判定持续成立，开火时刻落在云台高速通过瞄准点的瞬间，命中率从 61% 掉到 41%。`high_rate_control` 保留为实验开关。

## 五、当前实测

4 m 距离、全仓库自瞄链路、每个场景 40 s（`./scripts/sim_test.sh all 40`）：

| 场景 | 出膛 | 命中率 |
|---|---:|---:|
| 静止靶 | 399 | **100.0%** |
| 平移 1 m/s | 339 | **84.8%** |
| 平移 2 m/s | 223 | **65.3%** |
| 自转 4 rad/s | 327 | **93.5%** |
| 复合：平移 1 m/s + 自转 4 rad/s | 196 | **80.9%** |
| 复合：平移 2 m/s + 自转 6 rad/s | 83 | **47.6%** |

口径为仿真器 `stats`：命中率 = `hit / resolved`，在飞弹丸不计入分母。

### 5.1 为什么 2 m/s 平移只有 ~65%

物理限制而非算法缺陷：4 m 处该速度对应约 28.6°/s 的方位角速度，而小装甲半宽只允许约
0.97° 误差，等效误差预算约 34 ms，仅云台伺服时常数（`response_hz=6` → τ≈26 ms）就占掉
大半。降到 1 m/s 即回到 84%。

### 5.2 复合靶的误差性质（用 `SP_SIM_ANALYZE=1` 实测）

把每发的**出膛角**与该发弹丸**到达目标平面时刻**的真实装甲方位比较（横向不受重力影响，
可直接比较角度）：

| 场景 | 命中时误差 | 打偏(ground)时误差 |
|---|---:|---:|
| 复合 1 m/s + 4 rad/s | mean 0.19° / max 0.56° | mean 1.56° / max 2.37° |
| 复合 2 m/s + 6 rad/s | mean 0.17° / max 0.49° | mean 1.79° / max 4.78° |

结论很明确：**不是"边缘抖动"，而是偶发的大偏差**（约 2.4~9 倍允许值）。量级与
`wz × 飞行时间 × wz 相对误差` 一致——6 rad/s 时飞行 0.17 s 要提前 59°，
`wz` 估偏 5% 就偏 3°。因此进一步提高复合靶命中率只能靠提高 `wz` 估计精度，
而 `ekf_omega_acc_var` 在 5~400 之间扫描对命中率没有显著影响（46.6%~53.8%，噪声内）。

## 六、调试

| 环境变量 | 作用 |
|---|---|
| `SP_SIM_EKF=1` | 每秒打印 EKF 中心/速度、指令角、云台角、真值角 |
| `SP_SIM_ANALYZE=1` | 结束时按 hit/ground/expired 分组打印**横向瞄准误差**统计（见 5.2）|
| `SP_SIM_DUMP=<dir>` | 每 N 帧落盘原始图像（`SP_SIM_DUMP_EVERY` 控制间隔）|
| `SP_DETECTOR_DEBUG=1` | 打印灯条/配对/分类的接受与拒绝原因 |

辅助脚本：

```bash
# 参数扫描：每组配置自动重启仿真器（靶车 500 发命中就会被打死，必须重置）
python3 tools/sim_sweep.py --scene=combined --duration=25 --rounds=3 \
    --set aim_tolerance_ratio_spin=0.35 \
    --set aim_tolerance_ratio_spin=0.40 \
    --set aim_tolerance_ratio_spin=0.45

# 读当前会话的 stats / events / 装甲真值
python3 tools/sim_impact_report.py
```

`SP_SIM_ANALYZE` 的实现要点：诊断比较的是「出膛角」与「弹丸**到达目标平面时刻**的装甲
方位角」。如果用出膛时刻的真值比较，会把合法的提前量算成误差（自转 4 rad/s 时 0.17 s
就转 40°）。

`sim_impact_report.py` 的弹着点分析只对**静止目标**可靠：仿真 `truth` 只保留当前时刻的
装甲位置，与弹丸出膛时刻不对应。

## 七、已知坑

- **必须加 `--disable-vsync`**。否则窗口失焦/被遮挡时 Godot 掉到约 1 Hz，状态发布和 `sim_time` 都远慢于实时，闭环结果无意义（表现为 `fired` 不增长、云台几乎不动）。`scripts/run_sim.sh` 已内置。
- **OpenVINO 默认 `device: GPU`** 在无独显/驱动的机器上会直接抛异常，`configs/sim.yaml` 用 `CPU`。
- **仿真器的装甲灯条有局部光晕**，二值化阈值太低会把光晕并进灯条，角点长度虚高 40%，PnP 距离随之偏大。`configs/sim.yaml` 用 `threshold: 250` + `lightbar_color_mask` 取灯条真身。
- **靶车被打死后灯条熄灭**，`camera.armors` 为空，此时 `armors=0` 属正常；持续测试请用 `configs/sim/*.cfg` 的 `max_health=10000`。
- **配置里的 `max_health` 只能在 `[-1, 10000]`**，且 10000 血量按 20 伤害/发也只够 500 发
  命中；长时间参数扫描必须定期重启仿真器（`tools/sim_sweep.py` 每轮都重启）。
- **降低相机分辨率能提高帧率但不划算**：1024×768 时检测帧率从 ~30 提升到 ~44 fps，
  但装甲角点像素变少、PnP 噪声增大，复合靶命中率反而从 81% 掉到 75%。
- `OPENCV_UI_BACKEND=QT5`、`QT_QPA_PLATFORMTHEME=""`、`QT_QPA_PLATFORM=xcb` 已在程序内设置。
  缺任一项在部分 GNOME 环境下 `--show` 会崩溃：
  `Settings schema 'org.gnome.settings-daemon.plugins.xsettings' does not contain a key named 'antialiasing'`。
