# PX4 四旋翼高度阶跃实验

本项目在 UTM Ubuntu 24.04 ARM64 中运行 PX4 v1.17.0，支持 Gazebo Harmonic 三维 X500 仿真和 SIH 数值仿真，从原始 ULog 还原串级控制响应。未改动 PX4 控制算法源码，未连接真实飞行器。

## 最终成果与 MPC 代码入口（2026-10-27）

- [离线成果说明：最终结论、中文图、代码阅读路线](deliverables/成果说明.html)
- [轻量成果包 ZIP](deliverables/PX4高度MPC成果包.zip) · [中文对比图](deliverables/最终效果对比.png) · [归档清单及校验值](deliverables/成果清单.json)
- [最终六次试验数据](results/20261027-flight-quality-pipeline/comparison.json) · [冻结代码与验证记录](results/20261027-flight-quality-pipeline/source-snapshot/verification.json)

最终方案是 **标定二状态模型 + 固定仿真时钟调度 + 稳定地面基准 + gentle 温和动作 + 尾差补偿关闭**。旧 MPC 流程一次试验的末段真实高度绝对误差约 15.4 cm；最终方案两次试验平均约 4.4 cm，真实峰值速度从约 1.74 降到 1.06 m/s。上升更慢、更平缓。样本有限，其中一次最终试验未持续保持在 ±5 cm；这些结果不能推出 MPC 全面优于 PID，也不代表实机验证完成。

**核心代码：[`scripts/altitude_mpc.py`](scripts/altitude_mpc.py)，共 136 行。** 代码结构：

| 位置 | 内容 |
| --- | --- |
| 第 36 行 `AltitudeMPC` | MPC 控制器类 |
| 第 47 行 | 高度、速度两个状态的离散运动模型 |
| 第 55 行 | 展开未来 30 步的预测矩阵 |
| 第 64 行 | 跟踪误差、输入及输入变化代价 |
| 第 75 行 | 输入、输入变化硬约束与速度软约束 |
| 第 89 行 `solve()` | 接收当前状态，更新并求解 QP，返回第一步加速度 |
| [`run_experiment.py`](scripts/run_experiment.py) 第 119 行 | 调用 MPC，经 MAVLink 给 PX4 发送加速度 |
| `run_experiment.py` 第 409、417 行 | 加载标定模型，覆盖 gentle 最终参数 |

这是新增的外部 Python MPC；PX4 继续承担姿态、角速度及电机控制。`MPCConfig()` 本身保留 standard 和未标定默认值，最终方案由运行入口加载和覆盖。PX4 的 `MPC_` 参数前缀代表多旋翼位置控制，不等于模型预测控制。

复跑最终方案（在已配置的 Mac＋UTM 环境）：

```bash
bash scripts/run-on-vm.sh gazebo mpc calibrated scheduled settled gentle off
```

成果包含代码、配置、中文报告和最终六次试验分析数据；不含 vendor、虚拟环境、UTM 磁盘和原始 ULog，不是完整环境备份。归档及报告可用 `python3 scripts/package_results.py` 重新生成，不会启动飞行。下方保留完整历史说明，项目优化过程另见 [RESULTS.md](RESULTS.md)。

## Gazebo 三维实验（已跑通）

2026-10-26 更新：默认 Ogre 2 已恢复三维画面、X500 模型和完整界面文字。当前可用配置是 Xorg 桌面 + GLX + llvmpipe 软件 OpenGL 4.5，使用系统原版 Ogre 2 资源。UTM 5.0.8 的 Apple Core OpenGL 后端可提供硬件 OpenGL 4.1，但硬件渲染路径仍存在着色器和显示兼容性问题，因此当前 GUI 由 CPU 渲染。

渲染修复时确认了模型可见、进程持续运行与暂停状态；2026-10-27 又在此配置下完成了完整 Gazebo 飞行及一维 MPC 实验。此前完整飞行记录仍保留，属于修复前的 Ogre 兼容模式。诊断日志位于 results/render-diagnosis。
在 Mac 终端运行：

    bash scripts/run-on-vm.sh gazebo

先启动 UTM 的 Linux 虚拟机并登录 Ubuntu 桌面。窗口显示四旋翼起飞、2 m 悬停、升至 4 m、降落；实验结束后 Gazebo 窗口保留。相同命令可重跑，每次重建本项目 Gazebo 场景并生成新的日志和响应曲线。

也可以在虚拟机终端运行：

    bash ~/px4-altitude-lab/scripts/run-gazebo.sh

环境使用现有 ROS 2 Jazzy 提供的 Gazebo Harmonic 8.15.0。启动脚本为 GUI 设置 `LIBGL_ALWAYS_SOFTWARE=1`、`QT_XCB_GL_INTEGRATION=xcb_glx` 和 `__EGL_VENDOR_LIBRARY_FILENAMES=/dev/null`，绕开 Ogre 可选 EGL 设备枚举时的 Mesa 崩溃。GUI 日志中的 EGL Headless 不可用提示可以预期，实际窗口使用 GLX。上述变量仅作用于 GUI；本实验不使用相机，启动配置省略未编译的可选视频推流插件。尚未验证相机渲染和所有编辑交互。

滚轮调整视距，鼠标拖动调整视角。只打开画面、不重新飞行时，在虚拟机运行：

    bash ~/px4-altitude-lab/scripts/run-gazebo.sh --view-only

此命令新建世界时保持暂停；若已有世界则保留其运行状态。正常实验命令会关闭本项目旧的 GUI 与服务器，重建场景后执行起飞、阶跃和降落。一次复用场景的试验出现了估计值与真值严重不符，尚未锁定上游根因，因此所有正式对比都从新场景开始。

### 渲染故障记录与回退

- 原 ANGLE 后端只提供 OpenGL 2.1，不满足 Ogre 2 要求。切至 Apple Core OpenGL 后提升至 4.1，但缺少 `GL_ARB_texture_buffer_range`，触发 Ogre 着色器回退分支中的 `texelFetch` 参数错误。
- `scripts/prepare-ogre2-resources.py` 可生成项目内实验性着色器补丁，消除该编译错误；硬件路径仍出现黑屏及文字异常，因此此补丁没有被默认启动脚本启用，也未修改系统 Gazebo 资源。
- 仅强制软件渲染仍在 EGL 设备枚举阶段崩溃；限制 GUI 使用 GLX 并禁用该进程的 EGL vendor 发现后，原版 Ogre 2 可正常显示。
- Ubuntu 桌面已从 Wayland 切至 Xorg：`/etc/gdm3/custom.conf` 中设置 `WaylandEnable=false`，原文件备份为 `/etc/gdm3/custom.conf.px4-before-xorg`。如需恢复原桌面配置，可还原此备份后重启；这会结束当前桌面会话，且需重新验证 GUI。

独立通信分区为 px4-altitude-lab，后台进程记录在 results/gazebo-session。关闭 GUI 后，后台物理仿真仍可能运行。在虚拟机中停止本项目后台进程可执行：

    bash ~/px4-altitude-lab/scripts/stop-gazebo.sh

首次完整 Gazebo 记录为 results/20261026-182627-gazebo-first。估计高度的 10–90% 上升时间 2.144 s，超调 0.0263 m，进入并保持 ±5 cm 误差带耗时 4.54 s，最终 2 秒平均估计高度 4.0234 m、真值 4.0187 m。该次日志确认降落停桨且无丢帧。它与此前 SIH 使用不同动力学及机型参数，不能把指标差异单独归因于渲染界面。

## 一维高度 MPC

新增 `scripts/altitude_mpc.py`：在外部 Python 程序中用 OSQP 求解凸二次规划。高度与速度来自 PX4 的 EKF，不使用仿真真值闭环。状态是向上为正的 `[高度 h, 速度 v]`，输入 `u` 是请求的净向上加速度，不需要再把重力加进 MAVLink 指令。原模型假设 `a=u`，校准模型用 `a=K*u+b`，其悬停平衡指令为 `u=-b/K`。

离散模型保持高度、速度两个状态：

    a_predicted[k] = K*u[k] + b
    h[k+1] = h[k] + dt*v[k] + 0.5*dt²*a_predicted[k]
    v[k+1] = v[k] + dt*a_predicted[k]

`--mpc-model nominal` 使用 K=1、b=0。Gazebo 默认 `--mpc-model calibrated`，从 config/calibrated-altitude-model.json 读取固定系数 K=1.549265、b=0.041090 m/s²。此配置只适用于本项目的 Gazebo X500，不用于 SIH 或真实飞机。SIH 默认仍使用 nominal。每次校准飞行会复制实际模型文件到结果目录，避免后续修改配置覆盖历史。

默认 dt=0.1 s、预测 30 步，即 3 s。每次重新求解未来 30 个加速度，只发送第一个，再根据新测量求解。实际调度依赖遥测和通信，更新间隔保存在 controller.csv 中，不能把它当作硬实时控制器。

standard 配置的代价包含高度误差平方（权重 12）、速度平方（3）、相对悬停平衡指令的输入平方（0.2）、相邻加速度变化平方（0.8），末端使用当前预测模型的离散 LQR 代价（校准后按新模型自动重算）。standard 配置的预测速度软限制为 ±1.5 m/s，允许通过非负 slack 放宽，并对 slack 平方施加权重 1000。加速度指令硬限制为 ±2 m/s²，每个名义 0.1 s 步的加速度变化不超过 0.5 m/s²（名义 jerk 5 m/s³）。这些限制作用于预测及指令，不保证实际运动也严格满足；没有设置高度硬约束。最初的速度硬约束版本因实际速度与模型不符而无解退出，相关失败记录保留；软约束使超出速度边界后仍可规划减速恢复。

起飞与降落沿用 PX4 原控制器。稳定在 2 m 后，MPC 接管竖直方向，经过 10 s 预观察后执行 4 m 阶跃，固定记录 30 s，再交回 PX4 降落。基线采用相同的预观察和记录长度。MPC 期间仍保留水平位置保持与偏航控制。

接口为 MAVLink LOCAL_NED：竖直位置与速度字段均被忽略，竖直加速度使用 `az=-a_up`；force 位不设置。这样原竖直 P/PI 反馈不参与，PX4 只处理后续加速度到推力转换、姿态、角速度和控制分配。分析程序检查 ULog 的 trajectory_setpoint 与 vehicle_local_position_setpoint，要求 z/vz 为 NaN、az 有限，防止把“加速度前馈叠加 PID”误报为 MPC 替代。

MPC 预测器本身没有积分或扰动观测器；可选的外部慢积分见后文，默认关闭。真实推力映射、执行延迟、EKF 修正和模型误差可能导致超调、振荡或残余高度误差。QP 失败、状态过期或更新停顿会终止本次实验并尝试交回 PX4 降落，不会静默切成 PID 并把结果记作 MPC 成功。

在已配置的虚拟机安装新增依赖：

    cd ~/px4-altitude-lab
    .venv/bin/pip install -r requirements-mpc.txt
    .venv/bin/python scripts/test_altitude_mpc.py

在 Mac 上运行完整 Gazebo MPC 实验并同步结果：

    bash scripts/run-on-vm.sh gazebo mpc

在虚拟机内分别运行基线和 MPC：

    bash ~/px4-altitude-lab/scripts/run-gazebo.sh --controller pid
    bash ~/px4-altitude-lab/scripts/run-gazebo.sh --controller mpc --mpc-model nominal
    bash ~/px4-altitude-lab/scripts/run-gazebo.sh --controller mpc --mpc-model calibrated

SIH 数值验证（当前 PX4 二进制链接了 Gazebo 库，也需要 ROS 的库路径）：

    source /opt/ros/jazzy/setup.bash
    cd ~/px4-altitude-lab
    .venv/bin/python scripts/run_experiment.py --simulator sih --controller mpc --mpc-model nominal --label sih-mpc
    .venv/bin/python scripts/analyze_latest.py

MPC 记录额外包含 controller.csv（状态、参考、加速度指令、求解时间、约束残差及速度 slack）和 mpc-predictions.jsonl（每次真实求解的整段预测）。response.csv 中的竖直速度目标保持 NaN，因为 MPC 没有给 PX4 发送该目标；高度目标来自外部 MPC 参考。

比较同一模拟器、PX4 版本及参数的两个已完成记录：

    .venv/bin/python scripts/compare_controllers.py results/<pid记录> results/<mpc记录> --output results/<对比目录>

输出 pid-vs-mpc.png / .pdf、mpc-predictions.png / .pdf 和 comparison.json。比较脚本核对模型、PX4 提交、导出的控制参数及 MPC 接口。两种控制方法的约束和整定不同，单次试验用于探索性对照，不能证明一种方法普遍更优。

### 校准和模型版本对比

校准脚本只用已完成的旧 MPC 飞行前 6 s 作为训练数据，固定系数后，用另一条 P/PI 飞行验证；拟合和评分都使用 EKF 反馈，仿真真值不进入辨识或控制。加速度映射在闭环数据上做最小二乘，不能视为无偏的物理参数辨识。非负延迟扫描仅用于诊断；零延迟拟合较好，不代表物理延迟不存在。

    .venv/bin/python scripts/identify_altitude_model.py results/20261027-001439-gazebo-mpc results/20261027-001613-gazebo-pid --output results/20261027-model-calibration --model-output config/calibrated-altitude-model.json

在独立验证的动态样本上，加速度 RMSE 从 0.527 降至 0.309 m/s²；在 0.2 s 速度增量窗口上，RMSE 从 0.0430 降至 0.0136 m/s。未在新飞行结果上继续重拟合。新实验保持相同代价权重、预测窗口和约束，每种模型各跑两次；所有正式记录均保留，状态估计和更新间隔的试验间差异也必须纳入结果解读。

    .venv/bin/python scripts/compare_mpc_models.py results/<原模型记录> results/<校准模型记录> --nominal-repeat results/<原模型复跑> --calibrated-repeat results/<校准模型复跑> --output results/<模型对比目录>

### 时序与两类误差

新增 scripts/audit_timing_errors.py，分别计算：

    跟踪误差 = 目标高度 - EKF 估计高度
    估计误差 = EKF 估计高度 - 仿真真值高度
    物理误差 = 跟踪误差 + 估计误差

所有高度均沿用固定的起飞前地面基准。真值只在飞行结束后参与诊断，未用于 MPC 或补偿器。某次旧飞行末段的跟踪误差仅约 -3 cm，估计误差却约 +24 cm，物理误差约 +21 cm；控制积分不能据此直接消除估计偏差。时序优化阶段未加入积分或扰动观测器，先保留并区分这些误差；后续外部慢积分的测试见下节。

默认 `--timing-mode scheduled` 优先清空接收队列，按仿真时间的固定相位执行名义 10 Hz MPC；每次求解结束立即发出指令，之后才写诊断文件。位置遥测请求从 20 Hz 提至 50 Hz。重复设定值与 GCS 心跳仍按墙上时间独立发送。过载时跳过已错过的节拍，不会连续补算一串过时指令。

初版 scheduled 的间隔出现 80/120 ms 交替：最大间隔改善，但 95 分位反而变差，因此不能只看最大值。默认版本 2 允许使用比计划时刻早最多 4 ms 的新反馈，避免因采样量化而等待下一帧；计划相位保持不变。日志保留有正有负的 `deadline_phase_error_sim_ms`，不会把提前计算伪装成零误差。容差和版本保存在 metadata.json。名义模型步长仍为 0.1 s，不能据此宣称硬实时。

    bash ~/px4-altitude-lab/scripts/run-gazebo.sh --controller mpc --mpc-model calibrated --timing-mode legacy
    bash ~/px4-altitude-lab/scripts/run-gazebo.sh --controller mpc --mpc-model calibrated --timing-mode scheduled

Mac 包装器也接受第四个参数：`run-on-vm.sh gazebo mpc calibrated scheduled`。要复现此前的模型校准对比，应使用 legacy；新的模型比较工具会拒绝把不同调度/反馈频率混为“只改模型”。

controller.csv 新增计划节拍、相位误差、跳过节拍数、接收后状态年龄、求解开始与命令发送墙上时刻。`state_receive_age_wall_ms` 只是本进程收到消息之后的年龄，不含网络传输和估计器内部延迟；ULog 的 timestamp-timestamp_sample 也不等于网络延迟。历史日志无法恢复未记录的精确发送/接收时刻。

    .venv/bin/python scripts/audit_timing_errors.py results/<记录1> results/<记录2> --output results/<误差审计目录>
    .venv/bin/python scripts/compare_timing.py --legacy results/<旧调度记录> --scheduled results/<新调度1> results/<新调度2> --output results/<时序对比目录>

response.csv 新增三种高度误差列。时序比较会核对固定模型、整定与导出的 PX4 参数，输出更新间隔分布和三种误差的独立曲线。调度测试覆盖固定相位、采样容差、过载不补算和时钟倒退；假连接测试验证先接收新状态、再求解、再发出正确符号的加速度指令。

### 默认飞行质量配置：稳定地面基准与温和动作

2026-10-27 的顺序优化已经完成。Gazebo MPC 入口默认 `--ground-init settled --motion-profile gentle --bias-compensation off`。未改 EKF 参数、未降低传感器噪声、未添加真值反馈；改动是先等待估计收敛，再记录起飞地面零点。

settled 地面准备至少等待 40 s 仿真时间，检查最近 10 s 的估计高度标准差 ≤2.5 cm、漂移斜率 ≤2.5 mm/s、竖直速度均值绝对值 ≤1.5 cm/s、速度 RMS ≤3.5 cm/s，并连续通过 3 s 后才标定最后 3 s 的地面中位数。超过 150 s 仿真时间或 190 s 墙上时间仍不满足条件则不启动飞行。本次实际准备耗时约 43–120 s；比原来固定等待 15 s 更慢。legacy 保留原流程。

gentle 动作只改 MPC 的运动限制：预测速度软限制 ±1 m/s，加速度指令硬限制 ±0.8 m/s²，名义 jerk 2 m/s³；standard 为 ±1.5、±2、5。实际运动仍可能超出软限制或指令数值。起飞/降落仍使用原 PX4 控制器，温和动作主要用于接管后的高度阶跃。

最后一步实现了一个独立的、小幅慢积分参考修正：只在高度误差 <15 cm、速度 <15 cm/s、输入未饱和时积分；增益 0.15/s，参考偏移上限 ±12 cm，目标变化时清零。它位于 MPC 外部，不是状态估计偏差补偿，也不是完整 offset-free MPC。controller.csv 和 mpc-predictions.jsonl 明确区分原高度目标、修正量及优化器实际参考。仿真开/关各验证两次，未显示一致的末段真实高度精度收益，因此默认关闭，保留 `--bias-compensation integral` 作为可选实验。

    bash ~/px4-altitude-lab/scripts/run-gazebo.sh --controller mpc --ground-init legacy --motion-profile standard --bias-compensation off
    bash ~/px4-altitude-lab/scripts/run-gazebo.sh --controller mpc --ground-init settled --motion-profile standard --bias-compensation off
    bash ~/px4-altitude-lab/scripts/run-gazebo.sh --controller mpc --ground-init settled --motion-profile gentle --bias-compensation off
    bash ~/px4-altitude-lab/scripts/run-gazebo.sh --controller mpc --ground-init settled --motion-profile gentle --bias-compensation integral

Mac 包装器第五到第七个可选参数对应上述三个选项；例如 `run-on-vm.sh gazebo mpc calibrated scheduled settled gentle off`。省略时 Gazebo 使用 settled/gentle/off，SIH 地面准备默认 legacy。

对照分析 scripts/compare_flight_pipeline.py 使用原始真值 CSV 单独计算物理误差、超调、速度与 ±5 cm 稳定时间，并核对所有试验 EKF 参数一致。所有真值只在飞行结束后使用。相比一个旧流程对照，温和模式两次平均末段真实高度绝对误差从约 15.4 cm 降为 4.4 cm，真实速度峰值从约 1.74 降至 1.06 m/s；代价是上升和准备过程更慢。这是小样本仿真结果，并没有实现每次全程保持 ±5 cm。

新增检查：地面等待条件、积分限幅/门控/清零、相同未建模扰动下积分减少残差，以及修改后遥测接收至加速度发送的行为测试。原始对照和积分候选全部保留在 results/20261027-flight-quality-pipeline 对照报告中。

## 实验流程

1. 启动独立仿真实例，按 settled 条件等待状态估计稳定（legacy 为固定等待）。
2. 起飞前采集最后 3 秒的局部高度，以中位数确定地面基准。局部 NED 的 z=0 不一定等于起飞地面。
3. 通过 Offboard 位置目标起飞到相对基准 2 m，确认高度误差与垂直速度均小于 0.1 并持续稳定。
4. 再观察 10 秒仿真时间；MPC 在此期间接管。随后将目标高度阶跃至 4 m。
5. 记录固定 30 秒仿真时间，保留实际响应，不以最后一个采样点判断实验成败。
6. 正常降落并确认自动解除电机解锁，关闭本次仿真实例。

SIH 模式用 PX4 内置动力学；Gazebo 模式用外部物理引擎和传感器模型。两种模式都运行真实 PX4 状态估计、控制器及控制分配程序。

## 四张曲线

每次实验生成 altitude-response.png / .pdf，依次显示：

- 高度目标、EKF 估计高度、仿真动力学真值。
- 位置环产生的内部速度目标和估计速度。
- 速度环产生的内部加速度目标和估计加速度。
- NED 坐标系中的向上归一化推力指令，单位不是牛顿。

估计高度和真值高度分别扣除各自起飞前的地面值，避免混淆坐标原点的固定偏移。此基准在整次飞行中保持不变，后续估计误差仍会显示。仿真真值只用于事后比较，未替代 EKF 输入控制器。

图中向上为正：高度 = 起飞前地面 z − 当前 z；速度与加速度取负。原始 vehicle_*.csv 保留 NED 坐标，response.csv 使用绘图坐标。

## 对照源码

固定版本 v1.17.0，提交 d6f12ad1c4f70ad3230afd7d86e971421e02fef4。

首先阅读 vendor/PX4-Autopilot/src/modules/mc_pos_control/PositionControl/PositionControl.cpp 中的 _positionControl()、_velocityControl()、_accelerationControl()。

本实验没有速度前馈，竖直位置环近似为：

    向上速度目标 = MPC_Z_P × (目标高度 − 估计高度)

随后进行速度限幅。速度环根据速度误差产生加速度目标，包含积分、微分和饱和处理。默认竖直增益为 MPC_Z_P=1、MPC_Z_VEL_P_ACC=4、MPC_Z_VEL_I_ACC=2、MPC_Z_VEL_D_ACC=0，因此默认竖直速度环 D 项关闭。以每次导出的 parameters.json 为准。

本次重点是“位置 P → 速度 PI → 加速度到推力转换”。姿态和角速度控制仍运行，但没有主动给姿态阶跃。

## 在 Mac 上重跑

先启动 UTM 的 Linux 虚拟机，然后在终端执行：

    bash scripts/run-on-vm.sh

按 SSH 提示输入虚拟机密码。脚本不保存密码；传送实验脚本，在虚拟机内运行和分析，然后把结果复制回 Mac。通过 PX4_LAB_VM 指定虚拟机的 SSH 目标；脚本中保留了原实验环境的默认地址。

## 在虚拟机内重跑

    cd ~/px4-altitude-lab
    .venv/bin/python scripts/run_experiment.py --label baseline
    .venv/bin/python scripts/analyze_latest.py

只连接本机 127.0.0.1，使用实例 4、MAVLink 系统编号 5、UDP 14544。不要同时运行两个本项目实验。每次使用独立参数与运行目录。

编译命令：

    cd ~/px4-altitude-lab/vendor/PX4-Autopilot
    . ../../.venv/bin/activate
    make -j4 px4_sitl_default

## 结果目录

results/<时间>-<标签>/ 中包含：

- metadata.json：版本、模型、地面基准、运行状态。
- events.json：起飞、阶跃、降落时间。
- runtime/log/**/*.ulg：PX4 原始日志。
- vehicle_*.csv：原始内部消息。
- response.csv：统一到 20 Hz 的绘图数据。
- altitude-response.png / .pdf：四层响应曲线。
- metrics.json：上升时间、超调、稳定时间、最终误差及日志丢包统计。
- parameters.json：实际控制与仿真参数。

“运行完成”表示记录、降落及解除解锁完成，不表示跟踪误差为零。±5 cm 稳定时间为空，表示未在观察窗口内持续保持该误差带，或剩余验证时间不足 2 秒。指标用原始样本计算，绘图时才插值。

早期调试数据也保留：baseline 因单点误差判据未通过而标为失败；baseline-30s 完成局部坐标实验。首个正式地面相对高度实验标签为 ground-relative。新的分析脚本需要地面基准事件，不能用于两个早期记录。

## 响应曲线分析

先找到高度阶跃瞬间，看高度误差如何产生速度目标；再看推力为什么先增加、随后降低、最后回到悬停附近。对照估计值与真值，区分跟踪误差和状态估计误差。

## 官方参考

- [固定版本源码](https://github.com/PX4/PX4-Autopilot/tree/v1.17.0)
- [控制框图](https://docs.px4.io/main/en/flight_stack/controller_diagrams)
- [SIH 仿真](https://docs.px4.io/main/en/sim_sih/)
- [Offboard 接口](https://docs.px4.io/main/en/flight_modes/offboard)
