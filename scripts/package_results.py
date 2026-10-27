#!/usr/bin/env python3
"""Build an offline Chinese report and a lightweight, auditable results archive."""
from pathlib import Path
import hashlib
import html
import json
import zipfile

ROOT = Path(__file__).resolve().parents[1]
OUT = ROOT / 'deliverables'
PIPE = ROOT / 'results/20261027-flight-quality-pipeline'


def main():
    OUT.mkdir(exist_ok=True)
    report = json.loads((PIPE / 'comparison.json').read_text())
    trials = report['trials']
    final = [t for t in trials if t['ground_init'] == 'settled'
             and t['motion_profile'] == 'gentle' and t['bias_compensation'] == 'off']
    assert len(trials) == 6 and len(final) == 2
    baseline = trials[0]['physical']
    mean_error = sum(abs(t['physical']['final_mean_error_m']) for t in final) / len(final)
    mean_speed = sum(t['physical']['peak_abs_velocity_m_s'] for t in final) / len(final)
    rows = ''
    for i, t in enumerate(trials, 1):
        p = t['physical']
        label = ('旧流程' if t['ground_init'] == 'legacy' else
                 '稳定地面基准' if t['motion_profile'] == 'standard' else
                 '稳定基准＋温和动作' if t['bias_compensation'] == 'off' else
                 '稳定基准＋温和动作＋尾差补偿')
        rows += (f'<tr><td>{i}：{label}</td><td>{abs(p["final_mean_error_m"])*100:.2f}</td>'
                 f'<td>{p["peak_abs_velocity_m_s"]:.3f}</td><td>{p["overshoot_m"]*100:.2f}</td>'
                 f'<td>{"是" if p["settling_time_5cm_s"] is not None else "否"}</td></tr>')
    command = "bash scripts/run-on-vm.sh gazebo mpc calibrated scheduled settled gentle off"
    vm_command = 'bash ~/px4-altitude-lab/scripts/run-gazebo.sh --controller mpc --mpc-model calibrated --timing-mode scheduled --ground-init settled --motion-profile gentle --bias-compensation off'
    page = f'''<!doctype html><html lang="zh-CN"><meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1"><title>PX4 一维高度 MPC · 项目成果</title>
<style>body{{font:17px/1.8 -apple-system,BlinkMacSystemFont,"PingFang SC",sans-serif;color:#213040;background:#f5f7fa;margin:0}}main{{max-width:1060px;margin:auto;padding:35px 24px 80px}}section{{background:white;border-radius:12px;padding:20px 28px;margin:20px 0}}h1,h2{{line-height:1.4}}h2{{font-size:24px}}a{{color:#1462a5}}table{{border-collapse:collapse;width:100%;font-size:15px}}td,th{{padding:10px;border-bottom:1px solid #ddd;text-align:left}}pre{{white-space:pre-wrap;overflow-wrap:anywhere;background:#edf2f7;padding:18px;border-radius:8px;font-size:14px}}img{{width:100%;height:auto}}.muted{{color:#5b6875}}.kpi{{font-size:22px;color:#145d87}}@media print{{body{{background:white}}section{{padding:0}}}}</style>
<main><h1>PX4 四旋翼一维高度 MPC：建模、约束与仿真实验</h1>
<p class="muted">成果归档 · 2026-10-27 · Gazebo X500 · 高度从 2 米升至 4 米</p>
<section><h2>最终结论</h2>
<p>本实验在 PX4 SITL 与 Gazebo X500 上实现外部高度 MPC，完成模型标定、固定周期调度、地面基准稳定检查和动作约束优化，并通过六次顺序试验评价响应。</p>
<p class="kpi">末段真实高度平均绝对误差：约 {abs(baseline['final_mean_error_m'])*100:.1f} 厘米 → {mean_error*100:.1f} 厘米<br>真实峰值速度：约 {baseline['peak_abs_velocity_m_s']:.2f} → {mean_speed:.2f} 米/秒</p>
<p>左侧是旧 MPC 流程一次试验，右侧是最终方案两次试验的均值。动作更平缓，代价是上升变慢。尾差补偿试验没有表现出稳定收益，最终默认关闭。这里比较的是 MPC 工作流程优化前后，不能据此得出 MPC 全面优于 PID。</p>
<p>仍有估计偏差和试次波动。最终方案只有两次重复，其中一次未达到“进入 ±5 厘米后在剩余观察窗口内持续保持”的条件。当前结果构成可复查的仿真实验证据，尚未完成实机、多工况和工业级可靠性验证。</p></section>
<section><h2>六次试验的证据</h2><p>高度误差和峰值速度均来自仿真真值；真值只用于飞行结束后的评价，从未反馈给控制器。末段误差是末段平均高度相对 4 米的绝对偏差，不能当成整段最大误差。</p>
<table><thead><tr><th>方案</th><th>末段误差 / 厘米</th><th>峰值速度 / 米每秒</th><th>超调 / 厘米</th><th>持续进入 ±5厘米</th></tr></thead><tbody>{rows}</tbody></table>
<p><a href="../results/20261027-flight-quality-pipeline/comparison.json">完整指标与每次试验目录</a> · <a href="../results/20261027-flight-quality-pipeline/source-snapshot/verification.json">已完成的验证记录</a></p>
<img src="最终效果对比.png" alt="六次高度变化试验的中文对比图"><p>第一幅看真实高度；第二幅看飞控估计的上升速度；第三幅看飞控估计高度。第一幅与第三幅的差距体现状态估计偏差。同色实线和虚线是两次重复试验。灰带是目标 ±5 厘米。</p><a href="最终效果对比.pdf">下载对比图 PDF</a></section>
<section><h2>MPC 实现与飞控接口</h2>
<p><strong><a href="../scripts/altitude_mpc.py">scripts/altitude_mpc.py</a></strong> 只有 136 行，是核心。<code>AltitudeMPC</code> 从第 36 行开始；重点读第 89 行的 <code>solve(height, velocity, target)</code>，它接收高度、速度和目标，返回当前应执行的第一步加速度。</p>
<table><tr><th>实现模块</th><th>位置</th><th>解决的问题</th></tr>
<tr><td>1. 建模</td><td>altitude_mpc.py:47</td><td>用高度、速度两个状态预测未来运动</td></tr>
<tr><td>2. 展开未来轨迹</td><td>altitude_mpc.py:55</td><td>将未来 30 步写成矩阵</td></tr>
<tr><td>3. 代价函数</td><td>altitude_mpc.py:64</td><td>兼顾接近目标、速度小、控制输入小、指令平滑和末端状态</td></tr>
<tr><td>4. 约束</td><td>altitude_mpc.py:75</td><td>限制输入加速度、输入变化和预测速度</td></tr>
<tr><td>5. 每次求解</td><td>altitude_mpc.py:89</td><td>更新状态和目标，调用 OSQP，检查结果，取第一步</td></tr>
<tr><td>6. 接入飞控</td><td><a href="../scripts/run_experiment.py">run_experiment.py:119</a></td><td>调用 solve，将向上加速度转换为 NED 向下坐标的 az 后发送给 PX4</td></tr>
<tr><td>7. 最终参数</td><td>run_experiment.py:409、417</td><td>读标定模型，覆盖为 gentle 温和参数</td></tr>
<tr><td>8. 配套优化</td><td><a href="../scripts/control_clock.py">control_clock.py</a>、<a href="../scripts/flight_quality.py">flight_quality.py</a></td><td>固定仿真时钟调度、地面稳定检查和可选尾差补偿</td></tr></table>
<p>本实验在 PX4 外实现 Python MPC。它在高度变化阶段替代垂直位置与速度控制，PX4 继续执行加速度到推力、姿态、角速度及电机控制，并负责起飞和降落。横向位置与航向仍由 PX4 保持。PX4 参数名中的 <code>MPC_</code> 指多旋翼位置控制，不代表这些参数属于模型预测控制。</p></section>
<section><h2>模型与最终参数</h2><pre>状态：x = [高度 h, 向上速度 v]；输入：请求的净向上加速度 u
模型：a = K·u + b
h_next = h + dt·v + 0.5·dt²·(K·u+b)
v_next = v + dt·(K·u+b)
K = 1.5492651348831779；b = 0.041089602835091
每 0.1 秒求解一次，预测 30 步（未来 3 秒），每次仅执行第一步。</pre>
<p>这是高度的二阶运动学模型，有两个状态。标定参数见 <a href="../config/calibrated-altitude-model.json">calibrated-altitude-model.json</a>。</p>
<ul><li>最终 gentle：预测速度软限制 1 米/秒；请求加速度硬限制 ±0.8 米/秒²；相邻请求变化按 2 米/秒³ × 0.1 秒约束。</li><li>速度软约束允许付出惩罚后超出；模型误差、时序和执行器动态也会影响实际运动，所以这些数值不保证实际飞行严格不越界。</li><li>代价权重：高度 12、速度 3、输入 0.2、输入变化 0.8、速度松弛 1000；末端使用离散 LQR 代价。</li><li>起飞前至少等 40 秒仿真时间，并检查最近 10 秒地面高度与速度稳定，再连续通过 3 秒检查。使用末段地面观测中位数建立高度基准；超时拒绝起飞。</li><li>尾差补偿可选 integral，本次默认 off。它是慢速、限幅的参考修正，不是完整的无静差 MPC，不能修正飞控自身的高度估计偏差。</li></ul>
<p>注意：<code>MPCConfig()</code> 类本身保留 standard 参数及未标定模型。复现最终效果请使用下面完整入口，由实验脚本加载标定模型并覆盖 gentle 参数；单独实例化默认类不会自动得到最终配置。</p></section>
<section><h2>如何复跑</h2><p>Mac 仓库根目录（先启动并登录已配置好的 UTM Ubuntu，通过 PX4_LAB_VM 环境变量指定 SSH 目标）：</p><pre>{html.escape(command)}</pre><p>或在已配置的 Ubuntu 虚拟机中：</p><pre>{html.escape(vm_command)}</pre>
<p>命令会重建本项目仿真世界，执行起飞、2→4 米高度变化与降落，生成新的日志和图表。环境为 Ubuntu 24.04 ARM64、PX4 v1.17.0、Gazebo Harmonic 8.15；GUI 使用已有的软件渲染配置。</p>
<p>只重新生成本成果包，不飞行：<code>python3 scripts/package_results.py</code>。重新生成中文图需要已安装数值分析依赖和中文字体的环境：<code>python scripts/render_final_chart_zh.py</code>。</p></section>
<section><h2>成果清单与复查入口</h2><ul><li><a href="../README.md">README：项目入口与运行说明</a></li><li><a href="../RESULTS.md">RESULTS：完整实验过程、历史方案和边界</a></li><li><a href="../results/20261027-flight-quality-pipeline/comparison.json">最终六次试验汇总</a>与 <a href="../results/20261027-flight-quality-pipeline/source-snapshot/verification.json">代码冻结及验证记录</a></li><li><a href="成果清单.json">归档文件与 SHA-256 校验值</a></li></ul>
<p>轻量成果包包含核心及配套代码、配置、说明、中文图、最终六次试验的分析 CSV/JSON 和代码快照。它不包含 PX4 源码 vendor、Python 虚拟环境、UTM 磁盘、原始 ULog、运行缓存或控制台日志，不能当作完整环境备份。原始日志保留在原项目中。部分历史文档链接指向未打包的早期试验资料，请在原项目中查阅。</p></section></main></html>'''
    (OUT / '成果说明.html').write_text(page)
    chosen = set()
    def add(path):
        if path.is_file() and not path.is_symlink():
            chosen.add(path.relative_to(ROOT))
    for name in ('README.md', 'RESULTS.md', 'EXPERIMENT_NOTES.md', 'requirements.txt', 'requirements-mpc.txt', '.gitignore'):
        add(ROOT / name)
    for folder in ('scripts', 'config'):
        for path in (ROOT / folder).rglob('*'):
            if '__pycache__' not in path.parts and path.suffix in {'.py', '.sh', '.json', '.config'}:
                add(path)
    for path in PIPE.rglob('*'):
        if path.suffix in {'.json', '.py', '.sh', '.png', '.pdf'}:
            add(path)
    for trial in trials:
        folder = ROOT / 'results' / trial['run']
        for path in folder.iterdir():
            if path.suffix in {'.json', '.csv', '.jsonl'}:
                add(path)
    for name in ('成果说明.html', '最终效果对比.png', '最终效果对比.pdf'):
        path = OUT / name
        if not path.exists():
            raise FileNotFoundError(path)
        add(path)
    verification = json.loads((PIPE / 'source-snapshot/verification.json').read_text())
    for name, digest in verification['source_sha256'].items():
        if hashlib.sha256((ROOT / name).read_bytes()).hexdigest() != digest:
            raise RuntimeError(f'Flight source changed since verified snapshot: {name}')
    manifest = {'date': '2026-10-27', 'purpose': '轻量仿真实验归档，非完整环境备份',
                'selected_defaults': report['selected_defaults'],
                'files': {str(p): {'bytes': (ROOT / p).stat().st_size,
                                  'sha256': hashlib.sha256((ROOT / p).read_bytes()).hexdigest()}
                          for p in sorted(chosen)}}
    manifest_path = OUT / '成果清单.json'
    manifest_path.write_text(json.dumps(manifest, ensure_ascii=False, indent=2) + '\n')
    add(manifest_path)
    archive = OUT / 'PX4高度MPC成果包.zip'
    with zipfile.ZipFile(archive, 'w', compression=zipfile.ZIP_DEFLATED) as package:
        for p in sorted(chosen):
            package.write(ROOT / p, Path('px4-altitude-lab') / p)
    with zipfile.ZipFile(archive) as package:
        assert package.testzip() is None
    print(f'{len(chosen)} files packaged: {archive} ({archive.stat().st_size / 1024 / 1024:.2f} MiB)')


if __name__ == '__main__':
    main()
