#!/usr/bin/env python3
"""参数扫描：每组配置重启仿真器后跑 standard_sim，用 stats 差量算命中率。

为什么要重启仿真器：仿真的靶车血量上限是 10000，按 20 伤害/发只需 500 发命中
就会被打死，之后灯条熄灭、`camera.armors` 为空，无法再检测也无法继续计分。

用法：
    python3 tools/sim_sweep.py --scene=combined --duration=30 --rounds=2 \
        --set aim_delay=0.02 --set aim_delay=0.04
"""

import argparse
import json
import os
import re
import shutil
import signal
import subprocess
import time

SIM_DIR = os.environ.get("RM_AIM_SIM_DIR", os.path.expanduser("~/rm_aim_sim"))
STATE = "/dev/shm/rm_aim_sim_default/state.json"
SHM = "/dev/shm/rm_aim_sim_default"


def read_state():
    with open(STATE) as handle:
        return json.load(handle)


def patch_yaml(path, key, value):
    with open(path) as handle:
        text = handle.read()
    # 顶层键（无缩进）与 sim: 段内的子键都要能改
    pattern = re.compile(r"^([ \t]*%s:[ \t]*).*$" % re.escape(key), re.MULTILINE)
    if pattern.search(text):
        text = pattern.sub(lambda m: m.group(1) + str(value), text, count=1)
    else:
        anchor = re.search(r"^([ \t]+channel:[ \t]*.*)$", text, re.MULTILINE)
        if not anchor:
            raise SystemExit("找不到插入位置: %s" % key)
        line_end = text.index("\n", anchor.end())
        text = text[: line_end + 1] + "  %s: %s\n" % (key, value) + text[line_end + 1 :]
    with open(path, "w") as handle:
        handle.write(text)


class Simulator:
    """启动/停止 rm_aim_sim，并等待共享内存通道就绪。"""

    def __init__(self, scene, warmup=9.0):
        self.scene = scene
        self.warmup = warmup
        self.process = None
        self.log = None

    def start(self):
        self.stop()
        shutil.rmtree(SHM, ignore_errors=True)
        cmd = [
            os.path.join(SIM_DIR, "run.sh"), "--max-fps", "120", "--disable-vsync", "--",
            "--remote", "--config=" + self.scene, "--quit-after-seconds=1800",
        ]
        env = dict(os.environ)
        env.setdefault("OPENCV_UI_BACKEND", "QT5")
        env.setdefault("QT_QPA_PLATFORMTHEME", "")
        self.log = open("/tmp/sim_sweep_scene.log", "w")
        self.process = subprocess.Popen(
            cmd, cwd=SIM_DIR, env=env, stdout=self.log, stderr=subprocess.STDOUT,
            start_new_session=True)
        for _ in range(120):
            if os.path.exists(STATE):
                break
            time.sleep(0.5)
        else:
            raise SystemExit("仿真器未能在 60s 内建立 IPC 通道")
        time.sleep(self.warmup)  # 等云台归位与场景稳定

    def stop(self):
        if self.process is not None:
            try:
                os.killpg(os.getpgid(self.process.pid), signal.SIGTERM)
                self.process.wait(timeout=10)
            except (OSError, subprocess.TimeoutExpired):
                pass
            self.process = None
        if self.log is not None:
            self.log.close()
            self.log = None
        os.system("pkill -f 'godot --path %s' >/dev/null 2>&1" % SIM_DIR)
        time.sleep(0.5)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--scene", default="combined")
    parser.add_argument("--duration", type=float, default=30.0)
    parser.add_argument("--binary", default="./build/standard_sim")
    parser.add_argument("--config", default="configs/sim.yaml")
    parser.add_argument("--rounds", type=int, default=1)
    parser.add_argument("--set", action="append", default=[], metavar="KEY=VALUE")
    args, extra = parser.parse_known_args()

    scene_path = os.path.abspath(os.path.join("configs", "sim", args.scene + ".cfg"))
    if not os.path.exists(scene_path):
        raise SystemExit("找不到场景配置: " + scene_path)

    settings = []
    for item in args.set:
        key, _, value = item.partition("=")
        settings.append((key.strip(), value.strip()))

    shutil.copyfile(args.config, args.config + ".sweep.bak")
    simulator = Simulator(scene_path)
    try:
        for index in range(args.rounds):
            if settings:
                key, value = settings[index % len(settings)]
                patch_yaml(args.config, key, value)
                label = "%s=%s" % (key, value)
            else:
                label = "(当前配置)"
            simulator.start()

            before = read_state()["stats"]
            started = time.time()
            subprocess.run(
                [args.binary, args.config, "--duration=%g" % args.duration] + extra,
                stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, check=False)
            after = read_state()["stats"]
            fired = after["fired"] - before["fired"]
            hit = after["hit"] - before["hit"]
            resolved = after["resolved"] - before["resolved"]
            ground = after["ground"] - before["ground"]
            expired = after["expired"] - before["expired"]
            rate = (100.0 * hit / resolved) if resolved else 0.0
            print(
                "%-28s fired=%3d hit=%3d resolved=%3d ground=%2d expired=%2d rate=%5.1f%% (%.0fs)"
                % (label, fired, hit, resolved, ground, expired, rate,
                   time.time() - started),
                flush=True)
    finally:
        simulator.stop()
        shutil.move(args.config + ".sweep.bak", args.config)


if __name__ == "__main__":
    main()
