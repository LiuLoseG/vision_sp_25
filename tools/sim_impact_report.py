#!/usr/bin/env python3
"""分析 rm_aim_sim 会话的弹着点分布，用于闭环测试的偏差定位。

只读取 state.json 里仿真器自带的诊断数据（stats/events/truth），不参与控制决策。

用法：
    python3 tools/sim_impact_report.py                 # 读默认通道的当前状态
    python3 tools/sim_impact_report.py --channel=foo
"""

import argparse
import json
import math
import sys
from collections import Counter


def load_state(channel):
    path = "/dev/shm/rm_aim_sim_%s/state.json" % channel
    try:
        with open(path) as handle:
            return json.load(handle)
    except OSError as error:
        sys.exit("无法读取 %s: %s" % (path, error))


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--channel", default="default")
    args = parser.parse_args()

    state = load_state(args.channel)
    stats = state.get("stats", {})
    print("会话:", state.get("session_id"), " sim_time: %.2f" % state.get("sim_time", 0))
    print("统计:", json.dumps(stats, ensure_ascii=False))

    events = state.get("events", [])
    shots = [e for e in events if e.get("type") == "shot"]
    impacts = [e for e in events if e.get("type") == "impact"]
    kinds = Counter(e.get("kind") for e in impacts)
    print("事件环: shot=%d impact=%d %s" % (len(shots), len(impacts), dict(kinds)))

    truth = state.get("truth", {})
    armors = truth.get("armors", [])
    if impacts and armors:
        print("\n最近 %d 个弹着点相对最近装甲中心的偏差（mm，+横=右，+高=上）:" % len(impacts))
        for event in impacts[-20:]:
            point = event.get("point")
            if not point:
                continue
            px, py, pz = point
            best, best_d = None, 1e9
            for armor in armors:
                cx, cy, cz = armor["center"]
                d = math.dist((px, py, pz), (cx, cy, cz))
                if d < best_d:
                    best, best_d = armor, d
            if best is None:
                continue
            cx, cy, cz = best["center"]
            normal = best["normal"]
            # 装甲切向：法向垂直地面方向，近似取 (-ny, nx)
            nx, ny = normal[0], normal[1]
            horizon = math.hypot(nx, ny) or 1.0
            tx, ty = -ny / horizon, nx / horizon
            lateral = (px - cx) * tx + (py - cy) * ty
            vertical = pz - cz
            print(
                "  kind=%-7s armor=%d 横=%+7.1f 高=%+7.1f 距离=%6.3f m"
                % (event.get("kind"), event.get("armor_id", -1),
                   lateral * 1000, vertical * 1000, best_d)
            )

    if armors:
        print("\n装甲真值:")
        for armor in armors:
            cx, cy, cz = armor["center"]
            print(
                "  id=%d center=(%.2f, %.2f, %.2f) normal_yaw=%.1f°"
                % (armor["id"], cx, cy, cz,
                   math.degrees(armor.get("normal_yaw", 0.0)))
            )


if __name__ == "__main__":
    main()
