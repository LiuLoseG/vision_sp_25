#!/usr/bin/env bash
# 用本仓库自瞄代码（build/standard_sim）在 rm_aim_sim 仿真器上做闭环测试。
#
# 用法：
#   ./scripts/sim_test.sh                     # 跑 all：静止/平移/自转/复合靶，各 40s
#   ./scripts/sim_test.sh static 60           # 只测静止靶 60s
#   ./scripts/sim_test.sh combined 40         # 只测复合靶（平移+自转）
#   ./scripts/sim_test.sh all 40 --no-fire    # 只跟踪不发弹
#
# 场景：static / translate_slow / translate / spin / combined / combined_hard
# 输出：每个场景结束打印仿真端口径的 fired/hit/resolved/rate。
set -uo pipefail

ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
SIM_DIR="${RM_AIM_SIM_DIR:-$HOME/rm_aim_sim}"
BIN="$ROOT/build/standard_sim"
CONFIG="$ROOT/configs/sim.yaml"

SCENARIO="${1:-all}"
DURATION="${2:-40}"
shift 2 2>/dev/null || true
EXTRA=("$@")

if [[ ! -x "$BIN" ]]; then
  printf '找不到 %s，先编译：cd %s/build && make standard_sim -j8\n' "$BIN" "$ROOT" >&2
  exit 1
fi
if [[ ! -x "$SIM_DIR/run.sh" ]]; then
  printf '找不到仿真器：%s\n' "$SIM_DIR" >&2
  exit 1
fi

export OPENCV_UI_BACKEND="${OPENCV_UI_BACKEND:-QT5}"
export QT_QPA_PLATFORMTHEME="${QT_QPA_PLATFORMTHEME:-}"

SIM_PID=""
cleanup() {
  [[ -n "$SIM_PID" ]] && kill "$SIM_PID" 2>/dev/null
  pkill -f "godot --path $SIM_DIR" 2>/dev/null
  sleep 0.5
  rm -rf /dev/shm/rm_aim_sim_default
}
trap cleanup EXIT

run_scene() {
  local name="$1" cfg="$2"
  echo "=================== 场景: $name ==================="
  cleanup >/dev/null 2>&1
  rm -f /tmp/sim_scene.log
  ( cd "$SIM_DIR" && ./run.sh --max-fps 120 --disable-vsync -- --remote --config="$cfg" \
      --quit-after-seconds=$((DURATION + 60)) > /tmp/sim_scene.log 2>&1 ) &
  SIM_PID=$!

  # 等共享内存通道出现
  for _ in $(seq 1 60); do
    [[ -f /dev/shm/rm_aim_sim_default/state.json ]] && break
    sleep 0.5
  done
  sleep 8   # 让场景稳定（warmup + 云台归位）

  ( cd "$ROOT" && "$BIN" "$CONFIG" --duration="$DURATION" ${EXTRA[@]+"${EXTRA[@]}"} 2>&1 \
      | grep -E '\[Sim\] 结束|标定|检测后端' )

  # 仿真器侧的权威统计（与本轮客户端统计一致，用于交叉核对）
  python3 "$ROOT/tools/sim_impact_report.py" 2>/dev/null | head -2
  echo
}

case "$SCENARIO" in
  static)    run_scene "静止靶 4m" "$ROOT/configs/sim/static.cfg" ;;
  translate) run_scene "平移靶 2m/s" "$ROOT/configs/sim/translate.cfg" ;;
  translate_slow) run_scene "平移靶 1m/s" "$ROOT/configs/sim/translate_slow.cfg" ;;
  spin)      run_scene "自转靶 4rad/s" "$ROOT/configs/sim/spin.cfg" ;;
  combined)  run_scene "复合靶 平移1m/s+自转4rad/s" "$ROOT/configs/sim/combined.cfg" ;;
  combined_hard) run_scene "复合靶 平移2m/s+自转6rad/s" "$ROOT/configs/sim/combined_hard.cfg" ;;
  all)
    run_scene "静止靶 4m" "$ROOT/configs/sim/static.cfg"
    run_scene "平移靶 1m/s" "$ROOT/configs/sim/translate_slow.cfg"
    run_scene "平移靶 2m/s" "$ROOT/configs/sim/translate.cfg"
    run_scene "自转靶 4rad/s" "$ROOT/configs/sim/spin.cfg"
    run_scene "复合靶 平移1m/s+自转4rad/s" "$ROOT/configs/sim/combined.cfg"
    run_scene "复合靶 平移2m/s+自转6rad/s" "$ROOT/configs/sim/combined_hard.cfg"
    ;;
  *) echo "未知场景: $SCENARIO（可选 static/translate/translate_slow/spin/combined/combined_hard/all）" >&2; exit 1 ;;
esac
