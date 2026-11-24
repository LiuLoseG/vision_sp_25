#!/usr/bin/env bash
# 启动 rm_aim_sim（Godot 自瞄仿真器）并开好共享内存通道，供 build/standard_sim 接入。
#
# 用法：
#   ./scripts/run_sim.sh                 # 默认场景（蓝方步兵3，4m 静止靶）
#   ./scripts/run_sim.sh --config=res://configs/spin.cfg
#   ./scripts/run_sim.sh --quit-after-seconds=60
#
# 约定：
#   RM_AIM_SIM_DIR  仿真器仓库根目录，默认 $HOME/rm_aim_sim
#   GODOT_BIN       指定 Godot 可执行文件，默认让仿真器自己找 .tools/godot
set -euo pipefail

SIM_DIR="${RM_AIM_SIM_DIR:-$HOME/rm_aim_sim}"
if [[ ! -x "$SIM_DIR/run.sh" ]]; then
  printf '找不到仿真器：%s（可用 RM_AIM_SIM_DIR 指定路径）\n' "$SIM_DIR" >&2
  exit 1
fi

if [[ ! -x "$SIM_DIR/.tools/godot" && -z "${GODOT_BIN:-}" ]]; then
  printf '仿真器还没有 Godot 引擎，先执行：\n  cd %s && python3 tools/setup_godot.py\n' "$SIM_DIR" >&2
  exit 1
fi

# 首次运行需要导入资源（只需一次）
if [[ ! -d "$SIM_DIR/.godot" ]]; then
  echo "[run_sim] 首次运行，导入资源……"
  "$SIM_DIR/run.sh" --headless --editor --import --quit
fi

# 本仓库 standard_sim 与 OpenCV 窗口共用图形会话时，避开 GTK 主题插件崩溃
export OPENCV_UI_BACKEND="${OPENCV_UI_BACKEND:-QT5}"
export QT_QPA_PLATFORMTHEME="${QT_QPA_PLATFORMTHEME:-}"

# 关键：不禁用 vsync 时，窗口失焦/被遮挡会让 Godot 掉到 1Hz，
# 状态发布和 sim_time 都远慢于实时，闭环测试结果无意义。
# --remote：启动即打开“本机自瞄控制”，等价于窗口内按 F5
exec "$SIM_DIR/run.sh" --max-fps 120 --disable-vsync -- --remote "$@"
