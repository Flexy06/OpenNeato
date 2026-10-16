#!/usr/bin/env bash
# Host simulation of the guided clean controller (no hardware needed).
# Copies the firmware sources under test next to the host fakes so that the
# fakes shadow the hardware-facing headers, then builds and runs the scenarios.
set -euo pipefail
here="$(cd "$(dirname "$0")" && pwd)"
src="$here/../../src"
work="$(mktemp -d "${TMPDIR:-/tmp}/guided_sim.XXXXXX")"
trap 'rm -rf "$work"' EXIT
for f in guided_clean_manager.cpp guided_clean_manager.h floor_map.cpp floor_map.h json_fields.cpp json_fields.h \
         neato_commands.cpp neato_commands.h config.h loop_task.h; do
    cp "$src/$f" "$work/"
done
cp "$here"/fake/*.h "$work/"
cp "$here/sim.cpp" "$work/"
g++ -std=gnu++17 -O1 -g -Wall -Wno-unused-function -Wno-unused-variable \
    -DCHIP_MODEL='"HOST"' -DCONFIG_IDF_TARGET_ESP32C3=1 -DENABLE_LOGGING=1 -I"$work" \
    "$work"/sim.cpp "$work"/guided_clean_manager.cpp "$work"/floor_map.cpp "$work"/json_fields.cpp \
    "$work"/neato_commands.cpp -o "$work/guided_sim"
"$work/guided_sim" "$@"
