#!/usr/bin/env bash
# Test a built PlotJuggler apbin plugin without launching the GUI.
#
#   test_plugin.sh <plugin.so> [logfile.bin]
#
# Point it at a downloaded CI artifact or a locally built plugin. Runs three checks:
#   1. ABI floor    - which libstdc++/glibc the plugin demands, and whether this
#                     host can satisfy it. This is what catches "built on a newer
#                     distro than the user runs". Linux only.
#   2. Plugin load  - QPluginLoader + cast to PJ::DataLoader, as PlotJuggler does.
#   3. Parse        - readDataFromFile() on a real ArduPilot .bin.
#
# Build the helper it drives first:
#   cmake -S tests -B tests/build -DPJ_INCLUDE_DIR=<plotjuggler_install>/include
#   cmake --build tests/build
set -uo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
SMOKETEST="${SMOKETEST:-$SCRIPT_DIR/build/pj_plugin_smoketest}"

if [ $# -lt 1 ]; then
  echo "usage: $(basename "$0") <plugin.so> [logfile.bin]" >&2
  exit 2
fi

PLUGIN="$1"
shift
# Falls back to the small log committed alongside this script. Anything left over
# is forwarded to the helper, so --verify/--expect-* can be used from here too.
LOG="$SCRIPT_DIR/logs/00000001.BIN"
if [ $# -gt 0 ] && [ "${1#--}" = "$1" ]; then
  LOG="$1"
  shift
fi

if [ ! -f "$PLUGIN" ]; then
  echo "FAIL: no such plugin file: $PLUGIN" >&2
  exit 2
fi

echo "=== 1. ABI floor ==="
# highest_version <list of NAME_x.y.z> -> the numerically largest entry
highest() { sort -u -V | tail -n 1; }

need_cxx="$(objdump -T "$PLUGIN" 2>/dev/null | grep -oE 'GLIBCXX_[0-9.]+' | highest)"
need_glibc="$(objdump -T "$PLUGIN" 2>/dev/null | grep -oE 'GLIBC_[0-9.]+' | highest)"
have_cxx="$(strings /lib/*/libstdc++.so.6 2>/dev/null | grep -oE 'GLIBCXX_[0-9.]+' | highest)"

echo "  plugin requires : ${need_cxx:-none} / ${need_glibc:-none}"
echo "  this host has   : ${have_cxx:-unknown}"

abi_ok=1
if [ -n "$need_cxx" ] && [ -n "$have_cxx" ]; then
  # If the highest of (needed, have) is not `have`, the host is too old.
  if [ "$(printf '%s\n%s\n' "$need_cxx" "$have_cxx" | highest)" != "$have_cxx" ]; then
    echo "  FAIL: host libstdc++ is older than the plugin requires"
    echo "        the plugin was built on a newer distro than this one"
    abi_ok=0
  else
    echo "  OK  : host satisfies the plugin's libstdc++ requirement"
  fi
fi

echo "=== 2/3. Plugin load and parse ==="
if [ ! -x "$SMOKETEST" ]; then
  echo "  SKIP: $SMOKETEST not built." >&2
  echo "        cmake -S $SCRIPT_DIR -B $SCRIPT_DIR/build -DPJ_INCLUDE_DIR=<pj_install>/include" >&2
  echo "        cmake --build $SCRIPT_DIR/build" >&2
  [ "$abi_ok" -eq 1 ] || exit 1
  exit 3
fi

# offscreen so this runs over SSH and in CI with no display attached
export QT_QPA_PLATFORM=offscreen
"$SMOKETEST" "$PLUGIN" "$LOG" "$@"
result=$?

[ "$abi_ok" -eq 1 ] || exit 1
exit "$result"
