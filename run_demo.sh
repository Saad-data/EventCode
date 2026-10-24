#!/usr/bin/env bash
set -euo pipefail

project_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
metavision_root="${METAVISION_ROOT:-/home/aitsam/Documents/metavision_pro/openeb-5.3.0/build}"
hdf5_ecf_dir="$metavision_root/sdk/modules/stream/cpp/3rdparty/hdf5_ecf"

cmake -S "$project_dir" -B "$project_dir/build" \
  -DCMAKE_BUILD_TYPE=Release \
  -DCMAKE_PREFIX_PATH="$metavision_root/generated;$hdf5_ecf_dir"
cmake --build "$project_dir/build" --parallel

export MV_HAL_PLUGIN_PATH="$metavision_root/lib/metavision/hal/plugins"
export LD_LIBRARY_PATH="$metavision_root/lib${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}"
# OpenCV's Wayland backend cannot place a window on a chosen monitor. XWayland
# exposes the real 7680x2400 desktop coordinates reported by xrandr.
export QT_QPA_PLATFORM="${QT_QPA_PLATFORM:-xcb}"
exec "$project_dir/build/silky_barcode_demo" "$@"
