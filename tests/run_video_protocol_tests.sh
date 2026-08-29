#!/usr/bin/env bash
set -euo pipefail

package_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
source_file="${package_dir}/src/vr_video_forwarder.cpp"
launch_file="${package_dir}/launch/d435i_vr.launch.py"

expect() {
    local file="$1"
    local pattern="$2"
    local message="$3"
    if ! rg -q --fixed-strings "${pattern}" "${file}"; then
        echo "FAIL: ${message}" >&2
        exit 1
    fi
}

expect "${source_file}" 'constexpr uint8_t kStreamLayoutMono = 1;' 'mono wire layout constant is missing'
expect "${source_file}" 'constexpr uint8_t kStreamLayoutSideBySide = 2;' 'side-by-side wire layout constant is missing'
expect "${source_file}" 'declare_parameter<std::string>("stream_layout", "mono")' 'stream_layout parameter is missing'
expect "${source_file}" 'chunk_header[7] = wire_stream_layout(stream_layout_);' 'OAR3 stream layout is not serialized'
expect "${launch_file}" "'stream_layout': 'mono'" 'head and hand launchers do not select mono layout'

echo 'PASS: video protocol checks'
