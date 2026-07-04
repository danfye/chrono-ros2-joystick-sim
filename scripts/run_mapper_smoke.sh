#!/usr/bin/env bash
set -euo pipefail

repo_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
build_dir="${repo_root}/build/smoke"
mkdir -p "${build_dir}"

cxx="${CXX:-c++}"
"${cxx}" -std=c++17 -Wall -Wextra -Wpedantic \
  -I"${repo_root}/include" \
  "${repo_root}/src/joystick_mapper.cpp" \
  "${repo_root}/tests/joystick_mapper_smoke.cpp" \
  -o "${build_dir}/joystick_mapper_smoke"

"${build_dir}/joystick_mapper_smoke"
