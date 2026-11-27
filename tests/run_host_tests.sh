#!/bin/sh
# Builds and runs the unit tests on the development PC (no Raspberry Pi needed).
# Requires the imu_lib subproject to be present: meson subprojects download
set -e

cd "$(dirname "$0")/.."

output="${TMPDIR:-/tmp}/airburst_functions_test"

cc -std=c99 -Wall -Wextra -Wpedantic -Werror -Wshadow -Wdouble-promotion \
   -Iinclude -Isubprojects/imu_lib/include \
   tests/test_functions.c src/functions.c -lm -o "$output"

"$output"
