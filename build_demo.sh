#!/bin/sh
# Build the OSD megademo for a target: ./build_demo.sh <TARGET> [make args]
# e.g. ./build_demo.sh HAKRCF411D
set -e
cd "$(dirname "$0")"
T="${1:?usage: ./build_demo.sh <TARGET> [make args]}"
shift
make CONFIG="$T" EXTRA_FLAGS="-DCLOUD_BUILD ${EXTRA_FLAGS}" OPTIONS="USE_OSD USE_OSD_SD USE_DSHOT OSD_DEMO_BUILD" "$@"
