#!/bin/bash
# The OSD demo as a video: the firmware's demo code and music on the host (render.c), every
# field drawn as the AT7456E shows it, the AY synth for the sound -> 1080p50 MP4.
#   tmp/hostrender/make_video.sh [out.mp4]
set -euo pipefail
cd "$(dirname "$0")"
OUT="${1:-../osd_demo_1080p50.mp4}"
WORK="$(mktemp -d)"
trap 'rm -rf "$WORK"' EXIT

make -s
./render ../../src/platform/PICO/osd/betaflight.mcm "$WORK/a.raw" 900 > "$WORK/v.gray" 2> "$WORK/log.txt"
grep "scene" "$WORK/log.txt" || true

# Audio peak to -1 dB
PEAK=$(ffmpeg -hide_banner -f s16le -ar 48000 -ac 1 -i "$WORK/a.raw" -af volumedetect -f null - 2>&1 \
       | sed -n 's/.*max_volume: \(-*[0-9.]*\) dB/\1/p')
GAIN=$(echo "-1 - ($PEAK)" | bc -l)

# 360 × 288 OSD pixels a field -> the whole 1920 × 1080 frame (x6 / x4 nearest, then area down)
ffmpeg -hide_banner -v error -stats -y \
    -f rawvideo -pix_fmt gray -s 360x288 -r 50 -i "$WORK/v.gray" \
    -f s16le -ar 48000 -ac 1 -i "$WORK/a.raw" \
    -vf "scale=2160:1152:flags=neighbor,scale=1920:1080:flags=area,format=yuv420p" \
    -c:v libx264 -preset slow -crf 12 -tune animation -profile:v high -level 4.2 -r 50 \
    -af "volume=${GAIN}dB" -c:a aac -b:a 256k -ar 48000 -ac 2 -shortest -movflags +faststart "$OUT"
echo "$OUT"
