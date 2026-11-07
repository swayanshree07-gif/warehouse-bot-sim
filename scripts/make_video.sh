#!/usr/bin/env bash
# Turn captured camera frames into mp4 videos (Day 26).
#
#   scripts/make_video.sh MEDIA_DIR [FPS]
#
# MEDIA_DIR is what `run_sim.py --capture` / `launch_all.sh --media` wrote:
# MEDIA_DIR/<camera>/frame_NNNN.png. Writes MEDIA_DIR/<camera>.mp4 for each
# camera and MEDIA_DIR/demo.mp4 (overview | chase side by side) when both
# exist. FPS default 10 (frames are 0.5 sim-s apart -> 5x speed-up).
# H.264 (libx264, yuv420p) so the files play everywhere. Prints VIDEO_OK or
# VIDEO_FAIL; exit 1 if ffmpeg is missing or no video was made.
DIR="$1"; FPS="${2:-10}"
[ -d "$DIR" ] || { echo "usage: $0 MEDIA_DIR [FPS]"; exit 2; }
command -v ffmpeg >/dev/null || { echo "VIDEO_FAIL ffmpeg not installed (PNG frames are still in $DIR)"; exit 1; }
MADE=0
for cam in "$DIR"/*/; do
  cam="${cam%/}"; name="$(basename "$cam")"
  [ -f "$cam/frame_0000.png" ] || continue
  if timeout 300 ffmpeg -nostdin -loglevel error -y -framerate "$FPS" -i "$cam/frame_%04d.png" \
      -c:v libx264 -pix_fmt yuv420p -crf 26 "$DIR/$name.mp4"; then
    echo "VIDEO $DIR/$name.mp4 ($(ls "$cam" | wc -l) frames, $(du -h "$DIR/$name.mp4" | cut -f1))"
    MADE=$((MADE + 1))
  else
    echo "VIDEO_FAIL ffmpeg failed for $name"
  fi
done
if [ -f "$DIR/overview.mp4" ] && [ -f "$DIR/chase.mp4" ]; then
  # the two cameras can differ by a frame; -shortest keeps them in step
  timeout 300 ffmpeg -nostdin -loglevel error -y -i "$DIR/overview.mp4" -i "$DIR/chase.mp4" \
    -filter_complex "[0:v][1:v]hstack=inputs=2[v]" -map "[v]" -shortest \
    -c:v libx264 -pix_fmt yuv420p -crf 26 "$DIR/demo.mp4" \
    && echo "VIDEO $DIR/demo.mp4 (side by side, $(du -h "$DIR/demo.mp4" | cut -f1))"
fi
[ $MADE -gt 0 ] && { echo "VIDEO_OK $MADE"; exit 0; }
echo "VIDEO_FAIL no frames found in $DIR"; exit 1
