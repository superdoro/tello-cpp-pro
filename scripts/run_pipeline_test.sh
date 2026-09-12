#!/usr/bin/env bash
# End-to-end check of the whole mapping -> localization -> navigation chain,
# with no drone and no hardware.
#
# It renders a synthetic flight (deterministic, and with ground truth in
# metres), builds a map from that video exactly as you would from a real
# recording, localizes against the saved map, checks the reconstruction
# against ground truth, derives a route from the mapping trajectory, and flies
# it in dry-run. If this passes, everything except the radio link and the
# airframe has been exercised.
set -euo pipefail

cd "$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

BUILD_DIR=${BUILD_DIR:-build}
VOCAB=third_party/ORB_SLAM3/Vocabulary/ORBvoc.txt
DATA=tests/data
MAP=maps/synthetic.osa

if [[ ! -f "$VOCAB" ]]; then
    echo "error: $VOCAB missing. Run scripts/fetch_vocabulary.sh" >&2
    exit 1
fi
for binary in tests/tools/make_synthetic_video tests/tools/compare_trajectory \
              apps/map_builder/map_builder apps/relocalize/relocalize \
              apps/tools/trajectory_to_mission/trajectory_to_mission \
              apps/autonomous_mission/autonomous_mission; do
    if [[ ! -x "$BUILD_DIR/$binary" ]]; then
        echo "error: $BUILD_DIR/$binary missing." >&2
        echo "Configure with -DBUILD_SLAM=ON -DBUILD_TOOLS=ON -DBUILD_TESTS=ON" >&2
        exit 1
    fi
done

step() { printf '\n\033[1m== %s ==\033[0m\n' "$1"; }
fail() { printf '\n\033[31mFAILED: %s\033[0m\n' "$1" >&2; exit 1; }

step "1/6  Render a synthetic flight with ground truth"
"$BUILD_DIR/tests/tools/make_synthetic_video" --out "$DATA/synthetic.avi" \
    --truth "$DATA/synthetic_truth.txt" --camera "$DATA/synthetic_camera.yaml" | tail -4

step "2/6  Build a map from the video"
rm -f "$MAP" maps/synthetic.meta.yaml maps/synthetic_trajectory.txt
"$BUILD_DIR/apps/map_builder/map_builder" --video "$DATA/synthetic.avi" \
    --camera "$DATA/synthetic_camera.yaml" --out "$MAP" --no-viewer 2>&1 | tail -5

step "3/6  Check the reconstruction against ground truth, and record the scale"
compare_out=$("$BUILD_DIR/tests/tools/compare_trajectory" --estimate maps/synthetic_trajectory.txt \
    --truth "$DATA/synthetic_truth.txt" --write-scale "$MAP")
echo "$compare_out" | tail -10
# On noiseless synthetic input the reconstruction should be near-exact.
# Anything above a few centimetres means the geometry, not just the scale,
# has gone wrong.
ate=$(echo "$compare_out" | awk '/ATE \(RMSE\)/ {print $3}')
awk -v v="$ate" 'BEGIN { exit !(v < 0.05) }' || fail "ATE $ate m is too high - the map geometry is wrong"

step "4/6  Localize against the saved map"
reloc_out=$("$BUILD_DIR/apps/relocalize/relocalize" --map "$MAP" --video "$DATA/synthetic.avi" \
    --camera "$DATA/synthetic_camera.yaml" --no-viewer --no-preview 2>&1) || true
echo "$reloc_out" | grep -E "merged into|Relocalized after|Tracked [0-9]+ of"
echo "$reloc_out" | grep -q "merged into the pre-built map" \
    || fail "never merged into the pre-built map - localization fell back to a scratch map"

step "5/6  Derive a route from the mapping trajectory"
# --skip drops the run-up before the camera settles; --limit keeps the final
# waypoint away from the last frame of the footage, because a replay simply
# stops there and never gives the route's last waypoint its dwell time. A real
# flight has no such deadline, but the test needs a route that can finish.
"$BUILD_DIR/apps/tools/trajectory_to_mission/trajectory_to_mission" --map "$MAP" \
    --out config/mission_synthetic.yaml --spacing 0.8 --skip 3 --limit 5 2>&1 \
    | grep -E "sampled|Wrote"

step "6/6  Fly the route in dry-run"
mission_out=$("$BUILD_DIR/apps/autonomous_mission/autonomous_mission" --map "$MAP" \
    --mission config/mission_synthetic.yaml --camera "$DATA/synthetic_camera.yaml" \
    --video "$DATA/synthetic.avi" --dry-run --no-viewer --no-preview 2>&1 | tr '\r' '\n') || true
echo "$mission_out" | grep -E "reached|Mission |Reached"
echo "$mission_out" | grep -q "Mission COMPLETE" || fail "the route did not complete"

# Every stick command should be a small step away from the last one, except
# where the setpoint genuinely moved. Large steps mid-segment mean the
# derivative term is spiking - which on a real airframe is a visible twitch.
commands=$(echo "$mission_out" \
    | grep -oE "rc -?[0-9]+ -?[0-9]+ -?[0-9]+ -?[0-9]+ +AUTO +[A-Z]+ +wp [0-9]+/[0-9]+" || true)

# Assert the parse worked before trusting its verdict. A regex that matches
# nothing yields "0 spikes", which is indistinguishable from a clean run - so
# a cosmetic change to the HUD would silently disable this check.
command_count=$(printf '%s\n' "$commands" | grep -c "^rc" || true)
[[ "$command_count" -gt 20 ]] \
    || fail "only parsed $command_count stick commands - the HUD format changed and this check is no longer reading anything"

spikes=$(printf '%s\n' "$commands" \
    | awk '{r=$2;p=$3;t=$4;y=$5;wp=$9
            if (NR>1 && wp==pwp) {
                d=(r-pr<0?pr-r:r-pr)+(p-pp<0?pp-p:p-pp)+(t-pt<0?pt-t:t-pt)+(y-py<0?py-y:y-py)
                if (d>10) n++
            }
            pr=r;pp=p;pt=t;py=y;pwp=wp} END {print n+0}')
echo "  parsed $command_count stick commands, mid-segment spikes: $spikes"
[[ "$spikes" -eq 0 ]] || fail "$spikes mid-segment command spikes - the controller is kicking"

printf '\n\033[1;32mPipeline test PASSED.\033[0m\n'
