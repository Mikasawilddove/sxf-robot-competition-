#!/usr/bin/env bash
set -e
cd -- "$(dirname -- "$0")/.."
scope=${1:-full}
if [[ "$scope" != full && "$scope" != cake-only ]]; then
    printf 'Usage: bash tools/run_field_match.sh [full|cake-only]\n' >&2
    exit 2
fi
resource=data/calibration/cake_west_20cm_e6e4276
exec bash tools/run_match.sh "$scope" --at-start-a --planner path --speed fast \
    --grasp-source official-planar --geometry "$resource/geometry.yaml" \
    --calibration "$resource/profile.yaml"
