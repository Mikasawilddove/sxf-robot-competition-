#!/usr/bin/env bash
set -euo pipefail

if [[ $# == 0 || ${1:-} == --help ]]; then
    printf 'Usage: %s rollout|dwa [station_id] [baseline|fine]\n' "${0##*/}"
    printf 'Place the robot at Start_A facing D. Run each mode separately and restore the same start pose between runs.\n'
    exit 0
fi
if [[ $# -gt 3 || ( $1 != rollout && $1 != dwa ) ]]; then
    printf 'Choose rollout or dwa, optionally followed by a station ID and AMCL profile.\n' >&2
    exit 2
fi

exec bash "$(dirname "$0")/run_match.sh" --to-board --at-start-a --speed slow \
    --planner "$1" --board-station "${2:-latest_recording}" --amcl-profile "${3:-baseline}"
