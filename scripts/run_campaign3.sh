#!/usr/bin/env bash
# Campaign 3: self-calibrating localizer (track D drives) x formation arms.
# Sequential so real-time factor stays ~1 (mission window is wall-clock).
cd "$(dirname "$0")/.."
R=$PWD/results
./scripts/run_batch.sh 8 "$R/c3_fixed"       280 formation:=fixed
./scripts/run_batch.sh 8 "$R/c3_aware"       280 formation:=aware
./scripts/run_batch.sh 8 "$R/c3_informative" 280 formation:=informative
echo CAMPAIGN3_DONE
