#!/bin/bash
# csc21_race_batch.sh -- run N trials of csc21_race_trial.sh back to back.
# Usage: csc21_race_batch.sh <start_n> <end_n> <results_csv>
set -u
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
START="$1"
END="$2"
CSV="$3"
if [ ! -f "$CSV" ]; then
    echo "trial,outcome,race_verdict,d24_ptr,race_detail,resting_verdict,reserved,resting_pc,mode" > "$CSV"
fi
for n in $(seq "$START" "$END"); do
    "$HERE/csc21_race_trial.sh" "$n" "$CSV"
done
echo "BATCH DONE $START-$END"
