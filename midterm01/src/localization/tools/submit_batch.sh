#!/bin/bash
# Usage: submit_batch.sh track1|track2 file1.csv [file2.csv ...]   -> submits, waits, prints scores
case $1 in track1) C=sdc-2026-midterm-01-track-01;; track2) C=sdc-2026-localization-track-02;; *) exit 1;; esac
shift; n=$#
for f in "$@"; do kaggle competitions submit $C -f "$f" -m "$(basename "$f" .csv)" -q >/dev/null && echo "submitted $f"; done
for i in $(seq 120); do [ "$(kaggle competitions submissions $C | sed -n 3,$((n+2))p | grep -c COMPLETE)" = "$n" ] && break; sleep 10; done
kaggle competitions submissions $C | sed -n 3,$((n+2))p | awk '{print $2, $NF}'
