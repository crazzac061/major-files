#!/bin/bash
# usage: ./run_seeds.sh <warm: heuristic|random> <outdir>
WARM=$1; OUT=$2
cd ~/qosslice && source .venv/bin/activate
for s in 0 1 2 3 4 5 6 7 8 9; do
  if [ -f "$OUT/seed$s/summary.json" ]; then echo "seed $s done, skipping"; continue; fi
  python -u -m ai.train --seed $s --warm $WARM --out $OUT > "$OUT.seed$s.log" 2>&1
done
echo "all done" > "$OUT.finished"
