#!/bin/bash
# Test 161-166, once per condition, both conditions of a seed concurrently (prereg stop_rule.test).
WT=/Users/changmin/projects/ugrp-wt/kiro-owncam-memory
E=$WT/experiments/2026-09-26-zone-owncam-memory
O=/Users/changmin/projects/ugrp/outputs/owncam-memory-20260926
for s in 161 162 163 164 165 166; do
  FREE=$(df -g "$O" | awk 'NR==2 {print $4}')
  if [ "$FREE" -lt 30 ]; then echo "$(date '+%F %T') STOP: ${FREE} GiB free before s$s" >> $O/logs/test-a1-queue.log; exit 3; fi
  echo "$(date '+%F %T') start s$s load={$(sysctl -n vm.loadavg | tr -d '{}')} free=${FREE}GiB" >> $O/logs/test-a1-queue.log
  $E/launch_episode.sh test-a1 off m1mem-s$s test $E/frozen_source.json > $O/logs/test-a1-off-s$s.log 2>&1 &
  a=$!
  $E/launch_episode.sh test-a1 memory_v2 m1mem-s$s test $E/frozen_source.json > $O/logs/test-a1-memory_v2-s$s.log 2>&1 &
  b=$!
  wait $a; ra=$?; wait $b; rb=$?
  echo "$(date '+%F %T') done s$s off=$ra memory_v2=$rb load={$(sysctl -n vm.loadavg | tr -d '{}')}" >> $O/logs/test-a1-queue.log
done
echo "$(date '+%F %T') queue finished" >> $O/logs/test-a1-queue.log
