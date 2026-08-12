#!/bin/bash
# SCOPIO graph health check. Run with ROS 2 + the workspace sourced, from inside
# the container (`docker compose exec scopio bash`) or any machine on the LAN
# that has ROS 2 sourced and is on the same DDS domain.
#
#   bash scripts/smoke_test.sh
#
# It does NOT need hardware: nodes come up even with no camera/stage/galvo, so
# this verifies the graph itself first, then tells you which topics are actually
# producing data.
set -u
NS=/scopio

echo "================ nodes ================"
ros2 node list
echo
echo "expected:"
for n in calibration_node camera_node stage_node galvo_node temperature_node; do
  if ros2 node list 2>/dev/null | grep -q "$NS/$n"; then
    echo "  ok       $n"
  else
    echo "  MISSING  $n"
  fi
done

echo
echo "================ topics ================"
ros2 topic list

echo
echo "==== one message per topic (2s timeout) ===="
for t in stage/position camera/state awg/status temperature/status calibration image/compressed; do
  printf "  %-22s " "$NS/$t"
  if timeout 2 ros2 topic echo --once "$NS/$t" >/dev/null 2>&1; then
    echo "publishing"
  else
    echo "(no message in 2s)"
  fi
done

echo
echo "================ services ================"
ros2 service list | grep "$NS" || echo "  (none)"

echo
echo "================ actions ================"
ros2 action list || true

echo
echo "Done. 'MISSING' node => that node crashed (check 'docker compose logs')."
echo "'no message' on a topic => that hardware isn't present/working yet,"
echo "but the node is up (graceful degradation)."
