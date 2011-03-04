#!/bin/bash

BASE=/home/rolo/utaustin-wbc

${BASE}/release/apps/m3_behavior_test \
            -e "0.4 -0.2 -0.2" \
            -j "30 30 -30 45 120 0 0" \
            -t ${BASE}/apps/hello_goodbye_tasks.yaml \
            -f ${BASE}/robospecs/m3_with_hand.xml \
            -T ${BASE}/apps/jlimit_fallback.yaml \
            -b ${BASE}/apps/hello_goodbye_behaviors.yaml
