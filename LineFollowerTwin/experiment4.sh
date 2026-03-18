#!/bin/bash
# Run 1: Standard PD
./run_vsi.sh 1.0 0.0 2.8 "exp4_pure_pd" "straight" "false"
# Run 2: PID (with small Ki)
./run_vsi.sh 1.0 0.1 2.8 "exp4_with_pid" "straight" "false"
