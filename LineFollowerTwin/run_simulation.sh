#!/bin/bash

# 1. Source the VSI environment
source /data/tools/pave/innexis_home/vsi_2025.2/env_vsi.bash 2>/dev/null

# 2. Input Arguments
KP=${1:-1.0}
KI=${2:-0.0}
KD=${3:-2.8}
LABEL=${4:-"best_run"}
PATH_TYPE=${5:-straight}
RANDOM_SPAWN=${6:-"false"} # New argument: set to "true" to randomize start

echo "================================================="
echo "  VSI AUTOMATION: $LABEL"
echo "  Path: $PATH_TYPE  |  Random Spawn: $RANDOM_SPAWN"
echo "  Kp: $KP  Ki: $KI  Kd: $KD"
echo "================================================="

# 3. Create the Results Directory
mkdir -p results

# 4. Generate Makefile for Client 0 (The Simulator)
# Added --random_spawn here
cat > Makefile.client0 << EOF
all: build sim
compile:
	@echo "No compile needed"
build:
	@echo "No build needed"
sim:
	PYTHONPATH=. python3 src/simulator/simulator.py --domain=AF_UNIX --server-url=localhost --random_spawn=$RANDOM_SPAWN
clean:
	@echo "Nothing to clean"
EOF

# 5. Generate Makefile for Client 1 (The Controller)
cat > Makefile.client1 << EOF
all: build sim
compile:
	@echo "No compile needed"
build:
	@echo "No build needed"
sim:
	PYTHONPATH=. python3 src/controller/controller.py --domain=AF_UNIX --server-url=localhost --kp=$KP --ki=$KI --kd=$KD --path=$PATH_TYPE
clean:
	@echo "Nothing to clean"
EOF

# 6. Generate Makefile for Client 2 (The Logger)
cat > Makefile.client2 << EOF
all: build sim
compile:
	@echo "No compile needed"
build:
	@echo "No build needed"
sim:
	PYTHONPATH=. python3 src/logger/logger.py --domain=AF_UNIX --server-url=localhost --output=results/results_${LABEL}.csv
clean:
	@echo "Nothing to clean"
EOF

# 7. Run the VSI orchestrator
vsiSim LineFollowingRobot.dt

echo "================================================="
echo ">>> SUCCESS: Experiment '$LABEL' Complete"
echo "================================================="