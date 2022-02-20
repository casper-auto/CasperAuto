#!/bin/sh
trap "trap - TERM && kill -- -$$" INT TERM EXIT

CWD=`pwd`
cd ${CARLA_ROOT}

# Start Carla server without display
echo "Start CARLA server (-opengl -quality-level=Epic)."
DISPLAY=
# ./CarlaUE4.sh -opengl -quality-level=Low &
./CarlaUE4.sh -opengl -quality-level=Epic&
SERVER_PID=$!

sleep 5s

# Set the simulation step
echo "Simulation step is set to 0.05s (20Hz)."
cd PythonAPI/util
python config.py --delta-seconds 0.05

# Hint
echo ""
echo "If error bind is already in use, or address already being used"
echo "Run:"
echo "    ps -aux | grep carla"
echo "    kill id"
echo ""

# Do not exit until
echo "CARLA server initialization finishes."
echo "Ctrl+c to terminate the server."
wait ${SERVER_PID}

cd ${CWD}
