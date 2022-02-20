# Carla Simulator Setup

## Option 1: From Source

1. Download the version 0.9.11 release from https://github.com/carla-simulator/carla/releases.

2. Setup environment variables

Include CARLA Python API to the Python path:

```
export CARLA_ROOT=/path/to/your/carla/installation
export PYTHONPATH=$PYTHONPATH:${CARLA_ROOT}/PythonAPI/carla
export PYTHONPATH=$PYTHONPATH:${CARLA_ROOT}/PythonAPI/carla/agents
export PYTHONPATH=$PYTHONPATH:${CARLA_ROOT}/PythonAPI/carla/dist/carla-<VERSION>.egg # You need to check the exact file name in '${CARLA_ROOT}/PythonAPI/carla/dist/'
```

## Option 2: Use Docker

1. Pull Docker image

```
# download docker image (e.g. version 0.9.11)
docker pull carlasim/carla:<carla-version>
# extract the Carla Python API from the image
cd ~
mkdir carla-python
docker run --rm --entrypoint tar carlasim/carla:<carla-version> cC /home/carla/PythonAPI . | tar xvC ~/carla-python
```

2. Setup environment variables

```
export CARLA_ROOT=/path/to/your/carla/installation
export PYTHONPATH=$PYTHONPATH:~/carla-python/carla
export PYTHONPATH=$PYTHONPATH:~/carla-python/carla/agents
export PYTHONPATH=$PYTHONPATH:~/carla-python/carla/dist/carla-<VERSION>.egg
```