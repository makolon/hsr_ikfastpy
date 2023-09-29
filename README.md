# HSRIKFastPy
## Overview
The Toyota Human Support Robot (HSR) is a versatile robot designed to assist humans in various tasks. It is equipped with advanced kinematic capabilities, allowing it to perform complex manipulation tasks. HSRIKFastPy leverages OpenRave's IKFast C++ executables to provide a Python interface for efficient inverse kinematics calculations on the HSR.

## Features
- Python wrapper for OpenRave's IKFast C++ executables
- Fast and efficient inverse kinematics calculations for the HSR
- Simplified API for integreting inverse kinematics into Python applications

## Quick Start


### Create IK Python Interface

Clone this repository.
```
git clone --recursive https://github.com/makolon/hsr_ikfast.git
```

### Simulation & Real Demo
Build docker container.
```
cd hsr_ikfastpy/docker/docker_hsr
./build.sh
```

Then, run docker container.
```
cd ../
./run.sh
```

Compile the Cython wrapper.
```
cd ./hsr_ikfastpy/hsr_ikfast
python3 setup.py
```

Run demo in pybullet simulation environment.
```
cd ./examples/simulation
python test_hsr.py 
```

Run demo with real HSR. **(Please configure 'hsrrc' appropriately)**
```
cd ./examples/real
python ik_controller.py
```

## Modifying Robot Kinematics with OpenRave
#### :construction: Since arm_ik.cpp has already been generated using IKFast, you can skip the following steps. Only execute them if you want to regenerate arm_ik.cpp.

1. Build docker **(Optional)**
```
cd docker/docker_openrave
./build.sh
```

2. Run docker **(Optional)**
```
./run.sh
```

3. Debug the kinematics using OpenRave's viewer **(Optional)**
```
openrave wrapper.xml
```

4. Check the links in your file **(Optional)**
```
openrave-robot.py hsrb4s.dae --info links
```

5. Use OpenRave to re-generate the IKFast C++ code `ikfast61.cpp` **(Optional)**
```
./exec_openrave.sh
```

You can choose `--iktype` in `exec_openrave.sh` from the list of ik types described in this [document](http://openrave.org/docs/latest_stable/openravepy/ikfast/#ik-types).

6. Execute OpenRave's IKFast C++ executables code generation **(Optional)**
```
cd /ikfast/
./exec_openrave.sh
```

## License

## Acknowledgements
