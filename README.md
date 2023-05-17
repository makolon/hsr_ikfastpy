# HSRIKFastPy
## Overview
The Toyota Human Support Robot (HSR) is a versatile robot designed to assist humans in various tasks. It is equipped with advanced kinematic capabilities, allowing it to perform complex manipulation tasks with precision. HSRIKFastPy leverages OpenRave's IKFast C++ executables to provide a Python interface for efficient inverse kinematics calculations on the HSR.

## Features
- Python wrapper for OpenRave's IKFast C++ executables
- Fast and efficient inverse kinematics calculations for the HSR
- Simplified API for integreting inverse kinematics into Python applications

## Quick Start
### Create IK Python Interface
1. Clone this repository
```
git clone https://github.com/makolon/hsr_ikfast.git
```

2. Build docker (Optional)
```
cd docker/docker_openrave
./build.sh
```

3. Run docker (Optional)
```
./run.sh
```

4. Execute OpenRave's IKFast C++ executables code generation (Optional)
```
cd /ikfast/
./exec_openrave.sh
```

5. Compile the Cython wrapper
```
cd ./hsrb_ikfast
python setup.py
```

### Simulation & Real Demo
In oder to run the following demo, you need to build another docker container.
```
cd docker/docker_hsr
./build.sh
```

Then, run docker container.
```
cd ../
./run.sh
```

Run demo in pybullet simulation environment
```
cd ./examples/simulation
python test_hsr.py 
```

Run demo in Real HSR
```
cd ./examples/real
python ik_controller.py
```

## Modifying Robot Kinematics with OpenRave
1. (Optional) Debug the kinematics using OpenRave's viewer
```
openrave wrapper.xml
```

2. (Optional) Check the links in your file
```
openrave-robot.py hsrb4s.dae --info links
```

3. Use OpenRave to re-generate the IKFast C++ code `ikfast61.cpp`
```
./exec_openrave.sh
```
You can choose `--iktype` in `exec_openrave.sh` from the list of ik types described in this [document](http://openrave.org/docs/latest_stable/openravepy/ikfast/#ik-types).

## License
HSRIKFastPy is released under the MIT Licence.

## Acknowledgements
