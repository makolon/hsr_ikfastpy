docker run -it --rm --net host --privileged \
    -v ~/hsr_ikfastpy:/ikfast \
    -v ~/hsr_ikfastpy/hsrb_ikfast:/root/.openrave \
    --name openrave_docker openrave-ikfast-docker:latest bash
