docker run -it --rm --net host --privileged \
    -v ~/hsr_ikfastpy:/ikfast \
    -v ~/hsr_ikfastpy/output:/root/.openrave \
    --name openrave_docker openrave-ikfast-docker:latest bash
