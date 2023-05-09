docker run -it --rm --net host --privileged \
    -v `pwd`:/ikfast \
    -v `pwd`/hsrb_ikfast:/root/.openrave \
    --name openrave_docker openrave-ikfast-docker:latest bash
