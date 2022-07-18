# swiftnav-ros2
Swift Navigation's ROS2 SBP Driver

## Using Docker

Before you begin, make sure you have [Docker](https://docs.docker.com/docker-for-mac/install/) installed.
Start [Docker desktop](https://docs.docker.com/docker-for-mac/).

### Creating the image
You can make a local image fresh from by running `docker build` as such:

    docker build -t swiftnav-ros2 - < Dockerfile

Reading the Dockerfile from STDIN prevents docker from pulling in the whole
repostory into the build context (which is then immediately discarded anyway).
You can customize the UID of the user that's created with the docker image
by passing the desired `UID` value to the build:

    docker build -t swiftnav-ros2 --build-arg UID=1234 - < Dockerfile

You can then make this image operate on your local workspace like this:

    docker run --rm -v $PWD:/mnt/workspace/src/swiftnav-ros2 -i -t swiftnav-ros2:latest /bin/bash

### Using the docker image

