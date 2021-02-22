# Kuka iiwa ROS2 Driver
Collection of packages to run the KUKA iiwa robot arms using the fast robot interface in ROS2.
Mostly a port of the iiwa_fri_stack for ROS1 made a few years ago, using the [UR driver](https://github.com/PickNikRobotics/Universal_Robots_ROS2_Driver) to structure the package and understand the ros2 changes.

## Installation (quick version)
  1. Clone the repository into your catkin workspace
  2. Copy the contents of the iiwa_fri_java package into the src folder of your sunrise project (this will copy the RoboticsAPI.data.xml file so if you have changed the default one you will need to merge it with your file).
  3. Install the fast robot interface in the sunrise project, configure the KONI ip address and synchronise with the robot controller
  4. Build the catkin workspace
  
## KUKA IP setup
FRI Client IP needs to be the same subnet as ubuntu ethernet connection
IP in hardware launch file need to be the same as FRI IP set up in sunrise

## Docker
### Build
There is a docker file that contains the packages and all the dependencies. 
Where the general workflow to build the image:  
- Clone the repo
- Build the docker image

```
git clone git@github.com:grdwyer/Kuka_iiwa_ROS2_Driver.git
cd Kuka_iiwa_ROS2_Driver
docker build --pull --rm -f ./.docker/Dockerfile  -t iiwa_fri_driver:foxy .
```
If you are changing the Dockerfile remove the `--rm` tag to keep your intermediate builds. 

### Running
My approach (2.3 from the [ROS guide](http://wiki.ros.org/docker/Tutorials/GUI))
```
docker run -it \
    --user=$(id -u $USER):$(id -g $USER) \
    --group-add sudo \
    --env="DISPLAY" \
    --workdir="/dev_ws/src" \
    --volume="/home/$USER:/home/$USER" \
    --volume="/etc/group:/etc/group:ro" \
    --volume="/etc/passwd:/etc/passwd:ro" \
    --volume="/etc/shadow:/etc/shadow:ro" \
    --volume="/etc/sudoers.d:/etc/sudoers.d:ro" \
    --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
    iiwa_fri_stack:latest
```
The container needs to be privileged to run as it needs access to the USB, the user added also needs to be in dialout and sudo.
This should be logging your host user in the container, mounting your home directory within the image and other things like x server info and sudo access.  
The other perk of this is your ssh keys are hopefully in `~/.ssh` so you can then push your changes. 

if you are going to use this container for a while then give it name with: `--name iiwa_fri_stack_dev`

Lastly the repo has been added in the docker process and is owned by root so the user id you've added won't be able to use it.
Change ownership to the user with 
`sudo chown -R $UID /dev_ws/`
