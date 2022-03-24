# Kuka iiwa ROS2 Driver
Collection of packages to run the KUKA iiwa robot arms using the fast robot interface in ROS2.
Mostly a port of the iiwa_fri_stack for ROS1 made a few years ago, using the [UR driver](https://github.com/PickNikRobotics/Universal_Robots_ROS2_Driver) to structure the package and understand the ros2 changes.
  
The hardware interface currently only provides a position command interface but the state provides position, velocity and effort (measured torque on each joint).
## Installation (quick version)
  1. Clone the repository into your workspace
  2. Copy the contents of the iiwa_fri_java package into the src folder of your sunrise project (this will copy the RoboticsAPI.data.xml file so if you have changed the default one you will need to merge it with your file).
  3. Install the fast robot interface in the sunrise project, configure the KONI ip address and synchronise with the robot controller
  4. Build the workspace
  
## KUKA Networking Setup
The KONI connection on the kuka controller must be accessible from the computer running ros_control, check with ping (controller sends duplicates back that is ok).  
On the SmartPad in Process Data set the FRI Client IP to the address of the computer running ros_control and set the FRI Port to an unused port between 30200 and 30209.
In the launch file where the xacro file is processed set the robot_ip argument to the ip at the KONI connection and set the robot_port to the same value as on the SmartPad.  
An example of this can be found in the [example xacro](https://github.com/grdwyer/Kuka_iiwa_ROS2_Driver/blob/foxy/iiwa_fri_ros/config/load_iiwa.xacro#L42) line 42-53.  

## Running the example
A launch file and required configuration files are provided in `iiwa_fri_ros` to launch the hardware interface, controllers, a robot state publisher and rviz.  
To run you will need the [iiwa_fri_description](https://github.com/grdwyer/iiwa_fri_description.git) on the foxy branch.  
`git clone -b foxy https://github.com/grdwyer/iiwa_fri_description.git`  

Modify the load_iiwa.xacro to have your ip and port configuration.  
Build your workspace (you'll need to have ros2 control and rviz2 installed)  
`ros2 launch iiwa_fri_ros iiwa_position_controller.launch.py`

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
    iiwa_fri_driver:latest
```
This should be logging your host user in the container, mounting your home directory within the image and other things like x server info and sudo access.  
The repo has been added in the docker process and is owned by root so the user id you've added won't be able to use it.
Change ownership of the workspace to the user with:  
`sudo chown -R $UID /dev_ws/`
