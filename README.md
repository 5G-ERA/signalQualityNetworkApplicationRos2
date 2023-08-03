# signal_quality_network_application_ros2
Code developed  under ROS2 Foxy


Running instructions.

Run using docker compose

Download file "docker-compose.yaml" and open terminal in that Folder;

Then run:
```
docker compose run --rm signal-mapper
```


Install ROS2 Foxy environment.

Here is sugested source for instalatin 
```
https://automaticaddison.com/how-to-install-ros-2-foxy-fitzroy-on-ubuntu-linux/
```


After instalation check if the ROS Foxy is running in terminal
```
printenv ROS_DISTRO
```

Output should be: 

foxy

If other distro is displayed then foxy, run this command
```
source /opt/ros/foxy/setup.bash
```

Or open Home Directory and find file .bashrc, if is not visible press ctr+H to display hidden files
and replace/comment the 
"source /opt/ros/(your_ros_distribution)/setup.bash"

and add following to ".bashrc" file:
```
source /opt/ros/foxy/setup.bash
```


Create Workspace for ROS2. "ros2_ws" name is not mandatory, you can add different name or if is already available Workspace for ros2, just add to your workspace downloaded code.
```
mkdir -p ros2_ws/src
```

Go inside ros2_ws directory
```
cd ros2_ws/
```


Download the code by pressing green button in top right corrner "<> Code" by pressing Download ZIP from folding menu.

Unzip and copy Folder "era_5g_network_signal_mapper_ros2" 
which is inside of this folder: "signal_quality_network_application-era_5g_network_signal_mapper_ros2_foxy_dev"
to folder: src which is in folder: "ros2_ws"


From terminal run command for compiling ros2 package.
Make sure you in the "ros2_ws/" directory in terminal, not in "ros2_ws/src/" or other location
```
colcon build
```
Source Workspace
```
source ~/ros2_ws/install/setup.bash
```

Or open Home Directory and find file .bashrc, if is not visible press ctr+H to display hidden files
and replace/comment the 
"source "other source"/setup.bash"

and add following to ".bashrc" file:
```
source ~/ros2_ws/install/setup.bash
```

While Simulation in ROS2 is running RUN network_signal_mapper_ros2
```
ros2 launch era_5g_network_signal_mapper_ros2 era_5g_network_signal_mapper_ros2.launch.py
```
While Simulation in ROS2 is running RUN network_signal_mapper_ros2
```
ros2 launch era_5g_network_signal_mapper_ros2 era_5g_network_signal_mapper_ros2.launch.py
```

Publish colors accordingly to signal from InfluxDB
```
ros2 run era_5g_network_signal_mapper_ros2 publisher
```

Run following command to create costmap from map and signal quality pointclouds
```
ros2 run era_5g_network_signal_mapper_ros2 costmap_translate
```