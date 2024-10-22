# MHMR ROS Gazebo Simulation
High fidelity simulation of multiple human multiple robots (MHMR) systems using ROS 2 (Humble Hawksbill) LTS and Gazebo Fortress (Ignition Fortress) LTS on Ubuntu 22.04.3 LTS (Jammy Jellyfish)

<kbd><img src="logs/mhmr_scenario1.gif" width="800" alt="mhmr scenario"/></kbd>

### Initial setup
Install Gazebo, ROS 2, PX4, Micro XRCE-DDS Agent, and setup the ROS 2 workspace. 

First install Gazebo Fortress (Ignition Fortress) LTS

```console
$ sudo apt-get update
$ sudo apt-get install lsb-release gnupg
$ sudo curl https://packages.osrfoundation.org/gazebo.gpg --output /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg
$ echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" | sudo tee /etc/apt/sources.list.d/gazebo-stable.list > /dev/null
$ sudo apt-get update
$ sudo apt-get install ignition-fortress
```

**Note**: Restart computer after installation to avoid a "[QT] Failed to create OpenGL context for format QSurfaceFormat..." related error.


Install ROS 2 (Humble Hawksbill) 

```console
$ sudo apt install software-properties-common
$ sudo add-apt-repository universe
$ sudo apt update && sudo apt install curl -y
$ sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
$ echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
$ sudo apt update && sudo apt upgrade -y
$ sudo apt install ros-humble-desktop
$ sudo apt install ros-dev-tools
$ source /opt/ros/humble/setup.bash && echo "source /opt/ros/humble/setup.bash" >> .bashrc
```

Install PX4 autopilot

```console
$ cd ~/
$ mkdir PX4
$ cd ~/PX4/
$ git clone https://github.com/PX4/PX4-Autopilot.git --recursive
$ bash ./PX4-Autopilot/Tools/setup/ubuntu.sh
$ cd ~/PX4/PX4-Autopilot
$ make px4_sitl gz_x500
```

If an error is encountered, run the following:
 ```console
 $ make distclean
 ```
then remove "g++--multilib\" and "gcc--multilib\" from "ubuntu.sh" file rerun the bash... cd.. and make... commands.


Install Micro XRCE-DDS Agent & Client
```console
$ cd ~/PX4
$ pip install --user -U empy==3.3.4 pyros-genmsg setuptools
$ git clone https://github.com/eProsima/Micro-XRCE-DDS-Agent.git
$ cd Micro-XRCE-DDS-Agent
$ mkdir build
$ cd build
$ cmake ..
$ make
$ sudo make install
$ sudo ldconfig /usr/local/lib/
```

Setup ROS 2 Workspace (in new terminal). Copy the ros2_ws workspace folder to the home directory,
```console
$ cd ~/PX4/
$ git clone https://github.com/abioyeayo/mhmr-ros-gazebo-simulation.git
$ mkdir -p ~/ros2_ws/src/
$ cp -r mhmr-ros-gazebo-simulation/ros2_ws ~/ros2_ws
```

Build the ROS 2 workspace
```console
$ cd ~/ros2_ws/
$ source /opt/ros/humble/setup.bash
$ colcon build
```

For packaging<24.0 related build error, "TypeError: canonicalize_version() got an unexpected keyword argument 'strip_trailing_zero'", run the following to fix it: 

```console
$ pip install -U packaging
```

then re-run 
```console
$ colcon build
```

Copy Gazebo model files to PX4 directory
```console
$ cp -r ~/PX4/mhmr-ros-gazebo-simulation/gazebo/ ~/PX4/PX4-Autopilot/Tools/simulation/gz/worlds/

```


### Running the Waypoint Navigator program


Open the first terminal to start the Micro XRCE-DDS Agent
```console
$ MicroXRCEAgent udp4 -p 8888
```

Open a second terminal to to start the PX4 SITL Gazebo simulation instance
```console
$ cd ~/PX4/PX4-Autopilot
$ make px4_sitl gz_x500
```

Open a third terminal to run the GCS heartbeat script to keep the PX4 SITL connection alive
```console
$ cd ~/PX4/mhmr-ros-gazebo-simulation
$ python3 gcs_heartbeat.py
```

Open the fourth terminal to run the waypoint navigator node
```console
$ source /opt/ros/humble/setup.bash
$ cd ~/ros2_ws
$ colcon build
$ source install/setup.bash
$ ros2 run uav_waypoint_navigation uav_waypoint_navigator 0 0 0 -5 5 0 -5 5 5 -5 0 5 -5
```


### Running MHMR Scenario

Open the first terminal to start the Micro XRCE-DDS Agent
```console
$ MicroXRCEAgent udp4 -p 8888
```

Open a second terminal to to start the first PX4 SITL Gazebo simulation instance
```console
$ cd ~/PX4/PX4-Autopilot
$ PX4_SYS_AUTOSTART=4001 PX4_GZ_WORLD=mhmr_lawn PX4_GZ_MODEL_POSE="-7,-7" PX4_SIM_MODEL=gz_x500  ./build/px4_sitl_default/bin/px4 -i 0
```

Open a third terminal to to start the second PX4 SITL Gazebo simulation instance
```console
$ cd ~/PX4/PX4-Autopilot
$ PX4_SYS_AUTOSTART=4001 PX4_GZ_MODEL_POSE="-8,-6" PX4_SIM_MODEL=gz_x500 ./build/px4_sitl_default/bin/px4 -i 1
```

Open a fourth terminal to to start the third PX4 SITL Gazebo simulation instance
```console
$ cd ~/PX4/PX4-Autopilot
$ PX4_SYS_AUTOSTART=4001 PX4_GZ_MODEL_POSE="-8,-8" PX4_SIM_MODEL=gz_x500 ./build/px4_sitl_default/bin/px4 -i 2
```

Open a fifth terminal to run the GCS heartbeat script to keep the PX4 SITL connections alive
```console
$ cd ~/PX4/mhmr-ros-gazebo-simulation
$ python3 gcs_heartbeat.py
```

Open the sixth terminal to run the MARL inference and fetch waypoint
```console
$ source /opt/ros/humble/setup.bash
$ cd ~/ros2_ws
$ colcon build
$ source install/setup.bash
$ ros2 run uav_waypoint_navigation uav_waypoint_processor_marl
```


### Troubleshooting

clean/rebuild workspace
```console
$ cd ~/ros2_ws
$ rm -rf build/ install/ log/
$ colcon build
```

PX4 SITL commands
```console
$ commander takeoff
$ commander land
$ commander status
```

Starting Gazebo standalone, run
```console
$ ign gazebo shapes.sdf
```

Starting Gazebo standalone with debugging, run
```console
$ ign gazebo shapes.sdf -v 4
```

Uninstalling Gazebo
```console
$ sudo apt remove ignition-fortress && sudo apt autoremove
```
