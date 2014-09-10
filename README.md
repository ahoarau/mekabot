M3 Installation instructions
==============

This wiki describes the full installation of m3 software to control/simulate the Meka robot at Ensta ParisTech.

>  *Author* : Antoine Hoarau <hoarau.robotics@gmail.com>


| ***OS Tested*** | ***Status*** | ***Notes***
|:------------------|:----:|:---------------:
| Ubuntu 12.04 x86 | OK | Kinects might cause issues 
| Ubuntu 12.04 x64 | OK | Kinects might cause issues 
| Ubuntu 12.10 x86 | OK | On the Meka 04.04.14 -> will update to 14.04 LTS when ready 
| Ubuntu 12.10 x64 | OK | Now working 04.04.14 
| Ubuntu 13.04 x86 | OK | 
| Ubuntu 13.04 x64 | OK | 
| Ubuntu 13.10 x86 | OK | w ROS Indigo 
| Ubuntu 14.04 x64 | OK | w ROS Indigo/MoveIt! 


## Ubuntu 12.04 - 14.04 (x86/x64) w/ ROS Hydro/Indigo

### Prerequisites
#### Necessary 
```bash
sudo apt-get install cmake git libeigen3-dev libprotobuf-dev protobuf-compiler gnuplot-x11 libboost-dev python-dev python-protobuf python-matplotlib python-yaml python-gnuplot python-scipy python-sip-dev python-sip sip-dev swig python-pandas python-sympy python-nose python-numpy
```
#### Nice to have to maybe compile a kernel later
```bash
sudo apt-get install libqt4-dev moc g++ libncurses5-dev kernel-package gcc-multilib libc6-dev libtool automake  openssh-server openssh-client
```
------

### The RTAI-patched kernel
#### Download
```bash
# Determine if x86 or x64 (x86_x64)
_platform=$(uname -m) 

# Get the Rtai4.0 patched kernel headers
wget http://perso.ensta-paristech.fr/~hoarau/rtmeka-kern/$_platform/linux-headers-rt.deb

# Get the Rtai4.0 patched kernel image
wget http://perso.ensta-paristech.fr/~hoarau/rtmeka-kern/$_platform/linux-image-rt.deb
```
#### Installation

```bash
sudo dpkg -i linux-headers-rt.deb linux-image-rt.deb
```

Now **boot** on the new kernel using **grub** at **startup**. Please note the name of the kernel.
> Note : you might have to either hold sift on startup or update the grub config to boot on the rtai patched kernel: 
```bash
sudo nano /etc/defaults/grub
```

### RTAI 4.0 installation 
#### Download
```bash
wget --no-check-certificate https://www.rtai.org/userfiles/downloads/RTAI/rtai-4.0.tar.bz2
tar xjf rtai-4.0.tar.bz2
```

#### Installation
```bash
cd rtai-4.0
mkdir build; cd build
../configure --disable-comedi-lxrt --enable-cpus=$(nproc) --enable-math-c99 --with-linux-dir=/usr/src/linux-headers-3.8.13-rtmeka4.0
make -j$(nproc)
sudo make install
```
> **Note** : The --with-linux-dir option has to match the rtai-patched kernel

> ----
> **Know issues** : On 64-bit CPUs, if an error regarding -mpreferred-cache-boundary=3 shows up, edit line 57 in /usr/src/linux/arch/x86/Makefile (where linux is your rtai patched kernel) to set this parameter to 4:
```bash
KBUILD_CFLAGS += $(call cc-option,-mno-sse -mpreferred-stack-boundary=4)
```
Part of the explanation: http://mail.rtai.org/pipermail/rtai/2013-December/026198.html

> ----
> **Know issues** : on 12.04 32 bits machines, rtai fails to compile (some header is missing)
```bash
sudo apt-get install gcc-multilib g++-multilib libc6-dev
sudo ln -s /usr/include/i386-linux-gnu/gnu/stubs-32.h /usr/include/gnu/stubs-32.h
```

### Post install
Update the ld library path to find rtai:
```bash
sudo -s
echo /usr/realtime/lib/ > /etc/ld.so.conf.d/rtai.conf
exit
sudo ldconfig
```


## Install ROS
```bash
codename=`cat /etc/lsb-release | grep -m 1 "DISTRIB_CODENAME=" | cut -d "=" -f2`
sudo sh -c "echo 'deb http://packages.ros.org/ros/ubuntu $codename main' > /etc/apt/sources.list.d/ros-latest.list"
```
#### For Ensta people : use local repo (way faster)
```bash
codename=`cat /etc/lsb-release | grep -m 1 "DISTRIB_CODENAME=" | cut -d "=" -f2`
sudo sh -c "echo 'deb http://fermion.ensta.fr/ros/ubuntu $codename main' > /etc/apt/sources.list.d/ros-latest.list"
```

> If on Ubuntu < 13.10
```bash
ROS_DISTRO=hydro
```

> If on Ubuntu > 13.10
```bash
ROS_DISTRO=indigo
```

#### ROS + MoveIt! + ROS Control
```bash
wget http://packages.ros.org/ros.key -O - | sudo apt-key add -
sudo apt-get update
sudo apt-get install ros-$ROS_DISTRO-desktop-full ros-$ROS_DISTRO-moveit-* ros-$ROS_DISTRO-ros-control ros-$ROS_DISTRO-ros-controllers python-rosinstall python-pip
```
```bash
sudo -E rosdep init
rosdep update
```
#### Openni (ROS Hydro Only)
```bash
sudo apt-get install ros-$ROS_DISTRO-openni*
```

#### Create the ROS-workspace
```bash
source /opt/ros/$ROS_DISTRO/setup.bash
## Create the ROS-workspace
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/src
catkin_init_workspace
cd ~/catkin_ws/
catkin_make
```

#### Post-install
```bash
sudo -s
echo '/usr/local/lib' >> /etc/ld.so.conf
exit
sudo ldconfig
```

### Workaround for KDL issues

The function SetPayload that allow to modify how much weight the robot carries requires a tiny patch on the KDL library. This is a temporary solution.

```bash
#cd ~/catkin_ws/src
#git clone https://github.com/ahoarau/orocos_kinematics_dynamics
#cd orocos_kinematics_dynamics/orocos_kdl
#mkdir build;cd build; cmake ..
#make -j5
#sudo make install
#cd ../../python_orocos_kdl/
#mkdir build;cd build;cmake ..
#make -j5
#sudo make install 
```

> Note : This issue has been fixed in june 2014, we now use the ros orocos kdl.

### (Recommended) Install some IDEs

#### Python (for most users): Eclipse + PyDev
```bash
sudo apt-get install eclipse
```

#### ROS and C++ Real-time (Advanced users): Qt creator and/or Kdevelop
```bash
sudo apt-get install qtcreator 
sudo apt-get install kdevelop
```

##Install ENSTA M3

### Download
```bash
git clone --recursive git@bitbucket.org:ensta/mekabot.git ~/mekabot
cd ~/mekabot
git submodule init
git submodule update
git submodule foreach git checkout master
```
### Installation
#### Holomni PCV for the mobile base
> If on Ubuntu 14.04 LTS:
```bash
sudo -E add-apt-repository ppa:hoarau-robotics/ppa
sudo apt-get update
sudo apt-get install holomni-pcv
```
> If not :
```bash
cd ~/mekabot
cd holomni_pcv
mkdir build;cd build
cmake .. -DCMAKE_BUILD_TYPE=Release
make -j5
sudo make install
```

#### Mekabot

```bash
cd ~/mekabot
mkdir build;cd build
cmake .. -DETHERCAT=0 -DCMAKE_BUILD_TYPE=Release
make -j5
sudo make install
```

> Note : future option -DVIRTUAL=1 will remove fake ethercat module synchronization to avoid overrruns

## Update your bashrc
```bash
touch ~/.m3rc
echo '
##################################################################
## Meka

#export M3_ROBOT=~/mekabot/m3ens/real_meka # Real meka config
export M3_ROBOT=~/mekabot/m3ens/virtual_meka # Simulated Meka
export MALLOC_CHECK_=0 # Some Hack for Python
source /usr/local/share/m3/setup.bash

##################################################################
## ROS

#export ROS_MASTER_URI=http://meka-moch:11311 # If on real Meka, roscore is launched from meka-moch
#export ROS_IP=192.168.20.117 # Fix here your IP to avoid conflicts on Meka
source /opt/ros/hydro/setup.bash # Can be Hydro or Indigo

##################################################################
## ROS-workspace

source ~/catkin_ws/install_isolated/setup.bash
source ~/catkin_ws/devel/setup.bash

##################################################################
## Additional Meka-stuff

export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:~/mekabot/m3ens-demos/ros:~/mekabot/m3ens-tutos/ros:~/mekabot/m3ens-utils/ros:~/mekabot/meka-ros-pkg:~/mekabot/m3core/ros:~/mekabot/m3meka/ros
export PYTHONPATH=$PYTHONPATH:~/mekabot/m3ens-demos/scripts:~/mekabot/m3ens-utils/scripts:~/mekabot/m3ens-utils/python:~/mekabot/m3ens-utils/ros
'>>~/.m3rc

echo 'source ~/.m3rc' >> ~/.bashrc
source ~/.bashrc
```

### Get time synchronization for ROS (HIGHLY RECOMMENDED)
```bash
sudo apt-get install ntp
sudo nano /etc/ntp.conf
```
Comment all the servers lines and add 'server ensta.ensta.fr'.
It should look like that : 
```bash
#server 0.ubuntu.pool.ntp.org
#server 1.ubuntu.pool.ntp.org
#server 2.ubuntu.pool.ntp.org
#server 3.ubuntu.pool.ntp.org
server ensta.ensta.fr

# Use Ubuntu's ntp server as a fallback (or not at ensta ;) )
server ntp.ubuntu.com
```
```bash
sudo service ntp restart
```

> (OPTIONAL) Force the time to update every day (can drift after long shutdown) 
```bash
sudo -s
touch /etc/cron.daily/ntpdate
echo '#!/bin/sh
ntpdate ensta.ensta.fr'>>/etc/cron.daily/ntpdate
exit
sudo chmod 755 /etc/cron.daily/ntpdate
```



### (OPTIONAL) Setup robot's Pcs :
```bash
sudo -s
echo '192.168.20.117 meka-mob'>>/etc/hosts
echo '192.168.20.118 meka-moch'>>/etc/hosts
echo '192.168.20.119 meka-mud'>>/etc/hosts
exit
```

## Update the hostname (virtual installation)
If you want to run a simulated robot, you need to be the realtime controller and client. To do so, open the robot_config :

```bash
gedit ~/mekabot/m3ens/robot_config/m3_config.yml
```

And change the host name to your computer's hostname.

```bash
more /etc/hostname
```
## Run the server and visualize the robot on Rviz
```bash
m3rt_server_run # run the realtime server
roslaunch meka_description m3ens_viz.launch # launch robot description, robot state publisher, joint state publisher and rviz
```

## You're done !

