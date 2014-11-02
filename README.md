Mekabot M3 Installation instructions
==============
![Meka robot at Ensta Paristech](http://googledrive.com/host/0B6zWJ1Gzg1UTVkgtMWJaX1NCdVE/meka2.jpg)

This wiki describes the full installation of m3 software to control/simulate the Meka robot at Ensta ParisTech.
This installation supports 3 versions, depending on your needs : 
- Only the M3 python API for development
- Real-time M3 (c++ and python) for development and simulation
- Real-time M3 for Meka's real-time PC (same as above but with EtherCAT component)

| ***OS Tested*** | ***Status*** | ***Notes***
|:------------------|:----:|:---------------:
| Ubuntu 12.04 x86 | OK | w ROS Hydro 
| Ubuntu 12.04 x64 | OK | w ROS Hydro
| Ubuntu 13.10 x86 | OK | w ROS Indigo 
| **Ubuntu 14.04 x64**| OK | w ROS Indigo/MoveIt! 


> Current version on the Meka : Ubuntu 14.04LTS on kernel 3.10.32, rtai4.0 (magma branch from the [cvs](https://gna.org/cvs/?group=rtai)), Igh EtherCAT master 1.5.2, ROS Indigo+MoveIt!

## Build Status
[![Build Status](https://travis-ci.org/ahoarau/mekabot.svg?branch=master)](https://travis-ci.org/ahoarau/mekabot)
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

> Note : if you only want the **python** interface, jump to the "install Mekabot" section.

### The RTAI-patched kernel
#### Download
```bash
# Determine if x86 or x64 (x86_x64)
uname -m
```
#### x86_64 (64bits)
```
folder_id=0B6zWJ1Gzg1UTZWRnQ2lUYjVtWnM
kernel_name=3.10.32-rtwar3_3.10.32-rtwar3-10.00.Custom_amd64
```


#### x86 (32bits)
```
folder_id=0B6zWJ1Gzg1UTaWgwX01VOHNwX28
kernel_name=3.8.13-rtmeka4.0_3.8.13-rtmeka4.0-10.00.Custom_i386.deb
```

### Download
```
headers=linux-headers-$kernel_name.deb
image=linux-image-$kernel_name.deb

# Get the Rtai4.0 patched kernel headers
wget https://googledrive.com/host/$folder_id/$headers

# Get the Rtai4.0 patched kernel image
wget https://googledrive.com/host/$folder_id/$image
```
> Note: more rtai kernels are available [here](http://goo.gl/xFhHV6).

#### Installation

```bash
sudo dpkg -i --force-all $headers $image
```

Now **boot** on the new kernel using **grub** at **startup**. Please note the name of the kernel.
> Note : you might have to either hold sift on startup or update the grub config to boot on the rtai patched kernel: 
```bash
sudo nano /etc/defaults/grub
```

#### (Recommended) Configure your kernel boot options
```bash
sudo nano /etc/defaults/grub
# Then edit the following line: 
GRUB_CMDLINE_LINUX_DEFAULT="quiet splash"
# To :
GRUB_CMDLINE_LINUX_DEFAULT="quiet splash lapic=notscdeadline hpet=disable i915.i915_enable_rc6=0 i915.powersave=0 intel_idle.max_cstate=0 processor.max_cstate=0 idle=poll"

# Then apply the update:
sudo update-grub
sudo reboot
```


### RTAI 4.0 installation 
#### Download
```bash
sudo apt-get install automake
git clone https://github.com/ShabbyX/RTAI.git ~/RTAI
```

#### Installation
```bash
cd ~/RTAI
 ./autogen.sh
make menuconfig
# Configure the Number of cpus you have, and uncheck oneshot timer
make
sudo make install
```
> Notes: Rtai libraries, modules headers etc should be installed in /usr/realtime/, and that makes everyone's life easier.

> ----
> **Know issues** : On 64-bit CPUs, if an error regarding -mpreferred-cache-boundary=3 shows up, edit line 57 in /usr/src/linux/arch/x86/Makefile (where linux is your rtai patched kernel) to set this parameter to 4:
```bash
sudo gedit  /usr/src/linux/arch/x86/Makefile
# 57: KBUILD_CFLAGS += $(call cc-option,-mno-sse -mpreferred-stack-boundary=4)
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
sudo sh -c "echo 'deb http://packages.ros.org/ros/ubuntu $(lsb_release -cs) main' > /etc/apt/sources.list.d/ros-latest.list"
```
#### For Ensta people : use local repo (way faster)
```bash
sudo sh -c "echo 'deb http://fermion.ensta.fr/ros/ubuntu $(lsb_release -cs) main' > /etc/apt/sources.list.d/ros-latest.list"
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

### (Recommended) Install some IDEs

#### Python (for most users): Eclipse + PyDev or spyder
```bash
sudo apt-get install eclipse spyder
```

#### ROS and C++ Real-time (Advanced users): Qt creator and/or Kdevelop
```bash
sudo apt-get install qtcreator 
sudo apt-get install kdevelop
```

##Install Mekabot M3

### Download
```bash
git clone https://github.com/ahoarau/mekabot.git ~/mekabot
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
make -j$[$(nproc)+1]
sudo make install
```

#### Mekabot

```bash
cd ~/mekabot
mkdir build;cd build
cmake .. -DCMAKE_BUILD_TYPE=Release
make -j$[$(nproc)+1]
sudo make install
```
> Note : 
> * Compiling in Release makes the M3 system **twice** as fast (Essentially due to KDL). 
> * If you are running on real hardware, install [EtherCAT](https://github.com/ahoarau/ethercat-drivers) first, then compile Mekabot with -DETHERCAT=1 and in release as above.

## Update your bashrc
```bash
touch ~/.m3rc
echo '
##################################################################
## Meka

## The M3 Software environnement setup
source /usr/local/share/m3/setup.bash

## Meka config files location
export M3_ROBOT=~/mekabot/m3ens/real_meka

## Virtual Config onverlay
export M3_ROBOT=$M3_ROBOT:~/mekabot/m3ens/virtual_meka

## Some python hacks
export MALLOC_CHECK_=0


##################################################################
## ROS

#export ROS_MASTER_URI=http://meka-moch:11311 # If on real Meka, roscore is launched from meka-moch
#export ROS_IP=192.168.20.117 # Fix here your IP to avoid conflicts on Meka
source /opt/ros/indigo/setup.bash # Can be Hydro or Indigo

##################################################################
## ROS-workspace
source ~/catkin_ws/devel/setup.bash
source ~/catkin_ws/devel_isolated/setup.bash
source ~/catkin_ws/install_isolated/setup.bash

##################################################################
## Additional Meka-stuff

export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:~/mekabot/m3ens-demos/ros:~/mekabot/m3ens-tutos/ros:~/mekabot/m3ens-utils/ros:~/mekabot/meka-ros-pkg:~/mekabot/m3core/ros:~/mekabot/m3meka/ros
export PYTHONPATH=$PYTHONPATH:~/mekabot/m3ens-demos/scripts:~/mekabot/m3ens-utils/scripts:~/mekabot/m3ens-utils/python:~/mekabot/m3ens-utils/ros
'>>~/.m3rc

echo 'source ~/.m3rc' >> ~/.bashrc
source ~/.bashrc
```
### (Recommended) Compile Legacy shared memory ROS
```
cd ~/catkin_ws/src
ln -snf ~/mekabot/m3core/ros m3core_ros
ln -snf ~/mekabot/m3meka/ros m3meka_ros
cd ~/catkin_ws
catkin_make_isolated
catkin_make_isolated
catkin_make
```


### Get time synchronization for ROS (Highly recommended for Ensta users)
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

## Configure your virtual installation
This scripts just updates the hostname in your m3_config.yml, that tells which computer is running the m3rt_server. On the meka it's meka-mob, but for vitual installtions, it's your computer!

```bash
configure_robot_config_virtual.py
```

## Run the server and visualize the robot on Rviz (virtual installation)
```bash
# run the realtime server
m3rt_server_run 
# In another terminal :
# Launch roscore, robot description, robot state publisher, joint state publisher and rviz
roslaunch meka_description m3ens_viz.launch 
```

## You're done !

>  *Maintainer* : Antoine Hoarau <hoarau.robotics@gmail.com>
