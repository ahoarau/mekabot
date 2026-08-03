

Instrucciones de instalación de Mekabot M3
==============

<img src="https://drive.google.com/uc?export=view&id=0B6zWJ1Gzg1UTSmJwcVczTGJmSE0" style="width: 500px; max-width: 100%; height: auto" title="Meka Robot M3 Ensta ParisTech" /></a>

Este wiki describe la instalación completa del software m3 para controlar/simular el robot Meka en Ensta ParisTech.
Esta instalación es compatible con 3 versiones, según sus necesidades:
- Solo la API de Python M3 para desarrollo
- M3 en tiempo real (c++ y python) para desarrollo y simulación
- M3 en tiempo real para la PC en tiempo real de Meka (igual que lo anterior pero con el componente EtherCAT)

| ***Sistema Operativo Probado*** | ***Estado*** | ***Notas***
|:------------------|:----:|:---------------:
| Ubuntu 12.04 x86 | OK | con ROS Hydro 
| Ubuntu 12.04 x64 | OK | con ROS Hydro
| Ubuntu 13.10 x86 | OK | con ROS Indigo 
| **Ubuntu 14.04 x64**| OK | con ROS Indigo/MoveIt! 


> Versión actual en el Meka: Ubuntu 14.04LTS con el kernel 3.10.32, rtai4.0 (rama magma del [cvs](https://gna.org/cvs/?group=rtai)), maestro EtherCAT Igh 1.5.2, ROS Indigo+MoveIt!

## Estado de compilación
[![Build Status](https://travis-ci.org/ahoarau/mekabot.svg?branch=master)](https://travis-ci.org/ahoarau/mekabot)
## Ubuntu 12.04 - 14.04 (x86/x64) con ROS Hydro/Indigo

### Prerrequisitos
#### Necesarios 
```bash
sudo apt-get install cmake git libeigen3-dev libprotobuf-dev protobuf-compiler gnuplot-x11 libboost-dev python-dev python-protobuf python-matplotlib python-yaml python-gnuplot python-scipy python-sip-dev python-sip sip-dev swig python-pandas python-sympy python-nose python-numpy openssh-server openssh-client
```
##### Recomendados por si acaso necesita compilar un kernel más adelante
```bash
sudo apt-get install libqt4-dev moc g++ libncurses5-dev kernel-package gcc-multilib libc6-dev libtool automake  
```
------

> Nota: si solo desea la interfaz de **python**, salte a la sección "instalar Mekabot".

### El kernel parcheado con RTAI (x64)

### Descarga

La forma recomendada es compilar su propio kernel, pero puede probar este si lo desea: 

```
# Get the Rtai4.0 patched kernel headers
https://drive.google.com/open?id=0ByavWcVYC8lWWExlSlBLMjRaSW8

# Get the Rtai4.0 patched kernel image
https://drive.google.com/open?id=0ByavWcVYC8lWdERoazdNU285V2s
```

> Siempre es mejor [compilar su propio kernel](https://github.com/ahoarau/m3installation/blob/master/rtai-kernel-build.md)

#### Instalación

```bash
sudo dpkg -i linux-headers-3.10.32-rtwar4_3.10.32-rtwar4-10.00.Custom_amd64 linux-image-3.10.32-rtwar4_3.10.32-rtwar4-10.00.Custom_amd64
```

Ahora **inicie** el nuevo kernel usando **grub** al **arranque**. Tenga en cuenta el nombre del kernel.
> Nota: es posible que deba mantener presionada la tecla Shift al iniciar o actualizar la configuración de grub para arrancar con el kernel parcheado con rtai: 
```bash
sudo nano /etc/defaults/grub
```

#### (Recomendado) Configure las opciones de arranque de su kernel
```bash
sudo nano /etc/default/grub
# Then edit the following line: 
GRUB_CMDLINE_LINUX_DEFAULT="quiet splash"
# To :
GRUB_CMDLINE_LINUX_DEFAULT="quiet splash lapic=notscdeadline hpet=disable i915.i915_enable_rc6=0 i915.powersave=0 intel_idle.max_cstate=0 processor.max_cstate=0 idle=poll"

## Explanation
# lapic=notscdeadline : usefull for i5 and i7 processors, reduces latency by a factor of 10 on the meka-mob
# hpet=disable : rtai does not like hpet, so disable it to make sure it never uses it (it should not). Not mandatory as rtai is smart enough to not use it.
# i915.i915_enable_rc6=0 : disable deep sleep for processor, we want max power.
# intel_idle.max_cstate=0, processor.max_cstate=0 :  same as above
# i915.powersave=0 : disable the power management, i.e less interruptions.
# idle=poll : Poll forces a polling idle loop that can slightly improve the performance of waking up a idle CPU, but will use a lot of power and make the system run hot. Set CPU fan to maximum all the time (in bios). 

# Theses option will provide you ~300 nanosecond latencies (which is very low, so, very good).



# Then apply the update:
sudo update-grub
sudo reboot
```


### Instalación de RTAI 4.0 
#### Descarga
```bash
sudo apt-get install automake
git clone https://github.com/ShabbyX/RTAI.git ~/RTAI
```

#### Instalación
```bash
cd ~/RTAI
 ./autogen.sh
make menuconfig
# Configure the Number of cpus you have, and uncheck oneshot timer
make
sudo make install
```
> Notas: Las bibliotecas de Rtai, módulos, encabezados, etc., deben instalarse en /usr/realtime/, y eso le facilita la vida a todos.

> ----
> **Problemas conocidos** : En CPUs de 64 bits, si aparece un error relacionado con -mpreferred-cache-boundary=3, edite la línea 57 en /usr/src/linux/arch/x86/Makefile (donde linux es su kernel parcheado con rtai) para establecer este parámetro en 4:
```bash
sudo gedit  /usr/src/linux/arch/x86/Makefile
# 57: KBUILD_CFLAGS += $(call cc-option,-mno-sse -mpreferred-stack-boundary=4)
```
Parte de la explicación: http://mail.rtai.org/pipermail/rtai/2013-December/026198.html

> ----
> **Problemas conocidos** : en máquinas de 12.04 de 32 bits, rtai falla al compilar (falta algún encabezado)
```bash
sudo apt-get install gcc-multilib g++-multilib libc6-dev
sudo ln -s /usr/include/i386-linux-gnu/gnu/stubs-32.h /usr/include/gnu/stubs-32.h
```

### Post instalación
Actualice la ruta de la biblioteca ld para encontrar rtai:
```bash
sudo -s
echo /usr/realtime/lib/ > /etc/ld.so.conf.d/rtai.conf
exit
sudo ldconfig
```


## Instalar ROS
```bash
sudo sh -c "echo 'deb http://packages.ros.org/ros/ubuntu $(lsb_release -cs) main' > /etc/apt/sources.list.d/ros-latest.list"
```
#### Para personas de Ensta: use el repositorio local (mucho más rápido)
```bash
sudo sh -c "echo 'deb http://fermion.ensta.fr/ros/ubuntu $(lsb_release -cs) main' > /etc/apt/sources.list.d/ros-latest.list"
```

> Si está en Ubuntu < 13.10
```bash
ROS_DISTRO=hydro
```

> Si está en Ubuntu > 13.10
```bash
ROS_DISTRO=indigo
```

#### ROS + MoveIt! + ROS Control
```bash
wget http://packages.ros.org/ros.key -O - | sudo apt-key add -
sudo apt-get update
sudo apt-get install ros-$ROS_DISTRO-desktop-full ros-$ROS_DISTRO-moveit-* ros-$ROS_DISTRO-ros-control ros-$ROS_DISTRO-ros-controllers python-rosinstall python-pip
```

Puede consultar la implementación de ros_control en la [wiki](https://github.com/ahoarau/mekabot/wiki).


```bash
sudo -E rosdep init
rosdep update
```
#### Openni (Solo ROS Hydro)
```bash
sudo apt-get install ros-$ROS_DISTRO-openni*
```

#### Crear el espacio de trabajo de ROS
```bash
source /opt/ros/$ROS_DISTRO/setup.bash
## Create the ROS-workspace
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/src
catkin_init_workspace
cd ~/catkin_ws/
catkin_make
```

## Instalar Mekabot M3

### Descarga
```bash
git clone https://github.com/ahoarau/mekabot.git ~/mekabot
cd ~/mekabot
git submodule init
git submodule update
git submodule foreach git checkout master
```
### Instalación
#### Holomni PCV para la base móvil
> Si está en Ubuntu 14.04 LTS:
```bash
sudo -E add-apt-repository ppa:hoarau-robotics/ppa
sudo apt-get update
sudo apt-get install holomni-pcv
```
> Si no:
```bash
cd ~/mekabot
cd holomni_pcv
mkdir build;cd build
cmake .. -DCMAKE_BUILD_TYPE=Release
make -j$(nproc)
sudo make install
```

#### Mekabot

```bash
cd ~/mekabot
mkdir build;cd build
cmake .. -DCMAKE_BUILD_TYPE=Release
make -j$(nproc)
sudo make install
```
> Nota: 
> * Compilar en Release hace que el sistema M3 sea **el doble** de rápido (principalmente debido a KDL). 
> * Si está ejecutando en hardware real, instale [EtherCAT](https://github.com/ahoarau/ethercat-drivers) primero, luego compile Mekabot con -DETHERCAT=1 y en release como se indica arriba.

## Actualice su bashrc
```bash
touch ~/.m3rc
echo '
##################################################################
## Meka

## The M3 Software environnement setup
source /usr/local/share/setup.bash

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
### (Recomendado) Compilar ROS de memoria compartida Legacy
```
cd ~/catkin_ws/src
ln -snf ~/mekabot/m3core/ros m3core_ros
ln -snf ~/mekabot/m3meka/ros m3meka_ros
cd ~/catkin_ws
catkin_make_isolated
catkin_make_isolated
catkin_make
```


### Obtener sincronización de tiempo para ROS (Muy recomendado para usuarios de Ensta)
```bash
sudo apt-get install ntp
sudo nano /etc/ntp.conf
```
Comente todas las líneas de servidores y agregue 'server ensta.ensta.fr'.
Debería verse así: 
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

> (OPCIONAL) Forzar la actualización del tiempo todos los días (puede desviarse después de un apagado prolongado) 
```bash
sudo -s
touch /etc/cron.daily/ntpdate
echo '#!/bin/sh
ntpdate ensta.ensta.fr'>>/etc/cron.daily/ntpdate
exit
sudo chmod 755 /etc/cron.daily/ntpdate
```


### (Recomendado) Instalar algunos IDEs

#### Python (para la mayoría de los usuarios): Eclipse + PyDev o spyder
```bash
sudo apt-get install eclipse spyder
```

#### ROS y C++ en tiempo real (Usuarios avanzados): Qt creator y/o Kdevelop
```bash
sudo apt-get install qtcreator 
sudo apt-get install kdevelop
```


### (OPCIONAL) Configurar las PCs del robot:
```bash
sudo -s
echo '192.168.20.117 meka-mob'>>/etc/hosts
echo '192.168.20.118 meka-moch'>>/etc/hosts
echo '192.168.20.119 meka-mud'>>/etc/hosts
exit
```

## Configure su instalación virtual
Este script solo actualiza el nombre de host en su m3_config.yml, que indica qué computadora está ejecutando el m3rt_server. En el meka es meka-mob, pero para instalaciones virtuales, ¡es su computadora!

```bash
configure_robot_config_virtual.py
```

## Ejecute el servidor y visualice el robot en Rviz (instalación virtual)
```bash
# run the realtime server
m3rt_server_run 
# In another terminal :
# Launch roscore, robot description, robot state publisher, joint state publisher and rviz
roslaunch meka_description m3ens_viz.launch 
```

## ¡Has terminado!

Consulte la [wiki](https://github.com/ahoarau/mekabot/wiki) para más información !


>  *Mantenedor* : Antoine Hoarau <hoarau.robotics@gmail.com>
