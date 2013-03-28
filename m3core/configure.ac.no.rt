AC_INIT(m3, 1.4, info@mekabot.com)
AC_CONFIG_AUX_DIR(config)
AM_INIT_AUTOMAKE

AM_CONFIG_HEADER(config.h)
AC_PROG_CXX
AC_PROG_CC
AC_DISABLE_STATIC
AC_PROG_LIBTOOL

#------------------------------------------------------------------------------
# SWIG
#------------------------------------------------------------------------------
AM_PATH_PYTHON(2.3)
AC_PROG_SWIG(1.3.21)
SWIG_ENABLE_CXX
SWIG_PYTHON

#------------------------------------------------------------------------------
# Yaml-cpp
#------------------------------------------------------------------------------
PKG_CHECK_MODULES([YAMLCPP], [yaml-cpp >= 0.2.5])

#------------------------------------------------------------------------------
# Protobuf
#------------------------------------------------------------------------------
PKG_CHECK_MODULES([PROTOBUF], [protobuf >= 2.2.0a])

#------------------------------------------------------------------------------
# Orocos KDL
#------------------------------------------------------------------------------
PKG_CHECK_MODULES([OROCOSKDL], [orocos-kdl >= 1.0.2])

#TODO: have autoconf find correct rtai dir
AC_SUBST(RTAI_DIR,[/usr/realtime-2.6.32.11-rtai-3.8.1])

#TODO: have autoconf find correct ros dir
AC_SUBST(ROS_DIR,[/opt/ros/cturtle/])

#m3toolbox.pc

AC_CONFIG_FILES([
	m3base.pc
	Makefile
	python/Makefile	
])
AC_OUTPUT

