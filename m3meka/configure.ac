AC_PREREQ(2.59)
AC_INIT([m3],[1.3],[info@mekabot.com])
AC_CONFIG_AUX_DIR([config])
AM_INIT_AUTOMAKE([-Wall -Werror dist-bzip2])
AC_CONFIG_HEADERS([config.h])
AC_CONFIG_SRCDIR([config.h.in])
AC_CONFIG_MACRO_DIR([m4])

#------------------------------------------------------------------------------
# Global
#------------------------------------------------------------------------------

AC_PROG_CXX
AC_PROG_LIBTOOL
AM_PROG_CC_C_O
AM_PATH_PYTHON(2.3)
AC_PROG_SWIG(1.3.21)
SWIG_ENABLE_CXX
SWIG_PYTHON



#------------------------------------------------------------------------------
# Ethercat
#------------------------------------------------------------------------------
#PKG_CHECK_MODULES([ETHERCAT], [ethercat >= 1.5.0])

#------------------------------------------------------------------------------
# Eigen
#------------------------------------------------------------------------------
#PKG_CHECK_MODULES([EIGEN2], [eigen2 >= 2.0.15])

#------------------------------------------------------------------------------

#TODO: have autoconf find correct rtai dir
#AC_SUBST(RTAI_DIR,[/usr/realtime-2.6.32.11-rtai-3.8.1])

#TODO: have autoconf find correct ros dir
#AC_SUBST(ROS_DIR,[/opt/ros/cturtle/ros])

AC_CONFIG_FILES([
	Makefile
	python/Makefile
	src/m3/inverse_kine/Makefile
])
AC_OUTPUT
