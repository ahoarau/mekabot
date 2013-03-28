dnl Process this file with autoconf to produce a configure script.
AC_INIT(m3qa, 0.3, info@mekabot.com)
AC_CONFIG_AUX_DIR(config)
AC_CONFIG_SRCDIR(src/m3qa/components/factory_proxy.cpp)
AM_INIT_AUTOMAKE

AM_CONFIG_HEADER(config.h)
AC_PROG_CXX
AC_PROG_CC
AC_DISABLE_STATIC
AC_PROG_LIBTOOL

#Swig Stuff
AM_PATH_PYTHON(2.3)
AC_PROG_SWIG(1.3.21)
SWIG_ENABLE_CXX
SWIG_PYTHON

AC_OUTPUT( Makefile python/Makefile)
