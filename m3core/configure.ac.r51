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
#AC_PROG_SWIG(1.3.21)
AC_PROG_SWIG(2.0)
SWIG_ENABLE_CXX
SWIG_PYTHON

#------------------------------------------------------------------------------
# Virtual M3
#------------------------------------------------------------------------------
AC_ARG_ENABLE([virtual-only],
[  --enable-virtual-only    Turn off realtime and ethercat],
[case "${enableval}" in
  yes) rtai=false ;;
  no)  rtai=true ;;
  *) AC_MSG_ERROR([bad value ${enableval} for --enable-virtual-only]) ;;
esac],[rtai=true])
AM_CONDITIONAL([RTAI], [test x$rtai = xtrue])

#------------------------------------------------------------------------------
# Linux kernel directory
#------------------------------------------------------------------------------
AC_ARG_WITH([linux-dir],
    AC_HELP_STRING(
        [--with-linux-dir=<DIR>],
        [Linux kernel sources @<:@running kernel@:>@]
    ),
    [
        sourcedir=[$withval]
    ],
    [
	sourcedir=/usr/src/linux-headers-`uname -r`
        if test \! -d ${sourcedir} ; then
           echo
           AC_MSG_ERROR([Failed to find Linux sources. Use --with-linux-dir!])
        fi
    ]
)

AC_MSG_CHECKING([for Linux kernel sources])

if test \! -r ${sourcedir}/.config; then
    echo
    AC_MSG_ERROR([No configured Linux kernel sources in $sourcedir])
fi

# Try to get kernel release string
if test -r ${sourcedir}/include/config/kernel.release; then
    kernelrelease=`cat $sourcedir/include/config/kernel.release`
elif test -r ${sourcedir}/.kernelrelease; then
    kernelrelease=`cat $sourcedir/.kernelrelease`
elif test -r ${sourcedir}/include/linux/utsrelease.h; then
    hdr=${sourcedir}/include/linux/utsrelease.h
    kernelrelease=`grep UTS_RELEASE $hdr | cut -d " " -f 3- | tr -d \"`
elif test -r ${sourcedir}/include/linux/version.h; then
    hdr=${sourcedir}/include/linux/version.h
    kernelrelease=`grep UTS_RELEASE $hdr | cut -d " " -f 3- | tr -d \"`
fi

if test -z "$kernelrelease"; then
    echo
    AC_MSG_ERROR([Failed to extract Linux kernel version!])
fi

# Extract three numbers from kernel release string
linuxversion=`echo $kernelrelease | grep -oE "^[[0-9]]+\.[[0-9]]+\.[[0-9]]+"`

AC_SUBST(LINUX_SOURCE_DIR,[$sourcedir])
AC_MSG_RESULT([$LINUX_SOURCE_DIR (Kernel $linuxversion)])

#------------------------------------------------------------------------------
# Ethercat
#------------------------------------------------------------------------------

#if test "$rtai" = true; then
#    PKG_CHECK_MODULES([ETHERCAT], [ethercat >= 1.5.0])
#fi

#PKG_CHECK_MODULES([ETHERCAT], [ethercat >= 1.5.0])

#AS_IF(["$rtai" = true], [
#   PKG_CHECK_MODULES([ETHERCAT], [ethercat >= 1.5.0])
#])


#------------------------------------------------------------------------------
# Eigen
#------------------------------------------------------------------------------
PKG_CHECK_MODULES([EIGEN2], [eigen2 >= 2.0.15])

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
#AC_SUBST(RTAI_DIR,[/usr/realtime-2.6.32.11-rtai-3.8.1])
AC_SUBST(RTAI_DIR,[/usr/realtime])

#TODO: have autoconf find correct ros dir
AC_SUBST(ROS_DIR,[/opt/ros/cturtle/])

#m3toolbox.pc

AC_CONFIG_FILES([
	m3base.pc
	Makefile
	python/Makefile
	src/m3rt/base/Makefile
	src/m3rt/ethercat/Makefile
	src/m3rt/rt_system/Makefile
])
AC_OUTPUT

