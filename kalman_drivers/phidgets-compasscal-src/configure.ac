# Process this file with autoconf to produce a configure script.

AC_INIT([Compass Calibrator C], [1.0.0], [support@phidgets.com])
AM_INIT_AUTOMAKE([-Wall -Werror foreign subdir-objects])
AC_CONFIG_HEADERS([config.h])
AC_CANONICAL_HOST

# Checks for programs.
AC_PROG_CC

# Checks for libraries. uses -framework on MacOS
case $host in
  *darwin*)	EXTRA_LDFLAGS='-F/Library/Frameworks/ -framework Phidget22' ;;
  *)		AC_CHECK_LIB([phidget22],[Phidget_getLibraryVersion])
			EXTRA_LDFLAGS='' ;;
esac
AC_SUBST([EXTRA_LDFLAGS])
AC_SEARCH_LIBS([atan], [m], [],
	AC_MSG_ERROR([Missing libm!]))

# Checks for header files. 
AC_CHECK_HEADERS([unistd.h])
# Adds extra header search path for MacOS
case $host in
  *darwin*)	EXTRA_CFLAGS='-I/Library/Frameworks/Phidget22.framework/Headers' ;;
  *)		AC_CHECK_HEADERS([phidget22.h]) 
			EXTRA_CFLAGS='' ;;
esac
AC_SUBST([EXTRA_CFLAGS])

# Checks for typedefs, structures, and compiler characteristics.
AC_C_CONST

AC_CONFIG_FILES([Makefile])
AC_OUTPUT
