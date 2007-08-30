dnl autoconf script for abstract interface robot dynamics
dnl
dnl   Research carried out within the scope of the Associated International Laboratory: Joint Japanese-French Robotics Laboratory (JRL)
dnl
dnl   Author: Florent Lamiraux LAAS-CNRS
dnl

AC_INIT([abstractRobotDynamics],[1.6],[openrobots@laas.fr])
AC_PREREQ(2.59)

AM_INIT_AUTOMAKE([foreign no-define])
AC_PROG_INSTALL

dnl Matrix Abstraction Layer
define(mal_reqd, [MatrixAbstractLayer >= 1.3])
AC_SUBST(MAL_REQD, "mal_reqd")
PKG_CHECK_MODULES(MAL, mal_reqd)
AC_SUBST(MAL_CFLAGS)
AC_SUBST(MAL_LIBS)

AC_OUTPUT(Makefile
	interfaces/Makefile
	documentation/Makefile
	documentation/robotDynamics.dox
	abstractRobotDynamics.pc
)
