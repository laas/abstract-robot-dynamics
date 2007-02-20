
dnl 
dnl configure.ac file for package jrlCIRdynamicsJRLJapan 
dnl

AC_INIT([jrlCIRdynamicsJRLJapan], [1.0], [olivier.stasse@aist.go.jp])
AC_PREREQ(2.59)

AM_INIT_AUTOMAKE

AC_PROG_CXX
AC_PROG_INSTALL
AC_PROG_LIBTOOL

AC_ARG_WITH(MAL,
	for Matrix Abstract Layer,
	MAL_CFLAGS=-I${withval}/include
	CXXFLAGS="${CXXFLAGS} -I${withval}/include",
	)

AC_ARG_WITH(robotDynamics,
	for JRL robot dynamics abstract interface,
	JRLCIRDYNAMICS_CFLAGS=-I${withval}/include
	CXXFLAGS="${CXXFLAGS} -I${withval}/include",
	)

AC_ARG_WITH(boost,
        for ublas,
        BOOST_CFLAGS=-I${withval}/include
        CXXFLAGS="${CXXFLAGS} -I${withval}/include",
        )

AC_ARG_WITH(boost-sandbox,
        for lapack bindings,
        BOOST_SANDBOX_CFLAGS="-I${withval} -D_BOOST_MATRIX_"
        CXXFLAGS="${CXXFLAGS} -I${withval}"
        CPPFLAGS="${CPPFLAGS} -D_BOOST_MATRIX_",
        )


AC_ARG_WITH(VNL,
        for using VNL linear algebra package,
        VNL_CFLAGS="-I${withval}/include -D_VNL_MATRIX_"
        VNL_LIB="-L${withval}/lib -lVNL"
        CXXFLAGS="${CXXFLAGS} -D_VNL_MATRIX_ -I${withval}/include"
        LDFLAGS="${LDFLAGS} -L${withval}/lib -lVNL",
        )

CXXFLAGS="${CXXFLAGS} -D_DEFAULT_MATRIX3x3_ -D_DEFAULT_VECTOR3D_"
LDFLAGS="${LDFLAGS} -llapack"

AC_OUTPUT(
	jrlCIRdynamicsJRLJapan.pc
	Makefile
	include/jrlCIRdynamicsJRLJapan/Makefile
	src/Makefile
	doc/Makefile
	doc/package.dox
	unitTesting/Makefile
	)

