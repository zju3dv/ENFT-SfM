AC_INIT(TooN, version-2.0.0-beta7)
AC_COPYRIGHT([Copyright T. Drummond, E. Rosten, G. Reitmayr 2006, 2007, 2008, 2009])
AC_LANG(C++)
AC_PROG_CXX

typeof=check
AC_ARG_ENABLE(lapack, [AS_HELP_STRING([--enable-lapack],[Use LAPACK where optional])], [lapack=$enableval])
AC_ARG_ENABLE(typeof, [AS_HELP_STRING([--enable-typeof=X],[Force typeof to be decltype, __typeof__, typeof, boost or none])], [typeof=$enableval])


if test "$lapack" == "" 
then
	AC_CHECK_LIB(lapack, dgesvd_, [lapack=yes], [lapack=no])
fi

if test "$lapack" != "no" 
then
	AC_DEFINE(TOON_USE_LAPACK, 1)
fi


define(TYPEOF_TEST,[
	if test "$typeof" == check
	then
		AC_MSG_CHECKING([for $1])
		AC_COMPILE_IFELSE([
		$3
		int main()
		{
			typedef $2(int()*double()) foo;
		}
		], [ AC_MSG_RESULT(yes);typeof=$1], AC_MSG_RESULT(no))
	fi
])

TYPEOF_TEST(decltype, decltype)
TYPEOF_TEST(__typeof__, __typeof__)
TYPEOF_TEST(typeof, typeof)
TYPEOF_TEST(boost, BOOST_TYPEOF, [#include<boost/typeof/typeof.hpp>])

if test "$typeof" == typeof
then
	AC_DEFINE(TOON_TYPEOF_TYPEOF, 1)
elif test "$typeof" == __typeof__
then
	AC_DEFINE(TOON_TYPEOF___TYPEOF__, 1)
elif test "$typeof" == decltype
then
	AC_DEFINE(TOON_TYPEOF_DECLTYPE, 1)
elif test "$typeof" == boost
then
	AC_DEFINE(TOON_TYPEOF_BOOST, 1)
fi

AC_MSG_CHECKING([for __attribute__((deprecated))])
AC_COMPILE_IFELSE([
__attribute__((deprecated)) void foo(){}
int main()
{
	foo();
}
], [ AC_MSG_RESULT(yes);AC_DEFINE(TOON_DEPRECATED_GCC,1 )], AC_MSG_RESULT(no))



cat<<FOO

You're on the development branch of TooN 2.0. Everything will probably work, but
the interface is a bit different from TooN-1.x

If you want TooN-1, then get it using:

cvs -z3 -d:pserver:anoncvs@cvs.savannah.nongnu.org:/cvsroot/toon co -r Maintenance_Branch_1_x TooN

or update what you currently have using:

cvs up -r Maintenance_Branch_1_x

or head over to:

http://mi.eng.cam.ac.uk/~er258/cvd/

Otherwise, please report any bugs you come across.


FOO


AC_CONFIG_HEADERS(internal/config.hh)
AC_OUTPUT(Makefile)

