dnl $Id$
dnl config.m4 for extension pathfinder

PHP_ARG_ENABLE([pathfinder],
  [whether to enable pathfinder support],
  [AS_HELP_STRING([--enable-pathfinder],
    [Enable pathfinder support])],
  [no])

if test "$PHP_PATHFINDER" != "no"; then
  PHP_REQUIRE_CXX()

  AC_LANG_PUSH([C++])
  AC_MSG_CHECKING([whether the C++ compiler accepts -std=c++17])
  old_CXXFLAGS="$CXXFLAGS"
  CXXFLAGS="$CXXFLAGS -std=c++17"
  AC_COMPILE_IFELSE([AC_LANG_PROGRAM()], [
    AC_MSG_RESULT([yes])
  ], [
    CXXFLAGS="$old_CXXFLAGS"
    AC_MSG_RESULT([no])
    AC_MSG_ERROR([A C++17-capable compiler is required to build pathfinder])
  ])
  AC_LANG_POP([C++])

  PHP_NEW_EXTENSION(pathfinder,
    pathfinder.cpp \
    src/NavMesh.cpp \
    src/AStarSolver.cpp,
    $ext_shared,,
    -std=c++17 -O3 -fno-strict-aliasing -DNDEBUG)

  PHP_ADD_BUILD_DIR([$ext_builddir/src])
  PHP_ADD_INCLUDE([$ext_builddir])
  PHP_ADD_INCLUDE([$ext_srcdir])

  PHP_SUBST(PATHFINDER_SHARED_LIBADD)
fi
