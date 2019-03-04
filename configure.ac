dnl Autoconf file for building uavatt codels library.
dnl
dnl Copyright (c) 2018-2019 LAAS/CNRS
dnl All rights reserved.
dnl
dnl Redistribution  and  use  in  source  and binary  forms,  with  or  without
dnl modification, are permitted provided that the following conditions are met:
dnl
dnl   1. Redistributions of  source  code must retain the  above copyright
dnl      notice and this list of conditions.
dnl   2. Redistributions in binary form must reproduce the above copyright
dnl      notice and  this list of  conditions in the  documentation and/or
dnl      other materials provided with the distribution.
dnl
dnl THE SOFTWARE  IS PROVIDED "AS IS"  AND THE AUTHOR  DISCLAIMS ALL WARRANTIES
dnl WITH  REGARD   TO  THIS  SOFTWARE  INCLUDING  ALL   IMPLIED  WARRANTIES  OF
dnl MERCHANTABILITY AND  FITNESS.  IN NO EVENT  SHALL THE AUTHOR  BE LIABLE FOR
dnl ANY  SPECIAL, DIRECT,  INDIRECT, OR  CONSEQUENTIAL DAMAGES  OR  ANY DAMAGES
dnl WHATSOEVER  RESULTING FROM  LOSS OF  USE, DATA  OR PROFITS,  WHETHER  IN AN
dnl ACTION OF CONTRACT, NEGLIGENCE OR  OTHER TORTIOUS ACTION, ARISING OUT OF OR
dnl IN CONNECTION WITH THE USE OR PERFORMANCE OF THIS SOFTWARE.
dnl
dnl                                           Anthony Mallet on Wed May 30 2018
dnl

AC_PREREQ(2.59)

AC_INIT([uavatt-genom3],[1.0],[openrobots@laas.fr])
AC_CONFIG_MACRO_DIR([autoconf])
AC_CONFIG_AUX_DIR([autoconf])
AC_CONFIG_HEADERS([autoconf/acuavatt.h])
AM_INIT_AUTOMAKE([foreign no-define])

dnl Compilers
dnl
LT_INIT([disable-static])
AC_PROG_CC
AC_PROG_CXX

dnl Features
AC_SEARCH_LIBS([aio_write], [rt],, AC_MSG_ERROR([aio_write() not found], 2))

dnl External packages
PKG_CHECK_MODULES(requires, [
  openrobots2-idl >= 2.0
  genom3 >= 2.99.31
  eigen3
])

AC_PATH_PROG(GENOM3, [genom3], AC_MSG_ERROR([genom3 tool not found], 2))


dnl --with-templates option
AG_OPT_TEMPLATES([$GENOM3 ],
    [$srcdir/uavatt.gen])

dnl Doc
AM_MISSING_PROG([ASCIIDOCTOR], [asciidoctor])

dnl Output
AC_CONFIG_FILES([
	uavatt-genom3.pc
	uavatt-genom3-uninstalled.pc
	Makefile
	codels/Makefile
])
AC_OUTPUT
AG_OUTPUT_TEMPLATES
