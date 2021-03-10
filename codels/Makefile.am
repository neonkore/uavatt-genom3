#
# Copyright (c) 2018-2019,2021 LAAS/CNRS
# All rights reserved.
#
# Redistribution  and  use  in  source  and binary  forms,  with  or  without
# modification, are permitted provided that the following conditions are met:
#
#   1. Redistributions of  source  code must retain the  above copyright
#      notice and this list of conditions.
#   2. Redistributions in binary form must reproduce the above copyright
#      notice and  this list of  conditions in the  documentation and/or
#      other materials provided with the distribution.
#
# THE SOFTWARE  IS PROVIDED "AS IS"  AND THE AUTHOR  DISCLAIMS ALL WARRANTIES
# WITH  REGARD   TO  THIS  SOFTWARE  INCLUDING  ALL   IMPLIED  WARRANTIES  OF
# MERCHANTABILITY AND  FITNESS.  IN NO EVENT  SHALL THE AUTHOR  BE LIABLE FOR
# ANY  SPECIAL, DIRECT,  INDIRECT, OR  CONSEQUENTIAL DAMAGES  OR  ANY DAMAGES
# WHATSOEVER  RESULTING FROM  LOSS OF  USE, DATA  OR PROFITS,  WHETHER  IN AN
# ACTION OF CONTRACT, NEGLIGENCE OR  OTHER TORTIOUS ACTION, ARISING OUT OF OR
# IN CONNECTION WITH THE USE OR PERFORMANCE OF THIS SOFTWARE.
#
#                                           Anthony Mallet on Wed May 30 2018
#
lib_LTLIBRARIES = libuavatt_codels.la

libuavatt_codels_la_SOURCES  =	uavatt_c_types.h
libuavatt_codels_la_SOURCES +=	uavatt_codels.c
libuavatt_codels_la_SOURCES +=	uavatt_main_codels.c

libuavatt_codels_la_SOURCES +=	controller.cc
libuavatt_codels_la_SOURCES +=	codels.h

libuavatt_codels_la_CPPFLAGS =	$(requires_CFLAGS)
libuavatt_codels_la_LIBADD   =	$(requires_LIBS)
libuavatt_codels_la_CPPFLAGS+=	$(codels_requires_CFLAGS)
libuavatt_codels_la_LIBADD  +=	$(codels_requires_LIBS)
libuavatt_codels_la_LDFLAGS  =	-release $(PACKAGE_VERSION)


# idl  mappings
BUILT_SOURCES=	uavatt_c_types.h
CLEANFILES=	${BUILT_SOURCES}
uavatt_c_types.h: ${top_srcdir}/uavatt.gen
	${GENOM3}  mappings \
	  -MD -MF .deps/$@.d -MT $@ --signature -l c $< >$@

-include .deps/uavatt_c_types.h.d
