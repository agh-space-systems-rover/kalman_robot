## Process this file with automake to produce Makefile.in

noinst_PROGRAMS = compasscal

compasscal_SOURCES = \
 compasscal_ex/compass.c \
 compasscal_ex/compass.h \
 compasscal_lib/arithmetic_ansi.h \
 compasscal_lib/arithmetic_sse_double.h \
 compasscal_lib/arithmetic_sse_float.h \
 compasscal_lib/compasscal.h \
 compasscal_lib/compasscalroutines.c \
 compasscal_lib/lbfgs.c \
 compasscal_lib/lbfgs.h \
 compasscal_lib/matrix.c \
 compasscal_lib/matrix.h \
 compasscal_lib/stdafx.h

EXTRA_DIST = \
 compasscal.sln \
 compasscal_ex/compasscal_ex.vcproj \
 compasscal_lib/compasscal.c \
 compasscal_lib/stdafx.c \
 compasscal_lib/exports.def \
 compasscal_lib/compasscal_lib.vcproj
 

AM_LDFLAGS = $(EXTRA_LDFLAGS)
AM_CFLAGS = $(EXTRA_CFLAGS) -I../compasscal_lib -I../compasscal_ex -Icompasscal_lib -Icompasscal_ex
