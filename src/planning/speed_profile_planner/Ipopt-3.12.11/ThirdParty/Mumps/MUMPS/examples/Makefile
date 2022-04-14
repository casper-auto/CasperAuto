#
#  This file is part of MUMPS 4.10.0, built on Tue May 10 12:56:32 UTC 2011
#
topdir = ..
libdir = $(topdir)/lib

default: d

.PHONY: default all s d c z clean
.SECONDEXPANSION:

all:	s d c z

s:	ssimpletest
d:	dsimpletest c_example
c:	csimpletest
z:	zsimpletest


include $(topdir)/Makefile.inc

LIBMUMPS_COMMON = $(libdir)/libmumps_common$(PLAT)$(LIBEXT)


LIBSMUMPS = $(libdir)/libsmumps$(PLAT)$(LIBEXT) $(LIBMUMPS_COMMON)

ssimpletest:  $(LIBSMUMPS)  $$@.o
	$(FL) -o $@ $(OPTL) ssimpletest.o  $(LIBSMUMPS) $(LORDERINGS) $(LIBS) $(LIBBLAS) $(LIBOTHERS)


LIBDMUMPS = $(libdir)/libdmumps$(PLAT)$(LIBEXT) $(LIBMUMPS_COMMON)

dsimpletest: $(LIBDMUMPS)  $$@.o 
	$(FL) -o $@ $(OPTL) dsimpletest.o  $(LIBDMUMPS) $(LORDERINGS) $(LIBS) $(LIBBLAS) $(LIBOTHERS)


LIBCMUMPS = $(libdir)/libcmumps$(PLAT)$(LIBEXT) $(LIBMUMPS_COMMON)

csimpletest: $(LIBCMUMPS)  $$@.o
	$(FL) -o $@ $(OPTL) csimpletest.o  $(LIBCMUMPS) $(LORDERINGS) $(LIBS) $(LIBBLAS) $(LIBOTHERS)


LIBZMUMPS = $(libdir)/libzmumps$(PLAT)$(LIBEXT) $(LIBMUMPS_COMMON)

zsimpletest: $(LIBZMUMPS)  $$@.o
	$(FL) -o $@ $(OPTL) zsimpletest.o  $(LIBZMUMPS) $(LORDERINGS) $(LIBS) $(LIBBLAS) $(LIBOTHERS)


c_example:	$(LIBDMUMPS) $$@.o
	$(FL) -o $@ $(OPTL) $@.o $(LIBDMUMPS) $(LORDERINGS) $(LIBS) $(LIBBLAS) $(LIBOTHERS)


.SUFFIXES: .c .F .o
.F.o:
	$(FC) $(OPTF) $(INCS) -I. -I$(topdir)/include -c $*.F $(OUTF)$*.o
.c.o:
	$(CC) $(OPTC) $(INCS) -I. -I$(topdir)/include -c $*.c $(OUTC)$*.o


$(libdir)/libsmumps$(PLAT)$(LIBEXT):
	@echo 'Error: you should build the library' $@ 'first'
	exit -1

$(libdir)/libdmumps$(PLAT)$(LIBEXT):
	@echo 'Error: you should build the library' $@ 'first'
	exit -1

$(libdir)/libcmumps$(PLAT)$(LIBEXT):
	@echo 'Error: you should build the library' $@ 'first'
	exit -1

$(libdir)/libzmumps$(PLAT)$(LIBEXT):
	@echo 'Error: you should build the library' $@ 'first'
	exit -1

$(LIBMUMPS_COMMON):
	@echo 'Error: you should build the library' $@ 'first'
	exit -1

clean:
	$(RM) *.o [sdcz]simpletest c_example
