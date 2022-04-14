#
#  This file is part of MUMPS 4.10.0, built on Tue May 10 12:56:32 UTC 2011
#
topdir = ..
libdir = $(topdir)/lib

default:	d

.PHONY: default s d c z mumps_lib clean

s:
	$(MAKE) ARITH=s mumps_lib
d:
	$(MAKE) ARITH=d mumps_lib
c:
	$(MAKE) ARITH=c mumps_lib
z:
	$(MAKE) ARITH=z mumps_lib

include $(topdir)/Makefile.inc

mumps_lib:    $(libdir)/libmumps_common$(PLAT)$(LIBEXT) \
              $(libdir)/lib$(ARITH)mumps$(PLAT)$(LIBEXT)

OBJS_COMMON = \
        mumps_part9.o\
        mumps_common.o\
        mumps_ooc_common.o\
        mumps_orderings.o\
        mumps_size.o\
        mumps_io.o\
        mumps_io_basic.o\
        mumps_io_thread.o\
        mumps_io_err.o\
        mumps_static_mapping.o\
        mumps_sol_es.o\
	tools_common_mod.o

OBJS = 	$(ARITH)mumps_part1.o\
        $(ARITH)mumps_part2.o\
        $(ARITH)mumps_part3.o\
        $(ARITH)mumps_part4.o\
        $(ARITH)mumps_part5.o\
        $(ARITH)mumps_part6.o\
        $(ARITH)mumps_part7.o\
        $(ARITH)mumps_part8.o\
        $(ARITH)mumps_comm_buffer.o\
        $(ARITH)mumps_load.o\
        $(ARITH)mumps_c.o\
        $(ARITH)mumps_ooc_buffer.o\
        $(ARITH)mumps_ooc.o\
        $(ARITH)mumps_struc_def.o


$(libdir)/libmumps_common$(PLAT)$(LIBEXT):	$(OBJS_COMMON)
	$(AR)$@ $?
	$(RANLIB) $@

$(libdir)/lib$(ARITH)mumps$(PLAT)$(LIBEXT):    $(OBJS)
	$(AR)$@ $?
	$(RANLIB) $@

$(ARITH)mumps_load.o:		$(ARITH)mumps_comm_buffer.o \
				$(ARITH)mumps_struc_def.o

$(ARITH)mumps_ooc.o: 		$(ARITH)mumps_struc_def.o \
				$(ARITH)mumps_ooc_buffer.o \
				mumps_ooc_common.o

$(ARITH)mumps_ooc_buffer.o: 	mumps_ooc_common.o


$(ARITH)mumps_part1.o \
$(ARITH)mumps_part2.o \
$(ARITH)mumps_part3.o \
$(ARITH)mumps_part4.o \
$(ARITH)mumps_part5.o \
$(ARITH)mumps_part6.o \
$(ARITH)mumps_part7.o \
$(ARITH)mumps_part8.o:	$(ARITH)mumps_comm_buffer.o \
			$(ARITH)mumps_load.o \
			$(ARITH)mumps_ooc.o

$(ARITH)mumps_part5.o:	mumps_static_mapping.o
$(ARITH)mumps_part5.o:	$(ARITH)mumps_part2.o

$(ARITH)mumps_part2.o : tools_common_mod.o
$(ARITH)mumps_part8.o : mumps_sol_es.o

.SUFFIXES: .c .F .o
.F.o:
	$(FC) $(OPTF) $(INCS) $(IORDERINGSF) $(ORDERINGSF) -I. -I../include -c $*.F $(OUTF)$*.o
.c.o:
	$(CC) $(OPTC) $(INCS) -I../include $(CDEFS) $(IORDERINGSC) $(ORDERINGSC) -c $*.c $(OUTC)$*.o

$(ARITH)mumps_c.o:	mumps_c.c
	$(CC) $(OPTC) $(INCS) $(CDEFS) -DMUMPS_ARITH=MUMPS_ARITH_$(ARITH) \
	      $(IORDERINGSC) $(ORDERINGSC) -I../include -c $? $(OUTC)$@


clean:
	$(RM) *.o *.mod
