LIBRARIES = libb64.a

# Build flags (uncomment one)
#############################
# Release build flags
CFLAGS += -O3
#############################
# Debug build flags
#CFLAGS += -g
#############################

SOURCES = cdecode.c  cencode.c

TARGETS = $(LIBRARIES)

LINK.o = gcc

CFLAGS += -Werror -pedantic
CFLAGS += -I../include

vpath %.h ../include/b64

.PHONY : clean

all: $(TARGETS) #strip

libb64.a: cencode.o cdecode.o
	$(AR) $(ARFLAGS) $@ $^

strip:
	strip $(BINARIES) *.exe

clean:
	rm -f *.exe* *.o $(TARGETS) *.bak *~

distclean: clean
	rm -f depend

depend: $(SOURCES)
	makedepend -f- $(CFLAGS) $(SOURCES) 2> /dev/null 1> depend

-include depend

