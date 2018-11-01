# makefile written for gnu make
CXX		= g++
SRC		= ../src
CPPFLAGS	= -I$(SRC)
DEBUG		= -g
#OPTIMIZE	= -O2
GCCWARN		= -Wall
CXXFLAGS	= $(DEBUG) $(GCCWARN) $(OPTIMIZE) $(INCLUDES)

LIB		= ../libXmlRpc.a

# Add your system-dependent network libs here
# Solaris: -lsocket -lnsl
SYSTEMLIBS	=

LDLIBS		= $(SYSTEMLIBS) $(LIB)

TESTS		= HelloClient HelloServer TestBase64Client TestBase64Server TestValues TestXml Validator port_zero_server bool_test

all:		$(TESTS)

$(TESTS):	$(LIB)

clean:
		rm -f *.o
		rm -f $(TESTS)
