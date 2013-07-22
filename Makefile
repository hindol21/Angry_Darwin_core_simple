###############################################################
#
# Purpose: Makefile for AngryDarwin
# Author.: Hae Won Park
# Date:	2013. 7. 12.
#
###############################################################

TARGET = AngryDarwin

CXX = g++
INCLUDE_DIRS = -I./darwin/Linux/include -I./darwin/Framework/include -I./tinyxml
CXXFLAGS += -DLINUX -g -Wall -Wno-format -fmessage-length=0 -O3 $(INCLUDE_DIRS)
LIBS += -ljpeg -lpthread -lrt -lboost_regex  

SRCS :=	main.cpp CBRLfD_Simple.cpp Behavior.cpp ./tinyxml/tinyxml.cpp ./tinyxml/tinyxmlparser.cpp ./tinyxml/tinyxmlerror.cpp ./tinyxml/tinystr.cpp

# Add on the sources for libraries
SRCS := ${SRCS}

OBJS := $(addsuffix .o,$(basename ${SRCS}))


all: $(TARGET)

$(TARGET): $(OBJS) ./darwin/Linux/lib/darwin.a
	$(CXX) -o $(TARGET) $(OBJS) ./darwin/Linux/lib/darwin.a $(LIBS)
	
clean:
	rm -f $(OBJS) $(TARGET)





