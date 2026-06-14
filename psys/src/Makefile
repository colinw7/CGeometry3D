#.SILENT:

CPP_CC = g++
C_CC   = gcc

AR = ar
RM = rm

CDEBUG = -g

INC_DIR = ../include
OBJ_DIR = ../obj
LIB_DIR = ../lib

all: dirs $(LIB_DIR)/libCPSy.a

dirs:
	@if [ ! -e ../obj ]; then mkdir ../obj; fi
	@if [ ! -e ../lib ]; then mkdir ../lib; fi

CPP_SRC = \
CPSysAttraction.cpp \
CPSysEulerIntegrator.cpp \
CPSysModifiedEulerIntegrator.cpp \
CPSysParticle.cpp \
CPSysRungeKuttaIntegrator.cpp \
CPSysSpring.cpp \
CPSysSystem.cpp \

CPP_OBJS = $(patsubst %.cpp,$(OBJ_DIR)/%.o,$(CPP_SRC))

CPPFLAGS = \
-std=c++17 \
$(CDEBUG) \
-I$(INC_DIR) \
-I../../CMath/include \
-I../../COS/include \
-I../../CUtil/include \
-I. \

clean:
	$(RM) -f $(OBJ_DIR)/*.o
	$(RM) -f $(LIB_DIR)/libCPSys.a

.SUFFIXES: .cpp

$(CPP_OBJS): $(OBJ_DIR)/%.o: %.cpp
	$(CPP_CC) -c $< -o $(OBJ_DIR)/$*.o $(CPPFLAGS)

$(LIB_DIR)/libCPSy.a: $(CPP_OBJS) $(C_OBJS)
	$(AR) crv $(LIB_DIR)/libCPSy.a $(CPP_OBJS) $(C_OBJS)
