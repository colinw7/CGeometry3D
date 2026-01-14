CPP_CC = g++

AR = ar
RM = rm

CDEBUG = -g

INC_DIR = ../include
OBJ_DIR = ../obj
LIB_DIR = ../lib
BIN_DIR = ../bin

all: $(LIB_DIR)/libCGeometry3D.a

CPP_SRC = \
CGeomAxes3D.cpp \
CGeomBox3D.cpp \
CGeomCamera3D.cpp \
CGeomCircle3D.cpp \
CGeomCone3D.cpp \
CGeomCube3D.cpp \
CGeomCylinder3D.cpp \
CGeometry3D.cpp \
CGeomFace3D.cpp \
CGeomFill3D.cpp \
CGeomHyperboloid3D.cpp \
CGeomLight3D.cpp \
CGeomLine3D.cpp \
CGeomMaterial.cpp \
CGeomNodeData.cpp \
CGeomObject3D.cpp \
CGeomParticle3D.cpp \
CGeomPoint3D.cpp \
CGeomPyramid3D.cpp \
CGeomScene3D.cpp \
CGeomSphere3D.cpp \
CGeomTorus3D.cpp \
CGeomTriangle3D.cpp \
CGeomUtil3D.cpp \
CGeomVertex3D.cpp \

CPP_OBJS = $(patsubst %.cpp,$(OBJ_DIR)/%.o,$(CPP_SRC))

CFLAGS = \
-std=c++17 \
$(CDEBUG) \
-I$(INC_DIR) \
-I. \
-I../../CRenderer/include \
-I../../CImageLib/include \
-I../../CFont/include \
-I../../CMath/include \
-I../../CFile/include \
-I../../CConfig/include \
-I../../CTimer/include \
-I../../CArgs/include \
-I../../CStrUtil/include \
-I../../COS/include \
-I../../CUtil/include \

clean:
	$(RM) -f $(OBJ_DIR)/*.o
	$(RM) -f $(LIB_DIR)/libCGeometry3D.a

.SUFFIXES: .cpp

$(CPP_OBJS): $(OBJ_DIR)/%.o: %.cpp
	$(CPP_CC) -c $< -o $(OBJ_DIR)/$*.o $(CFLAGS)

$(LIB_DIR)/libCGeometry3D.a: $(CPP_OBJS)
	$(AR) crv $(LIB_DIR)/libCGeometry3D.a $(CPP_OBJS)
