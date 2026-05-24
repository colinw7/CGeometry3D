APPNAME = CQTclModel3DView

BUILD_DIR = $$PWD

include($$(MAKE_DIR)/qt_app.mk)

QMAKE_CXXFLAGS += \
-DBUILD_DIR=\"$$BUILD_DIR\"

QT += opengl

SOURCES += \
Main.cpp \
App.cpp \
Canvas.cpp \
Overview.cpp \
Control.cpp \
Status.cpp \
Camera.cpp \
ShaderProgram.cpp \
CQGLTexture.cpp \

HEADERS += \
Main.h \
App.h \
Canvas.h \
Overview.h \
Control.h \
Status.h \
Camera.h \
ShaderProgram.h \
CQGLTexture.h \

INCLUDEPATH += \
$(INC_DIR)/CQGLUtil \
$(INC_DIR)/CQImage \
$(INC_DIR)/CQTabSplit \
$(INC_DIR)/CQGroupBox \
$(INC_DIR)/CQIconButton \
$(INC_DIR)/CQRealSpin \
$(INC_DIR)/CQUtil \
$(INC_DIR)/CImportModel \
$(INC_DIR)/CGeometry3D \
$(INC_DIR)/CMaterial \
$(INC_DIR)/CGLCamera \
$(INC_DIR)/CTclUtil \
$(INC_DIR)/CImageLib \
$(INC_DIR)/CFont \
$(INC_DIR)/CUtil \
$(INC_DIR)/CMath \
$(INC_DIR)/CFileType \
$(INC_DIR)/CFile \
$(INC_DIR)/COS \

PRE_TARGETDEPS = \
$(LIB_DIR)/libCGeometry3D.a

unix:LIBS += \
-L$(LIB_DIR) \
-lCQGLUtil \
-lCQImage \
-lCQTabSplit \
-lCQGroupBox \
-lCQIconButton \
-lCQRealSpin \
-lCQStyle \
-lCQPixmapCache \
-lCQUtil \
$$CIMPORT_MODEL_LIBS \
-lCGeometry3D \
-lCJson \
-lCTclUtil \
$$CIMAGE_LIBS \
-lCRGBUtil \
-lCEncode \
-lGLU \
-ltk -ltcl \
-lz
