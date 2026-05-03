CDEBUG = -g
LDEBUG = -g

PROG = CTclGeometry3D

CPP_SRC = App.cpp

EXTRA_DEPS = \

EXTRA_CFLAGS = \
-I$(INC_DIR)/CTclUtil \
-I$(INC_DIR)/CGeometry3D \
-I$(INC_DIR)/CImportModel \
-I$(INC_DIR)/CMaterial \
-I$(INC_DIR)/CImageLib \
-I$(INC_DIR)/CFont \
-I$(INC_DIR)/CFile \
-I$(INC_DIR)/CFileType \
-I$(INC_DIR)/CNoise \
-I$(INC_DIR)/CMath \
-I$(INC_DIR)/CUtil \
-I$(INC_DIR)/CRGBA \
-I$(INC_DIR)/COS \
-I$(INC_DIR)/CRGBA \

LIBS = \
-lCTclUtil \
-lCGeometry3D -lCCSG \
-lCImportModel \
$(CIMAGE_LIBS) \
-lCNoise \
-lCFile \
-lCRGBUtil \
-lCStrParse \
-lCStrUtil \
-lCPrintF \
-ltk -ltcl

include $(MAKE_DIR)/bin.mk
