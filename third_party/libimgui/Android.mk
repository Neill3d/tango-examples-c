LOCAL_PATH := $(call my-dir)
 
include $(CLEAR_VARS)
 
LOCAL_MODULE := libimgui
LOCAL_SRC_FILES = imgui.cpp \
                  imgui_demo.cpp \
                  imgui_draw.cpp \
                  

LOCAL_EXPORT_LDLIBS := -lz
 
include $(BUILD_STATIC_LIBRARY)
