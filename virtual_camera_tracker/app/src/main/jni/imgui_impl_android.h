#ifndef IMGUI_IMPL_ANDROID_GL2
#define IMGUI_IMPL_ANDROID_GL2

#include "imgui.h"
#include <android/asset_manager_jni.h>

IMGUI_API bool        ImGui_ImplAndroidGLES2_Init(AAssetManager* aasset_manager);
IMGUI_API void        ImGui_ImplAndroidGLES2_Shutdown();
IMGUI_API void        ImGui_ImplAndroidGLES2_NewFrame(int width, int height, unsigned int millis);
//IMGUI_API bool        ImGui_ImplAndroidGLES2_ProcessEvent(SDL_Event* event);

bool ImGui_ImplAndroidGLES2_ProcessTouchEvent(int event, int x, int y);

// Use if you want to reset your rendering device without losing ImGui state.
IMGUI_API void        ImGui_ImplAndroidGLES2_InvalidateDeviceObjects();
IMGUI_API bool        ImGui_ImplAndroidGLES2_CreateDeviceObjects();

#endif // IMGUI_IMPL_ANDROID_GL2