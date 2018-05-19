/*
 * Copyright 2014 Google Inc. All Rights Reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

package com.projecttango.examples.cpp.motiontracking;

import android.app.Activity;
import android.content.res.AssetManager;
import android.os.IBinder;
import android.util.Log;

import com.projecttango.examples.cpp.util.TangoInitializationHelper;

/**
 * Interfaces between native C++ code and Java code.
 */
public class TangoJNINative {
  static {
    // This project depends on tango_client_api, so we need to make sure we load
    // the correct library first.
    if (TangoInitializationHelper.loadTangoSharedLibrary() ==
        TangoInitializationHelper.ARCH_ERROR) {
      Log.e("TangoJNINative", "ERROR! Unable to load libtango_client_api.so!");
    }
    System.loadLibrary("cpp_motion_tracking_example");
  }

  /**
   * Interfaces to native OnCreate function.
   *
   * @param callerActivity the caller activity of this function.
   */
  public static native void onCreate(Activity callerActivity);

  /**
   * Called when the Tango service is connected successfully.
   *
   * @param nativeTangoServiceBinder The native binder object.
   */
  public static native void onTangoServiceConnected(IBinder nativeTangoServiceBinder);


  /**
   * Interfaces to native OnPause function.
   */
  public static native void onPause();

  /**
   * Shared an android directory with native code
   */
  public static native void setupArchiveDir(String path);
  /**
   * Allocate OpenGL resources for rendering.
   */
  public static native void onGlSurfaceCreated(AssetManager assetManager);

  /**
   * Setup the view port width and height.
   */
  public static native void onGlSurfaceChanged(int width, int height);

  /**
   * Main render loop.
   */
  public static native void onGlSurfaceDrawFrame();

  /**
   * Set screen rotation index.
   *
   * @param rotationIndex  The screen rotation index,
   *   the index is following Android screen rotation enum.
   *    see Android documentation for detail:
   *    http://developer.android.com/reference/android/view/Surface.html#ROTATION_0
   */
  public static native void setScreenRotation(int rotationIndex);

  // Pass touch events to the native layer.
  public static native void onTouchEvent(int touchCount, int event0,
                                         float x0, float y0, float x1, float y1);

  // UI actions
  public static native void onButtonToggleClicked();
  public static native void onButtonResetClicked();
  public static native void onButtonRecordClicked();
}
