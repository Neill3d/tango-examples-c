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
import android.content.ComponentName;
import android.content.ServiceConnection;
import android.graphics.Point;
import android.opengl.GLSurfaceView;
import android.os.Bundle;
import android.os.IBinder;
import android.view.Display;
import android.view.View;
import android.view.View.OnTouchListener;
import android.view.MotionEvent;
import android.view.WindowManager;
import android.widget.Toast;
import android.os.Environment;
import android.util.Log;

import com.projecttango.examples.cpp.util.TangoInitializationHelper;

import java.io.File;
import java.io.FileOutputStream;
import java.io.IOException;

/**
 * The main activity of the application which shows debug information and a
 * glSurfaceView that renders graphic content.
 */
public class MotionTrackingActivity extends Activity {

  // GLSurfaceView and its renderer, all of the graphic content is rendered
  // through OpenGL ES 2.0 in the native code.
  private MotionTrackingRenderer mRenderer;

  private GLSurfaceView mGLView;

  // Screen size for normalizing the touch input for orbiting the render camera.
  private Point mScreenSize = new Point();


  public static final String TAG = MotionTrackingActivity.class.getSimpleName();

  final String downloadPath = Environment.getExternalStorageDirectory() + "/download/";
  final File file = new File(downloadPath);

  private float _touchedX;
  private float _touchedY;

  private View.OnTouchListener mOnTouchListener;

  private void setOnTouchListener(){
    mOnTouchListener = new OnTouchListener() {
      @Override
      public boolean onTouch(View v, MotionEvent event) {

        _touchedX = event.getX();
        _touchedY = event.getY();

        if (event.getAction() == MotionEvent.ACTION_DOWN) {

          TangoJNINative.onTouchEvent(1, event.getAction(),
                  _touchedX, _touchedY, 0.0f, 0.0f);

        } else if (event.getAction() == MotionEvent.ACTION_MOVE) {

          TangoJNINative.onTouchEvent(1, event.getAction(),
                  _touchedX, _touchedY, 0.0f, 0.0f);

        }
        else if (event.getAction() == MotionEvent.ACTION_UP) {

          TangoJNINative.onTouchEvent(1, event.getAction(),
                  _touchedX, _touchedY, 0.0f, 0.0f);
        }

        return true;
      }
    };
  }

  // Tango Service connection.
  ServiceConnection mTangoServiceConnection = new ServiceConnection() {
      @Override
      public void onServiceConnected(ComponentName name, IBinder service) {
        TangoJNINative.onTangoServiceConnected(service);
      }

      @Override
      public void onServiceDisconnected(ComponentName name) {
        // Handle this if you need to gracefully shutdown/retry
        // in the event that Tango itself crashes/gets upgraded while running.
      }
    };

  @Override
  protected void onCreate(Bundle savedInstanceState) {
    super.onCreate(savedInstanceState);

    //String filesDir = this.getFilesDir().getAbsolutePath();

    file.mkdirs();
    TangoJNINative.setupArchiveDir(file.getAbsolutePath());

    TangoJNINative.onCreate(this);

    setContentView(R.layout.activity_motion_tracking);

    // OpenGL view where all of the graphics are drawn.
    mGLView = (GLSurfaceView) findViewById(R.id.gl_surface_view);

    // Configure OpenGL renderer.
    mGLView.setEGLContextClientVersion(2);

    // Configure OpenGL renderer.
    mRenderer = new MotionTrackingRenderer(getAssets());
    mGLView.setRenderer(mRenderer);
    setOnTouchListener();
    mGLView.setOnTouchListener(mOnTouchListener);

    // Check the current screen rotation and set it to the renderer.
    WindowManager mWindowManager = (WindowManager) getSystemService(WINDOW_SERVICE);
    Display mDisplay = mWindowManager.getDefaultDisplay();
    mDisplay.getSize(mScreenSize);

    TangoJNINative.setScreenRotation(mDisplay.getOrientation());
  }

  @Override
  protected void onResume() {
    super.onResume();
    mGLView.onResume();
    TangoInitializationHelper.bindTangoService(this, mTangoServiceConnection);
  }

  @Override
  protected void onPause() {
    super.onPause();
    mGLView.onPause();
    TangoJNINative.onPause();
    unbindService(mTangoServiceConnection);
  }
}
