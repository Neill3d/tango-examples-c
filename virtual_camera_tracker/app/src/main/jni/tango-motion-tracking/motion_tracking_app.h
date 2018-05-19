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

#ifndef CPP_MOTION_TRACKING_EXAMPLE_TANGO_MOTION_TRACKING_MOTION_TRACKING_APP_H_
#define CPP_MOTION_TRACKING_EXAMPLE_TANGO_MOTION_TRACKING_MOTION_TRACKING_APP_H_

#include <android/asset_manager.h>
#include <jni.h>
#include <memory>

#include <thread>
#include <string>

#include "StreamThread.h"
#include "NetworkTango.h"

#include <tango_client_api.h>  // NOLINT
#include <tango-gl/util.h>

#include <tango-motion-tracking/scene.h>

namespace tango_motion_tracking {

    enum {
        CAMERA_MODE_NONE,
        CAMERA_MODE_SWITCH,
        CAMERA_MODE_ALIGN,
        CAMERA_MODE_PARENT,
        CAMERA_MODE_COUNT
    };

    struct CTrackValues
    {
        size_t                      ndx;
        float                       lastTime;
        std::vector<float>          values;

        //! a constructor
        CTrackValues()
        {
            ndx = 0;
            lastTime = 0.0f;
            values.resize(128);
        }

        void reset()
        {
            ndx = 0;
            lastTime = 0.0f;
        }

        void addvalue(float value);

        // plot with GUI
        float   *getPointer();
        int     getLength();
    };

    // track average for last 8 seconds
    struct CTrackAvgInt
    {
        size_t              ndx;
        double              lastTime;
        int                 accumValue;
        int                 values[8];

        //! a constructor
        CTrackAvgInt()
        {
            ndx = 0;
            lastTime = 0.0;
            accumValue = 0;
        }

        void addvalue(int value, double timestamp);

        int getAvgValue();
    };

    struct CTrackAvgFloat
    {
        size_t              ndx;
        double              lastTime;
        float               accumValue;
        float               values[8];

        //! a constructor
        CTrackAvgFloat()
        {
            ndx = 0;
            lastTime = 0.0;
            accumValue = 0.0f;
        }

        void addvalue(float value, double timestamp);

        float getAvgValue();
    };


    /////////////////////////////////////////////////////////////////////////////////////////////
    // MotionTrackingApp handles the application lifecycle and resources.
    class MotionTrackingApp {
    public:
        // Constructor and deconstructor.
        MotionTrackingApp();
        ~MotionTrackingApp();

  // OnCreate() callback is called when this Android application's
  // OnCreate function is called from UI thread. In the OnCreate
  // function, we are only checking the Tango Core's version.
  //
  // @param env, java environment parameter OnCreate is being called.
  // @param caller_activity, caller of this function.
  void OnCreate(JNIEnv* env, jobject caller_activity);

  // OnPause() callback is called when this Android application's
  // OnPause function is called from UI thread. In our application,
  // we disconnect Tango Service and free the Tango configuration
  // file. It is important to disconnect Tango Service and release
  // the corresponding resources in the OnPause() callback from
  // Android, otherwise, this application will hold on to the Tango
  // resources and other applications will not be able to connect to
  // Tango Service.
  void OnPause();

    //
    void OnSetupArchiveDir(const char *dir);

  // Call when Tango Service is connected successfully.
  void OnTangoServiceConnected(JNIEnv* env, jobject iBinder);

  // Allocate OpenGL resources for rendering, mainly initializing the Scene.
  void OnSurfaceCreated(AAssetManager* aasset_manager);

  // Set up the viewport width and height.
  void OnSurfaceChanged(int width, int height);

  // Main render loop.
  void OnDrawFrame();

  // Set screen rotation index.
  //
  // @param screen_roatation: the screen rotation index,
  //    the index is following Android screen rotation enum.
  //    see Android documentation for detail:
  //    http://developer.android.com/reference/android/view/Surface.html#TANGO_SUPPORT_ROTATION_0
  void SetScreenRotation(int screen_rotation);

    // UI actions
    void OnButtonToggleClicked();
    void OnButtonResetClicked();
    void OnButtonRecordClicked();

    void OnTouchEvent(int touch_count,
                 tango_gl::GestureCamera::TouchEvent event,
                 float x0, float y0, float x1, float y1);

 private:
  // Setup the configuration file for the Tango Service
  void TangoSetupConfig();

  // Connect to Tango Service.
  // This function will start the Tango Service pipeline, in this case, it will
  // start Motion Tracking.
  void TangoConnect();

  // Disconnect from Tango Service, release all the resources that the app is
  // holding from Tango Service.
  void TangoDisconnect();

  // Release all resources that allocate from the program.
  void DeleteResources();

  // main_scene_ includes all drawable object for visualizing Tango device's
  // movement.
  Scene main_scene_;

  // Tango configration file, this object is for configuring Tango Service setup
  // before connect to service. For example, we set the flag
  // config_enable_auto_recovery based user's input and then start Tango.
  TangoConfig tango_config_;

  // Screen rotation index.
  int screen_rotation_;

  // Flag indicating when the Tango service can be used.
  bool is_service_connected_;
    bool is_gl_initialized_;

    bool mIsTopView;

private:

        bool        mShowStats;
        bool        mShowTransport;
        bool        mShowCamera;
        bool        mShowTrigger;

    int mWidth;
    int mHeight;

        std::unique_ptr<tango_gl::Texture>      mTextureStats;
        std::unique_ptr<tango_gl::Texture>      mTextureTransport;
        std::unique_ptr<tango_gl::Texture>      mTextureCamera;
        std::unique_ptr<tango_gl::Texture>      mTextureTrigger;

        std::unique_ptr<tango_gl::Texture>      mTextureDelete;
        std::unique_ptr<tango_gl::Texture>      mTexturePlay;
        std::unique_ptr<tango_gl::Texture>      mTexturePause;
        std::unique_ptr<tango_gl::Texture>      mTextureStop;
        std::unique_ptr<tango_gl::Texture>      mTextureRecOn;
        std::unique_ptr<tango_gl::Texture>      mTextureRecording;
        std::unique_ptr<tango_gl::Texture>      mTextureRewind;
        std::unique_ptr<tango_gl::Texture>      mTextureLoopOff;
        std::unique_ptr<tango_gl::Texture>      mTextureLoopOn;


        double          mImageFirstTime;
        unsigned int    mImageCounter;
        unsigned int    mImagePerSecond;

    float     mLastImageTimeStamp;

        std::vector<unsigned char>  mImageBuffer;
        std::vector<unsigned char>  mCurImageData;

        bool                    mIsSynced;
        Network::Address        mSyncAddress;

        Network::Socket         mSocketSender;
        double                  mLastSendTimeStamp;

        float                   mLastSyncPoseTimeStamp;
        float                   mLastSyncTimeStamp;

        double          mCurSyncTimestamp;
        double          mCurImageTimestamp;


        Network::Socket         mSocketImage;

        Network::CSyncControl     mSyncState;
        Network::CSyncControl     mInternalState; // receive UI state and extract user changes


        std::vector<Network::CStaticName>       mTakeNames;

        double                                  mCameraInfoTimeStamp;
        int                                     mCameraInfoCounter;
        std::vector<Network::CCameraInfo>        mSceneCameras;

        bool    mNeedRefreshCameraList;

    std::string     mFilesDir;
    std::string     mPosesFile;

        double          mPacketFirstTime;
        unsigned int    mPacketCounter;
        unsigned int    mPacketPerSecond;

        glm::vec3           mLastPos;

        double mLastImageFeedbackTimestamp;

    std::vector<TangoPoseData>      mPoses;
    std::vector<glm::vec3>        mPoints;

        float   mCameraFly;
        float   mCameraLens;
        float   mCameraScale;
        float   mCameraTrigger[6];

    // let's track receving poses delta time to see if there is no lags
    CTrackValues                mTrackPoses;
    CTrackValues                mTrackRender;
    CTrackValues                mTrackRecv;

    // network bandwidth stats

    CTrackAvgInt          mBytesRecv;
    CTrackAvgInt          mBytesSend;

        bool mLastTakeDeletePress;
        bool mLastSyncPress;
        bool mLastRecordPress;
        bool mLastRewindPress;
        bool mLastStopPress;
        bool mLastPlayPress;
        bool mLastLoopPress;
        bool mLastNewCameraPress;

        int mCameraMode;
        bool mTextureUpdate;

    void PrepLog(const char *path);
    void WritePose(const TangoPoseData &data);

    void WriteLog();


    // receive all pending UDP packets and try to update a foreground camera image
    void UpdateTexture(double timestamp);
    void RenderHUD(double timestamp);

        bool SendCommand(float timestamp, int commandId, int commandValue);
        bool SendFeedback(float timestamp);

        bool SendImageFeedback(double timestamp);

public:
    void CollectPose(const TangoPoseData &data);

    // TODO: register clients and send packet to an address with last sync
    void SendPacket(const TangoPoseData &data);

        const double GetSpaceScale() {
            return 0.01 * (double) mCameraScale;
        }
};
}  // namespace tango_motion_tracking

#endif  // CPP_MOTION_TRACKING_EXAMPLE_TANGO_MOTION_TRACKING_MOTION_TRACKING_APP_H_
