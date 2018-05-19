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

// motion_tracking_app.cc
//

#include <tango_support.h>

#include <tango-gl/conversions.h>
#include <android/asset_manager_jni.h>
#include <tango_client_api.h>
#include "tango-motion-tracking/motion_tracking_app.h"

#include "android/native_activity.h"
#include <fcntl.h>

#include "NetworkUtils.h"
#include "NetworkTango.h"
#include "UDPThread.h"
#include "imgui_impl_android.h"

#include "zlib.h"

namespace tango_motion_tracking {

#define TRASNPORT_PLAY_SPEED "0.10x\0 0.20x\0 0.25x\0 0.35x\0 0.50x\0 1x\0 All frames\0 2x\0 3x\0 4x\0 5x\0 10x\0"

    ///////////////////////////////////////////////////////////////////////////////////////////////

// The minimum Tango Core version required from this application.
constexpr int kTangoCoreMinimumVersion = 9377;
    constexpr int kImageReceivePort = 8887;


    void CTrackValues::addvalue(float value)
    {
        if (0 == ndx)
        {
            values[0] = value;
            lastTime = value;
            ndx += 1;
        } else
        {
            float curr = value - lastTime;

            values[ndx] = curr;
            ndx += 1;

            if (ndx >= values.size())
            {
                size_t halfSize = values.size() / 2;

                memcpy(&values[0], &values[halfSize], sizeof(float)*(halfSize-1));

                ndx = halfSize;
            }

            lastTime = value;
        }
    }

    float *CTrackValues::getPointer()
    {
        size_t pos = 1;
        size_t halfSize = values.size() / 2;

        if (ndx > halfSize)
            pos = ndx - halfSize;

        return &values[pos];
    }

    int CTrackValues::getLength()
    {
        int len = (int)values.size() / 2;

        if (ndx < len)
            len = ndx;

        return len;
    }

    void CTrackAvgInt::addvalue(int value, double timestamp)
    {

        if (lastTime > 0.0)
        {
            if ( (timestamp - lastTime) < 1.0 )
            {
                accumValue += value;
            }
            else
            {
                values[ndx] = accumValue;
                ndx += 1;
                if (ndx >= 8)
                    ndx = 0;

                accumValue = 0;
                lastTime = timestamp;
            }
        }
        else
        {
            ndx = 0;
            for (int i=0; i<8; ++i)
                values[i] = value;
            accumValue = 0;
            lastTime = timestamp;
        }
    }

    int CTrackAvgInt::getAvgValue()
    {
        int res = 0;

        if (lastTime > 0.0)
        {
            for (int i=0; i<8; ++i)
                res += values[i];
            res /= 8;
        }
        return res;
    }

    void CTrackAvgFloat::addvalue(float value, double timestamp)
    {

        if (lastTime > 0.0)
        {
            if ( (timestamp - lastTime) < 1.0 )
            {
                accumValue += value;
            }
            else
            {
                values[ndx] = accumValue;
                ndx += 1;
                if (ndx >= 8)
                    ndx = 0;
            }
        }
        else
        {
            ndx = 0;
            for (int i=0; i<8; ++i)
                values[i] = value;
        }

        lastTime = timestamp;
    }

    float CTrackAvgFloat::getAvgValue()
    {
        float res = 0.0f;

        if (lastTime > 0.0)
        {
            for (int i=0; i<8; ++i)
                res += values[i];
            res /= 8.0f;
        }
        return res;
    }


    //////////////////////////////////////////////////////////////////////////////////////////////
    //

MotionTrackingApp::MotionTrackingApp()
{
    is_gl_initialized_ = false;

    mShowStats = true;
    mShowTransport = true;
    mShowCamera = false;
    mShowTrigger = false;

    mIsTopView = false;

    mIsSynced = false;
    mLastSendTimeStamp = 0.0;
    mCameraInfoTimeStamp = 0.0;

    mLastSyncTimeStamp = 0.0f;

    mLastSyncPoseTimeStamp = 0.0f;

    //
    mLastImageTimeStamp = 0.0f;
    mImageBuffer.resize(MAX_IMAGE_SIZE+8);

    mImageFirstTime = 0.0;
    mImageCounter = 0;
    mImagePerSecond = 0;

    //
    mSyncState.startFrame = 0;
    mSyncState.stopFrame = 100;
    mSyncState.currFrame = 30;
    mSyncState.takeCount = 1;

    mCurSyncTimestamp = 0.0;
    mCurImageTimestamp = 0.0;

    mPacketFirstTime = 0.0;
    mPacketCounter = 0;
    mPacketPerSecond = 0;

    mCameraFly = 0.0f;
    mCameraLens = 40.0f;
    mCameraScale = 100.0f;

    mLastImageFeedbackTimestamp = 0.0;

    mNeedRefreshCameraList = true;

    mLastTakeDeletePress = false;
    mLastSyncPress = false;
    mLastRecordPress = false;
    mLastRewindPress = false;
    mLastStopPress = false;
    mLastPlayPress = false;
    mLastLoopPress = false;
    mLastNewCameraPress = false;

    mCameraMode = CAMERA_MODE_SWITCH;
mTextureUpdate = false;

    for (int i=0; i<6; ++i)
        mCameraTrigger[i] = 0.0f;

    std::thread lthread(Network::MainServerFunc);
    lthread.detach();
}

MotionTrackingApp::~MotionTrackingApp()
{
    //
    //mThreadSync.WaitForInternalThreadToExit();

    // TODO: thread / socket close
    mSocketSender.Close();
    mSocketImage.Close();

    //
  if (tango_config_ != nullptr) {
    TangoConfig_free(tango_config_);
  }

    // TODO: free global ref from assets !

    ImGui_ImplAndroidGLES2_Shutdown();
}

    void MotionTrackingApp::PrepLog(const char *path)
    {
        mPosesFile = path;
        mPosesFile = mPosesFile + "/poses.txt";

        FILE *fp = fopen(mPosesFile.c_str(), "w+");
        if (fp != NULL)
        {
            fputs("Project Tango Poses", fp);
            fputc('\n', fp);
            fputs("-------------------", fp);
            fputc('\n', fp);

            fflush(fp);
            fclose(fp);

        }
    }

    void MotionTrackingApp::WritePose(const TangoPoseData &data)
    {
        char buffer[512] = {0};

        FILE *fp = fopen(mPosesFile.c_str(), "a");
        if (fp != NULL)
        {
            sprintf(buffer, "%.2lf %.2lf %.2lf %.2lf %.2lf %.2lf %.2lf %.2lf", data.timestamp, data.translation[0],
                data.translation[1], data.translation[2], data.orientation[0], data.orientation[1],
                data.orientation[2], data.orientation[3]);

            fputs(buffer, fp);
            fputc('\n', fp);

            fflush(fp);
            fclose(fp);
        }
    }

    void MotionTrackingApp::WriteLog()
    {
        if (0 == mPoses.size())
            return;

        char buffer[512] = {0};
        int firstStamp = (int) mPoses[0].timestamp;
        sprintf(buffer, "/poses_%d.txt", firstStamp);

        std::string fname = mFilesDir;
        fname = fname + buffer;

        FILE *fp = fopen(fname.c_str(), "w+");
        if (fp != NULL)
        {
            fputs("Project Tango Poses", fp);
            fputc('\n', fp);
            fputs("-------------------", fp);
            fputc('\n', fp);

            for (auto iter=begin(mPoses); iter!=end(mPoses); ++iter)
            {
                TangoPoseData &data = *iter;

                sprintf(buffer, "%.2lf %.2lf %.2lf %.2lf %.2lf %.2lf %.2lf %.2lf", data.timestamp, data.translation[0],
                        data.translation[1], data.translation[2], data.orientation[0], data.orientation[1],
                        data.orientation[2], data.orientation[3]);

                fputs(buffer, fp);
                fputc('\n', fp);
            }

            fflush(fp);
            fclose(fp);
        }
    }

    void MotionTrackingApp::CollectPose(const TangoPoseData &data)
    {

        glm::vec3 v = tango_gl::conversions::Vec3FromArray(data.translation);

        if (mPoses.size() >= 512)
        {
            mPoses.back() = data;
            mPoints.back() = v;
        } else {
            mPoses.push_back(data);
            mPoints.push_back(v);
        }

        // stats
        mTrackPoses.addvalue( (float)data.timestamp);

        if (0.0 == mPacketFirstTime)
            mPacketFirstTime = data.timestamp;

        mPacketCounter += 1;

        int secs = (int) floor(data.timestamp - mPacketFirstTime);
        if (secs < 1)
            secs = 1;

        mPacketPerSecond = mPacketCounter / secs;
    }



void MotionTrackingApp::OnCreate(JNIEnv* env, jobject activity) {
  // Check the installed version of the TangoCore.  If it is too old, then
  // it will not support the most up to date features.
  int version;
  TangoErrorType err = TangoSupport_getTangoVersion(env, activity, &version);
  if (err != TANGO_SUCCESS || version < kTangoCoreMinimumVersion) {
    LOGE("MotionTrackingApp::OnCreate, Tango Core version is out of date.");
    std::exit(EXIT_SUCCESS);
  }
    /*
    // get asset manager pointer
    jclass activity_class = env->GetObjectClass(activity);

    jmethodID activity_class_getAssets = env->GetMethodID(activity_class, "getAssets", "()Landroid/content/res/AssetManager;");
    jobject asset_manager = env->CallObjectMethod(activity, activity_class_getAssets);
    jobject global_asset_manager = env->NewGlobalRef(asset_manager);

    AAssetManager *pAssetManager = AAssetManager_fromJava(env, global_asset_manager);
*/


    mLastSyncTimeStamp = 0.0f;

    mSocketSender.Open(NETWORK_MOBU_TRACKER, false);
    mSocketImage.Open(NETWORK_IMAGE_PORT, false);

  is_service_connected_ = false;
}

// Initialize Tango.
void MotionTrackingApp::OnTangoServiceConnected(JNIEnv* env, jobject iBinder) {
  TangoErrorType ret = TangoService_setBinder(env, iBinder);
  if (ret != TANGO_SUCCESS) {
    LOGE(
        "MotionTrackingApp: OnTangoServiceConnected, TangoService_setBinder "
        "error");
    std::exit(EXIT_SUCCESS);
  }

  TangoSetupConfig();
  TangoConnect();

        //PrepLog(mFilesDir.c_str());

  is_service_connected_ = true;

        mTrackPoses.reset();
}

void MotionTrackingApp::TangoSetupConfig() {
  // Here, we'll configure the service to run in the way we'd want. For this
  // application, we'll start from the default configuration
  // (TANGO_CONFIG_DEFAULT). This enables basic motion tracking capabilities.
  tango_config_ = TangoService_getConfig(TANGO_CONFIG_DEFAULT);
  if (tango_config_ == nullptr) {
    LOGE("MotionTrackingApp: Failed to get default config form");
    std::exit(EXIT_SUCCESS);
  }

  // Set auto-recovery for motion tracking as requested by the user.
  int ret =
      TangoConfig_setBool(tango_config_, "config_enable_auto_recovery", true);
  if (ret != TANGO_SUCCESS) {
    LOGE(
        "MotionTrackingApp: config_enable_auto_recovery() failed with error"
        "code: %d",
        ret);
    std::exit(EXIT_SUCCESS);
  }

    // Set drift-corrected mode
    ret =
            TangoConfig_setBool(tango_config_, "config_enable_drift_correction", false);
    if (ret != TANGO_SUCCESS) {
        LOGE(
                "MotionTrackingApp: config_enable_auto_recovery() failed with error"
                        "code: %d",
                ret);
        std::exit(EXIT_SUCCESS);
    }

    // Low latency IMU integration enables aggressive integration of the latest
    // inertial measurements to provide lower latency pose estimates. This will
    // improve the AR experience.
    ret = TangoConfig_setBool(tango_config_,
                              "config_enable_low_latency_imu_integration", false);
    if (ret != TANGO_SUCCESS) {
        LOGE(
                "VideoStabilizationApp: config_enable_low_latency_imu_integration() "
                        "failed with error code: %d",
                ret);
        std::exit(EXIT_SUCCESS);
    }
}

// Connect to Tango Service, service will start running, and
// pose can be queried.

    void OnPoseAvaliable(void* context, const TangoPoseData* pose)
    {
        if (nullptr != context)
        {
            MotionTrackingApp *app = (MotionTrackingApp*) context;
            const double scale = app->GetSpaceScale();

            TangoPoseData oglPose = *pose;

            glm::vec3 tangoT = tango_gl::conversions::Vec3FromArray(pose->translation);
            glm::quat tangoQ = tango_gl::conversions::QuatFromArray(pose->orientation);

            glm::vec3 tr = tango_gl::conversions::Vec3TangoToGl(tangoT);
            glm::quat q = tango_gl::conversions::QuatTangoToGl(tangoQ);

            for (int i=0; i<3; ++i)
            {
                oglPose.translation[i] = scale * (double)tr[i];
                oglPose.orientation[i] = (double)q[i];
            }
            oglPose.orientation[3] = (double)q[3];

            //
            app->CollectPose(oglPose);
            //app->SendPacket(oglPose);
        }
    }


void MotionTrackingApp::TangoConnect() {
  TangoErrorType ret = TangoService_connect(this, tango_config_);
  if (ret != TANGO_SUCCESS) {
    LOGE(
        "MotionTrackingApp: Failed to connect to the Tango service with"
        "error code: %d",
        ret);
    std::exit(EXIT_SUCCESS);
  }

        TangoCoordinateFramePair pair;
        pair.target = TANGO_COORDINATE_FRAME_DEVICE;
        pair.base = TANGO_COORDINATE_FRAME_START_OF_SERVICE;

        if (TANGO_SUCCESS != TangoService_connectOnPoseAvailable(1, &pair, OnPoseAvaliable ) )
        {
            LOGE(
                    "MotionTrackingApp: Failed to connect on pose available"
                            "error code: %d",
                    ret);
            std::exit(EXIT_SUCCESS);
        }

}

void MotionTrackingApp::OnPause() {
  TangoDisconnect();
  DeleteResources();
}

void MotionTrackingApp::TangoDisconnect() {
  // When disconnecting from the Tango Service, it is important to make sure to
  // free your configuration object. Note that disconnecting from the service,
  // resets all configuration, and disconnects all callbacks. If an application
  // resumes after disconnecting, it must re-register configuration and
  // callbacks with the service.
  is_service_connected_ = false;
  TangoConfig_free(tango_config_);
  tango_config_ = nullptr;
  TangoService_disconnect();


}

void MotionTrackingApp::OnSurfaceCreated(AAssetManager* aasset_manager)
{
  TangoSupport_initialize(TangoService_getPoseAtTime,
                          TangoService_getCameraIntrinsics);
  main_scene_.InitGLContent(aasset_manager);

    ImGui_ImplAndroidGLES2_Init(aasset_manager);

    is_gl_initialized_ = true;

    // load button textures
    mTextureStats.reset(new tango_gl::Texture(aasset_manager, "browsing/ui_template_details.png") );
    mTextureTransport.reset(new tango_gl::Texture(aasset_manager, "browsing/template_story_story-l.png") );
    mTextureCamera.reset(new tango_gl::Texture(aasset_manager, "browsing/template_model_camera-l.png") );
    mTextureTrigger.reset(new tango_gl::Texture(aasset_manager, "browsing/openreality_trigger-l.png") );

    mTextureDelete.reset(new tango_gl::Texture(aasset_manager, "tape/tape_clear_on.png") );
    mTexturePlay.reset(new tango_gl::Texture(aasset_manager, "tape/tape_play_on.png") );
    mTexturePause.reset(new tango_gl::Texture(aasset_manager, "tape/tape_still_on.png"));
    mTextureStop.reset(new tango_gl::Texture(aasset_manager, "tape/tape_stop_off.png"));
    mTextureRecOn.reset(new tango_gl::Texture(aasset_manager, "tape/tape_rec_on.png"));
    mTextureRecording.reset(new tango_gl::Texture(aasset_manager, "tape/tape_rec_recording.png"));
    mTextureLoopOff.reset(new tango_gl::Texture(aasset_manager, "tape/tape_loop_off.png"));
    mTextureLoopOn.reset(new tango_gl::Texture(aasset_manager, "tape/tape_loop_on.png"));
    mTextureRewind.reset(new tango_gl::Texture(aasset_manager, "tape/tape_start_on.png"));


    LOGI("Texture play id - %d", mTexturePlay->GetTextureID());
}

void MotionTrackingApp::OnSurfaceChanged(int width, int height) {

    mWidth = width;
    mHeight = height;

  main_scene_.SetupViewPort(width, height);

    // for test only
    int value = (width << 16) | (height & 0xFFFF);
    SendCommand(0.0f, PACKET_COMMAND_SCREEN_SIZE, value);
}

void MotionTrackingApp::OnDrawFrame() {
    if (!is_service_connected_) {
        return;
    }

    TangoPoseData pose;
    pose.status_code = TANGO_POSE_INVALID;
/*
  TangoSupport_getPoseAtTime(
      0.0, TANGO_COORDINATE_FRAME_START_OF_SERVICE,
      TANGO_COORDINATE_FRAME_DEVICE, TANGO_SUPPORT_ENGINE_OPENGL,
      TANGO_SUPPORT_ENGINE_OPENGL,
      static_cast<TangoSupport_Rotation>(screen_rotation_), &pose);

*/

/*
    // pose with drift correction feature
    TangoSupport_getPoseAtTime(
            0.0, TANGO_COORDINATE_FRAME_AREA_DESCRIPTION,
            TANGO_COORDINATE_FRAME_DEVICE, TANGO_SUPPORT_ENGINE_OPENGL,
            TANGO_SUPPORT_ENGINE_OPENGL,
            static_cast<TangoSupport_Rotation>(screen_rotation_), &pose);
*/

    if (mPoses.size() > 0) {
        pose = mPoses.back();

        mLastPos.x = (float) pose.orientation[0];
        mLastPos.y = (float) pose.orientation[1];
        mLastPos.z = (float) pose.orientation[2];
    }

    if (pose.status_code != TANGO_POSE_VALID) {
        LOGE("MotionTrackingApp: Tango pose is not valid.");
        return;
    }

    uint32_t clientAddress = Network::GetLastSyncAddress();
    mIsSynced = (clientAddress > 0);
    if (clientAddress > 0) {
        mSyncAddress.Set(clientAddress, 8889);

        if (0.0 == mLastSendTimeStamp || (pose.timestamp - mLastSendTimeStamp) > 0.02) {
            SendPacket(pose);
            mLastSendTimeStamp = pose.timestamp;
        }
    }

    // TODO: get last image from image thread
    UpdateTexture(pose.timestamp);

    // Rotate the logo cube related with the delta time
    main_scene_.RotateCubeByPose(pose);

    main_scene_.Render(pose);

    // render an accumulated trajectory
    main_scene_.RenderTrajectory(mPoints);

    // render GUI layer
    mTrackRender.addvalue(1000.0f * (float) pose.timestamp);

    // sync states if new data arrived
    //LOGI("Get sync data");

    double timestamp = Network::GetLastSyncTimestamp();
    if (timestamp > mCurSyncTimestamp) {
        Network::GetLastSyncState(mSyncState);
        mCurSyncTimestamp = timestamp;
    }

    /*
    if (true == mThreadSync.GetSyncData(mLastSyncTimeStamp, mLastSyncTimeStamp, mSyncAddress, mSyncState) )
    {
        mLastSyncPoseTimeStamp = pose.timestamp;
        mSyncAddress.SetPortOnly(NETWORK_MOBU_COMMANDS);
    }
*/
    //Network::GetLastSyncState(mSyncState);

    // we've received last packet in nearest 3 seconds
/*
    bool lsynced = (pose.timestamp - mLastSyncPoseTimeStamp < 3.0);

    if (lsynced)
    {
        if (!mIsSynced)
        {
            mIsSynced = true;
            SendFeedback(pose.timestamp);
        }
    } else{
        mIsSynced = false;
    }
*/
    // check if we receive new scene info
    if (mIsSynced)
    {
        const double camstamp = Network::GetLastSyncCamerasTimeStamp();

        if (0.0 == mCameraInfoTimeStamp || mCameraInfoTimeStamp < camstamp)
        {
            mSceneCameras = Network::GetLastSyncCamerasVector();
            mCameraInfoTimeStamp = camstamp;
        }

        //mThreadSync.GetCameraData(mCameraInfoTimeStamp, mCameraInfoCounter, mSceneCameras);
    }

    RenderHUD(pose.timestamp);


    //CollectPose(pose);

}

void MotionTrackingApp::DeleteResources() { main_scene_.DeleteResources(); }

void MotionTrackingApp::SetScreenRotation(int screen_rotation) {
  screen_rotation_ = screen_rotation;
}


    void MotionTrackingApp::OnSetupArchiveDir(const char *dir) {
        mFilesDir = dir;
    }

    void MotionTrackingApp::OnButtonToggleClicked() {
        mIsTopView = !mIsTopView;
    }

    void MotionTrackingApp::OnButtonResetClicked() {
        // DONE: do a reset
        TangoService_resetMotionTracking();

        mPoses.clear();
        mPoints.clear();
    }

    void MotionTrackingApp::OnButtonRecordClicked() {
        // DONE: record collected poses and start new recording

        if ( mPoses.size() > 0 )
        {
            WriteLog();
            mPoses.clear();
            mPoints.clear();
        }
    }

    static int sendCounter = 1;

    void MotionTrackingApp::SendPacket(const TangoPoseData &data)
    {
        // DONE: connect address to the last receiving address

        Network::CDeviceData lDeviceData;

        for (int i=0; i<3; ++i)
        {
            lDeviceData.translation[i] = (float) data.translation[i];
            lDeviceData.orientation[i] = (float) data.orientation[i];
        }
        lDeviceData.orientation[3] = (float) data.orientation[3];

        // custom values
        lDeviceData.lens = mCameraLens;
        lDeviceData.moveMult = mCameraScale;
        lDeviceData.fly = mCameraFly;

        for (int i=0; i<6; ++i)
        {
            lDeviceData.triggers[i] = (unsigned char) ((mCameraTrigger[i] > 0.0f) ? 1 : 0);
        }

        if (mIsSynced && mInternalState.liveMode > 0)
        {
            Network::Address address(mSyncAddress.GetD(), mSyncAddress.GetC(), mSyncAddress.GetB(), mSyncAddress.GetA(), NETWORK_MOBU_TRACKER);

            Network::CPacketDevice  packet;
            Network::FillBufferWithHeader( (unsigned char*) &packet.header, PACKET_REASON_CAMERA, (float)data.timestamp);
            packet.body = lDeviceData;

            if (true == mSocketSender.Send(address, &packet, sizeof(Network::CPacketDevice)) )
            {
                mBytesSend.addvalue( sizeof(Network::CPacketDevice), data.timestamp );
            }
        }

/*
        if (sendCounter < 5)
        {
            sendCounter += 1;
            return;
        } else{
            sendCounter = 0;
        }
*/
        //Network::ExchangeWriteDeviceData(data.timestamp, lDeviceData);
        //Network::SendPoseImid(data.timestamp, lDeviceData);
        /*
        if (false == mIsSynced)
            return;

        if (sendCounter < 5)
        {
            sendCounter += 1;
            return;
        } else{
            sendCounter = 1;
        }

        Network::CPacketDevice packet;
        packet.header.timestamp = (float) data.timestamp;
        packet.header.reason = PACKET_REASON_CAMERA;
        packet.header.magic[0] = PACKET_MAGIC_1;
        packet.header.magic[1] = PACKET_MAGIC_2;
        packet.header.magic[2] = PACKET_MAGIC_3;

        //packet.body.timestamp = data.timestamp;
        //packet.body.cameraNdx = 0;

        for (int i=0; i<3; ++i)
        {
            packet.body.translation[i] = data.translation[i];
            packet.body.orientation[i] = data.orientation[i];
        }
        packet.body.orientation[3] = data.orientation[3];

        // custom values
        packet.body.lens = mCameraLens;
        packet.body.moveMult = mCameraScale;
        packet.body.fly = mCameraFly;

        for (int i=0; i<6; ++i)
        {
            packet.body.triggers[i] = (unsigned char) ((mCameraTrigger[i] > 0.0f) ? 1 : 0);
        }*/
/*
        Network::Address address(mSyncAddress.GetAddress(), NETWORK_MOBU_TRACKER);

        if (true == mSocketSender.Send(address, &packet, sizeof(Network::CPacketDevice)) )
        {
            mBytesSend.addvalue( sizeof(Network::CPacketDevice), data.timestamp );
        }
        */
    }

    void MotionTrackingApp::OnTouchEvent(int touch_count,
                                     tango_gl::GestureCamera::TouchEvent event,
                                     float x0, float y0, float x1, float y1) {
        if (!is_service_connected_ || !is_gl_initialized_) {
            return;
        }

        if (touch_count > 0)
        {
            int button = 0;
            if (tango_gl::GestureCamera::kTouch0Down == event)
                button = 1;
            else if (tango_gl::GestureCamera::kTouch0Up == event)
                button = 2;

            int x = (int) (x0); // * mWidth);
            int y = (int) (y0); // * mHeight);

            ImGui_ImplAndroidGLES2_ProcessTouchEvent(button, x, y);

            //int value = (x << 16) | (y & 0xFFFF);
            //SendCommand(0.0f, PACKET_COMMAND_SCREEN_POS, value);
        }
        //main_scene_.OnTouchEvent(touch_count, event, x0, y0, x1, y1);
    }


    void MotionTrackingApp::UpdateTexture(double timestamp) {


        if (false == mTextureUpdate)
            return;

        // check if we have received a new background/foreground texture
        int counter = 0;
        int receivedImage = 0;
        int received_bytes = 0;

        Network::Address lastAddress;
        //Network::CImageHeader header;

        if (0.0 == mImageFirstTime)
            mImageFirstTime = timestamp;

        double temp = mLastImageTimeStamp;


        if (true == mSocketImage.IsOpen())
        {
            int recv = 1;
            int numberOfPackets = 0;

            while (recv > 0)
            {
                recv = mSocketImage.Receive(lastAddress, mImageBuffer.data(), mImageBuffer.size());

                if (recv > 0)
                    numberOfPackets += 1;
            }

            // read and apply the latest packet

            if (numberOfPackets > 0)
            {
                Network::CHeader    *pHeader = (Network::CHeader*) mImageBuffer.data();

                if (Network::CheckMagicNumber(mImageBuffer.data()) && PACKET_REASON_IMAGE == pHeader->reason)
                {
                    Network::CImageHeader *pImageHeader = (Network::CImageHeader*) (mImageBuffer.data() + sizeof(Network::CHeader));

                    unsigned char *ptr = mImageBuffer.data() + sizeof(Network::CHeader) + sizeof(Network::CImageHeader);

                    if (pImageHeader->compressed > 0)
                    {
                        uLongf imageSize = pImageHeader->dataSize;
                        mCurImageData.resize(imageSize);

                        int err = uncompress(mCurImageData.data(), &imageSize, ptr, pImageHeader->compressed);

                        if (Z_OK == err)
                        {
                            main_scene_.UpdateDynamicTexture(pImageHeader->aspect, pImageHeader->internalFormat, pImageHeader->width,
                                                             pImageHeader->height, mCurImageData.data(), imageSize);
                        }
                    }
                }
            }
        }

        //
/*
        double stamp = Network::GetLastImageTimestamp();
        if (0.0f == mLastImageTimeStamp || stamp > mLastImageTimeStamp)
        {
            mLastImageTimeStamp = stamp;

            //Network::CImageHeader imageHeader;
            Network::GetLastImageHeader(header);
            size_t imageSize = Network::GetLastImageSize();

            mCurImageData.resize(imageSize);
            Network::GetLastImageData(mCurImageData.data());

            if (true == mTextureUpdate)
            {
                main_scene_.UpdateDynamicTexture(header.aspect, header.internalFormat, header.width,
                                                 header.height, mCurImageData.data(), imageSize);
            }
        }
*/
        /*
        if (true ==
            mThreadImage.GetLastImageData(mLastImageTimeStamp, mLastImageTimeStamp, lastAddress,
                                          header, (char *) mImageBuffer.data(),
                                          mImageBuffer.size())) {

            unsigned char *ptr = mImageBuffer.data();
            unsigned int imageSize = header.dataSize;

            // update dynamic texture
            //LOGE( "[App] Update dynamic texture!");
            if (true == mTextureUpdate)
            {
                main_scene_.UpdateDynamicTexture(header.aspect, header.internalFormat, header.width,
                                         header.height, ptr, imageSize);

            }
            mImageCounter += 1;
            mTrackRecv.addvalue(mLastImageTimeStamp - temp);
        }
*/
        int secs = (int) floor(timestamp - mImageFirstTime);
        if (secs < 1)
            secs = 1;

        mImagePerSecond = mImageCounter / secs;

        // send image feedback
        SendImageFeedback(timestamp);
    }

    bool TakeGetName(void *userData, const int index, const char **text)
    {
        if (nullptr == userData)
            return false;

        std::vector<Network::CStaticName> *pNames = (std::vector<Network::CStaticName>*) userData;
        *text = pNames->at(index).raw;

        return true;
    }

    void MotionTrackingApp::RenderHUD(double timestamp) {
        unsigned int millis = (unsigned int) (1000.0 * timestamp);
        ImGui_ImplAndroidGLES2_NewFrame(mWidth, mHeight, millis);

        mInternalState = mSyncState;

        bool takeDeletePress = false;
        bool syncPress = false;
        bool recordPress = false;
        bool rewindPress = false;
        bool stopPress = false;
        bool playPress = false;
        bool loopPress = false;

        bool newCameraPress = false;

        bool triggers[6] = {false};

        {
            int flags = ImGuiWindowFlags_NoTitleBar | ImGuiWindowFlags_NoResize | ImGuiWindowFlags_NoSavedSettings;

            ImGui::Begin("##Toolbar", nullptr, ImVec2(130.0f, 450.0f), -0.5f, flags);
            ImGui::SetWindowPos(ImVec2(5.0f, 5.0f), ImGuiCond_Always);

            ImVec2 buttonSize(100.0f, 100.0f);

            bool pressStats = false;
            bool pressTransport = false;
            bool pressCamera = false;
            bool pressTriggers = false;

            // stats
            if (nullptr != mTextureStats.get())
                pressStats = ImGui::ImageButton((ImTextureID) (uintptr_t) mTextureStats->GetTextureID(), buttonSize,
                                                ImVec2(0.0f, 0.0f), ImVec2(0.5f, 0.5f));
            else
                pressStats = ImGui::Button("Stats", buttonSize );

            // transport
            if (nullptr != mTextureTransport.get())
                pressTransport = ImGui::ImageButton((ImTextureID) (uintptr_t) mTextureTransport->GetTextureID(), buttonSize,
                                                    ImVec2(0.0f, 0.0f), ImVec2(0.5f, 0.5f));
            else
                pressTransport = ImGui::Button("Transport", buttonSize);

            // camera
            if (nullptr != mTextureCamera.get())
                pressCamera = ImGui::ImageButton((ImTextureID) (uintptr_t) mTextureCamera->GetTextureID(), buttonSize,
                                                 ImVec2(0.0f, 0.0f), ImVec2(0.5f, 0.5f));
            else
                pressCamera = ImGui::Button("Camera", buttonSize);

            // triggers
            if (nullptr != mTextureTrigger.get())
                pressTriggers = ImGui::ImageButton((ImTextureID) (uintptr_t) mTextureTrigger->GetTextureID(), buttonSize,
                                                   ImVec2(0.0f, 0.0f), ImVec2(0.5f, 0.5f));
            else
                pressTriggers = ImGui::Button("Triggers", buttonSize);

            ImGui::End();

            if (pressStats) mShowStats = !mShowStats;
            if (pressTransport) mShowTransport = !mShowTransport;
            if (pressCamera) mShowCamera = !mShowCamera;
            if (pressTriggers) mShowTrigger = !mShowTrigger;
        }


        if (mShowCamera)
        {
            if (mIsSynced && mNeedRefreshCameraList)
            {
                SendCommand(timestamp, PACKET_COMMAND_CAMERALIST_REQ, 1);
                mNeedRefreshCameraList = false;
            }

            ImGui::Begin("Camera Controls", nullptr, ImVec2(800.0f, 600.0f), -1.0f, ImGuiWindowFlags_NoScrollbar);
            ImGui::SetWindowPos(ImVec2(150.0f, 175.0f), ImGuiCond_Once);
            //ImGui::Text("Movement scale, focal lens, top view");

            float halfwidth = 0.5f * ImGui::GetWindowWidth();
            float height = ImGui::GetWindowHeight();

            const float spacing = 10.0f;
            ImGui::PushStyleVar(ImGuiStyleVar_ItemSpacing, ImVec2(spacing, spacing));

            ImGui::Columns(2, "camera_controls", true);
            ImGui::Separator();

            ImGui::PushStyleVar(ImGuiStyleVar_GrabMinSize, 40.0f);

            if (false == ImGui::VSliderFloat("##fly", ImVec2(0.28f * halfwidth, height - 80.0f), &mCameraFly, -100.0f, 100.0f, "%.2f\nFly") )
            {
                mCameraFly = 0.0f;
            }

            ImGui::SameLine();
            ImGui::VSliderFloat("##scale", ImVec2(0.28f * halfwidth, height - 80.0f), &mCameraScale, 1.0f, 400.0f, "%.2f\nScale");

            ImGui::SameLine();
            ImGui::VSliderFloat("##lens", ImVec2(0.28f * halfwidth, height - 80.0f), &mCameraLens, 5.0f, 120.0f, "%.2f\nFOV");

            ImGui::PopStyleVar();

            ImGui::NextColumn();

            float buttonw = halfwidth - 20.0f;

            if (true == ImGui::Button("Reset Tracker", ImVec2(0.5f * buttonw, 60.0f)) )
            {
                OnButtonResetClicked();
            }
            ImGui::SameLine();
            newCameraPress = ImGui::Button("New Camera", ImVec2(0.5f * buttonw, 60.0f));
            ImGui::Separator();

            //ImGui::Text("Align To Scene Camera:");

            if (true == ImGui::Selectable("Switch To", (CAMERA_MODE_SWITCH == mCameraMode), 0, ImVec2(0.33f * buttonw, 40.0f)) )
            {
                mCameraMode = CAMERA_MODE_SWITCH;
            }
            ImGui::SameLine();
            if (true == ImGui::Selectable("Align To", (CAMERA_MODE_ALIGN == mCameraMode), 0, ImVec2(0.33f * buttonw, 40.0f)) )
            {
                mCameraMode = CAMERA_MODE_ALIGN;
            }
            ImGui::SameLine();
            if (true == ImGui::Selectable("Parent To", (CAMERA_MODE_PARENT == mCameraMode), 0, ImVec2(0.33f * buttonw, 40.0f)) )
            {
                mCameraMode = CAMERA_MODE_PARENT;
            }

            // output a scene camera list to align virtual camera for
            float listBoxHeight = height - 210.0f;
            if (listBoxHeight < 1.0f)
                listBoxHeight = 5.0f;

            ImGuiStyle &style = ImGui::GetStyle();
            float temp = style.ScrollbarSize;
            style.ScrollbarSize = 80.0f;

            ImGui::ListBoxHeader("##cameras", ImVec2(buttonw, listBoxHeight));

            int alignId = -2;

            if (CAMERA_MODE_ALIGN == mCameraMode || CAMERA_MODE_PARENT == mCameraMode)
            {
                if (true == ImGui::Selectable("Reset Position / Parent"))
                {
                    alignId = -1;
                }
            }

            for (auto iter=begin(mSceneCameras); iter!=end(mSceneCameras); ++iter)
            {
                if (true == ImGui::Selectable(iter->name.raw) )
                {
                    // DONE: align/parent to that camera
                    alignId = iter->id;
                }
            }

            ImGui::ListBoxFooter();

            style.ScrollbarSize = temp;

            if (alignId >= -1)
            {
                if (CAMERA_MODE_SWITCH == mCameraMode)
                {
                    SendCommand(timestamp, PACKET_COMMAND_CAMERA_SWITCH, alignId);
                }
                if (CAMERA_MODE_ALIGN == mCameraMode)
                {
                    SendCommand(timestamp, PACKET_COMMAND_CAMERA_ALIGN, alignId);
                }
                else if (CAMERA_MODE_PARENT == mCameraMode)
                {
                    SendCommand(timestamp, PACKET_COMMAND_CAMERA_PARENT, alignId);
                }
            }

            //ImGui::Button("Top View", ImVec2(buttonw, 80));
            //ImGui::Button("Reset", ImVec2(buttonw, 80));
            //ImGui::Button("Show Trajectory", ImVec2(buttonw, 80));

            ImGui::PopStyleVar();

            ImGui::End();
        } else{
            mNeedRefreshCameraList = true;
        }

        if (mShowStats)
        {
            ImGui::Begin("Statistics", nullptr);
            ImGui::SetWindowPos(ImVec2(mWidth - 650.0f, 20.0f), ImGuiCond_Once);

            ImGui::Checkbox("Texture Update", &mTextureUpdate);

            ImGui::Text("Device Pos %.2f, %.2f, %.2f", mLastPos.x, mLastPos.y, mLastPos.z);

            ImGui::Text("Receiving timestamp - %.2lf", mCurSyncTimestamp);
            ImGui::Text("Receiving %d b/sec, Sending %d b/sec", mBytesRecv.getAvgValue(),
                        mBytesSend.getAvgValue());
            ImGui::Text("Poses rate - %d packets/sec", mPacketPerSecond);
            ImGui::Text("Images rate - %d images/sec", mImagePerSecond);

            if (mTrackRender.getLength() > 0) {
                float *ptr = mTrackRender.getPointer();
                int len = mTrackRender.getLength();

                ImGui::PlotLines("Render Delta (ms)", ptr, len, 0, NULL, 0.0f, 100.0f);
            }

            if (mTrackPoses.getLength() > 0) {
                float *ptr = mTrackPoses.getPointer();
                int len = mTrackPoses.getLength();

                ImGui::PlotLines("Poses Delta", ptr, len, 0, NULL, 0.0f, 1.0f);
            }

            if (mTrackRecv.getLength() > 0) {
                float *ptr = mTrackRecv.getPointer();
                int len = mTrackRecv.getLength();

                ImGui::PlotLines("Recv Delta", ptr, len, 0, NULL, 0.0f, 1.0f);
            }

            ImGui::Text("Application average %.3f ms/frame (%.1f FPS)",
                        1000.0f / ImGui::GetIO().Framerate, ImGui::GetIO().Framerate);

            if (mIsSynced)
            {
                ImGui::TextColored(ImVec4(0.1f, 0.8f, 0.1f, 1.0f), "Status: Synced with %d.%d.%d.%d:%d", mSyncAddress.GetD(),
                mSyncAddress.GetC(), mSyncAddress.GetB(), mSyncAddress.GetA(), mSyncAddress.GetPort());
            } else{
                ImGui::TextColored(ImVec4(0.8f, 0.1f, 0.1f, 1.0f), "Status: unsynced!");
            }


            ImGui::End();
        }

        if (mShowTransport)
        {
            ImVec2 size(1650.0f, 160.0f);
            ImGui::Begin("Transport Controls", nullptr, size, -1.0f, ImGuiWindowFlags_NoScrollbar | ImGuiWindowFlags_NoResize);
            ImGui::SetWindowPos(ImVec2(100.0f, mHeight - 180.0f), ImGuiCond_Once);

            const ImVec2 buttonSize(100.0f, 50.0f);

            {
                ImGui::BeginChild("Buttons", ImVec2(1800.0f, 65.0f));
                //ImGui::SetWindowFontScale(3.0f);

                ImGui::Columns(3, "FuncButtons", false);
                //ImGui::SetColumnWidth(2, 120);
                ImGui::SetColumnWidth(1, 700);

                /*
                if (mIsSynced)
                {
                    ImGui::PushStyleColor(ImGuiCol_Button, ImVec4(0.1f, 0.8f, 0.1f, 1.0f));
                    syncPress = ImGui::Button("Synced", buttonSize);
                    ImGui::PopStyleColor(1);
                } else
                {
                    ImGui::PushStyleColor(ImGuiCol_Button, ImVec4(0.8f, 0.1f, 0.1f, 1.0f));
                    syncPress = ImGui::Button("UnSynced", buttonSize);
                    ImGui::PopStyleColor(1);
                }
                */

                bool isLive = (mInternalState.liveMode > 0);
                ImGui::Checkbox("Live", &isLive);
                mInternalState.liveMode = (unsigned char) ((isLive) ? 1 : 0);

                // takes
                int take_ndx = (int) mInternalState.take;

                if ((int) mInternalState.takeCount != mTakeNames.size()) {

                    mTakeNames.resize((int) mInternalState.takeCount);

                    for (size_t ii = 0; ii < mTakeNames.size(); ++ii) {
                        memset(mTakeNames[ii].raw, 0, sizeof(char)*32);
                        sprintf(mTakeNames[ii].raw, " Take %03d", (int)(ii+1));
                    }

                }

                ImGui::SameLine();
                ImGui::Combo("##Takes", &take_ndx, &TakeGetName, &mTakeNames, (int) mTakeNames.size(), 25);
                mInternalState.take = (unsigned char) take_ndx;
                ImGui::SameLine();
                if (nullptr != mTextureDelete.get())
                {
                    takeDeletePress = ImGui::ImageButton((ImTextureID) (uintptr_t) mTextureDelete->GetTextureID(),
                                       ImVec2(0.5f * buttonSize.x, buttonSize.y), ImVec2(0.33f, 0.0f), ImVec2(0.66f, 1.0f));
                } else {
                    takeDeletePress = ImGui::Button("Delete", ImVec2(0.5f * buttonSize.x, buttonSize.y));
                }


                ImGui::NextColumn();

                // record
                if (mInternalState.recordMode > 0) {
                    if (nullptr != mTextureRecording.get())
                        recordPress = ImGui::ImageButton(
                                (ImTextureID) (uintptr_t) mTextureRecording->GetTextureID(),
                                buttonSize);
                    else
                        recordPress = ImGui::Button("Recording", buttonSize);
                } else {
                    if (nullptr != mTextureRecOn.get())
                        recordPress = ImGui::ImageButton(
                                (ImTextureID) (uintptr_t) mTextureRecOn->GetTextureID(),
                                buttonSize);
                    else
                        recordPress = ImGui::Button("Record", buttonSize);
                }

                // rewind
                ImGui::SameLine();
                if (nullptr != mTextureRewind.get())
                    rewindPress = ImGui::ImageButton(
                            (ImTextureID) (uintptr_t) mTextureRewind->GetTextureID(), buttonSize);
                else
                    rewindPress = ImGui::Button("Rewind", buttonSize);


                // stop
                ImGui::SameLine();
                if (nullptr != mTextureStop.get())
                    stopPress = ImGui::ImageButton(
                            (ImTextureID) (uintptr_t) mTextureStop->GetTextureID(), buttonSize);
                else
                    stopPress = ImGui::Button("Stop", buttonSize);

                // play
                ImGui::SameLine();
                if (mInternalState.playMode > 0) {
                    if (nullptr != mTexturePause.get()) {
                        playPress = ImGui::ImageButton(
                                (ImTextureID) (uintptr_t) mTexturePause->GetTextureID(),
                                buttonSize);
                    } else {
                        playPress = ImGui::Button("Pause", buttonSize);
                    }

                } else {
                    if (nullptr != mTexturePlay.get()) {
                        playPress = ImGui::ImageButton(
                                (ImTextureID) (uintptr_t) mTexturePlay->GetTextureID(), buttonSize);
                    } else {
                        playPress = ImGui::Button("Play", buttonSize);
                    }

                }

                // loop
                ImGui::SameLine();
                if (mInternalState.loopMode > 0) {
                    if (nullptr != mTextureLoopOff.get())
                        loopPress = ImGui::ImageButton(
                                (ImTextureID) (uintptr_t) mTextureLoopOn->GetTextureID(),
                                buttonSize);
                    else
                        loopPress = ImGui::Button("Loop On", buttonSize);
                } else {
                    if (nullptr != mTextureLoopOff.get())
                        loopPress = ImGui::ImageButton(
                                (ImTextureID) (uintptr_t) mTextureLoopOff->GetTextureID(),
                                buttonSize);
                    else
                        loopPress = ImGui::Button("Loop Off", buttonSize);
                }


                ImGui::NextColumn();


                int speed_type = (int) mInternalState.speed;
                ImGui::Combo("##Speed", &speed_type, TRASNPORT_PLAY_SPEED);
                mInternalState.speed = (unsigned char) speed_type;

                ImGui::EndChild();
            }

            {
                ImGui::BeginChild("Slider", ImVec2(1700.0f, 45.0f));
                //ImGui::SetWindowFontScale(3.0f);

                ImGui::Columns(3, "timelineId", false);
                ImGui::Separator();

                ImGui::SetColumnWidth(0, 400);
                ImGui::SetColumnWidth(1, 900);
                ImGui::SetColumnWidth(2, 400);

                ImGui::Text("Start:");

                ImGui::SameLine();
                //ImGui::PushItemWidth(60);
                ImGui::DragInt("##start", &mInternalState.startFrame);
                //ImGui::PopItemWidth();

                ImGui::NextColumn();

                //ImGui::SameLine();
                ImGui::PushItemWidth(-1);
                ImGui::SliderInt("##slider", &mInternalState.currFrame, mInternalState.startFrame,
                                 mInternalState.stopFrame);
                ImGui::PopItemWidth();

                ImGui::NextColumn();

                //ImGui::SameLine();
                ImGui::Text("Stop:");

                ImGui::SameLine();
                //ImGui::PushItemWidth(60);
                ImGui::DragInt("##stop", &mInternalState.stopFrame);
                //ImGui::PopItemWidth();

                //ImGui::SameLine();

                ImGui::EndChild();
            }


            ImGui::Columns(1);
            ImGui::Separator();

            ImGui::End();
        }

        if (mShowTrigger)
        {
            ImGui::Begin("Trigger Controls");

            ImVec2 buttonSize(120.0f, 120.0f);

            ImGui::Button("##trigger1", buttonSize);
            triggers[0] = ImGui::IsItemClicked(0);

            ImGui::SameLine();
            ImGui::Button("##trigger2", buttonSize);
            triggers[1] = ImGui::IsItemClicked(0);

            ImGui::Button("##trigger3", buttonSize);
            triggers[2] = ImGui::IsItemClicked(0);

            ImGui::SameLine();
            ImGui::Button("##trigger4", buttonSize);
            triggers[3] = ImGui::IsItemClicked(0);

            ImGui::Button("##trigger5", buttonSize);
            triggers[4] = ImGui::IsItemClicked(0);

            ImGui::SameLine();
            ImGui::Button("##trigger6", buttonSize);
            triggers[5] = ImGui::IsItemClicked(0);

            ImGui::End();
        }

        ImGui::Render();


        // process user actions and sync with mobu
        if (playPress && playPress != mLastPlayPress) {
            SendCommand((float) timestamp, PACKET_COMMAND_PLAY, (int) mInternalState.playMode);
        } else if (stopPress && stopPress != mLastStopPress) {
            SendCommand((float) timestamp, PACKET_COMMAND_STOP, (int) mInternalState.playMode);
        } else if (rewindPress && rewindPress != mLastRewindPress) {
            SendCommand((float) timestamp, PACKET_COMMAND_REWIND, 1);
        } else if (loopPress && loopPress != mLastLoopPress) {
            SendCommand((float) timestamp, PACKET_COMMAND_LOOP, (int) mInternalState.loopMode);
        } else if (recordPress && recordPress != mLastRecordPress) {
            SendCommand((float) timestamp, PACKET_COMMAND_RECORD, (int) mInternalState.recordMode);
        } else if (mSyncState.currFrame != mInternalState.currFrame) {
            SendCommand((float) timestamp, PACKET_COMMAND_FRAME_CURR, mInternalState.currFrame);
        }
        else if (mSyncState.startFrame != mInternalState.startFrame)
        {
            SendCommand((float) timestamp, PACKET_COMMAND_FRAME_START, mInternalState.startFrame);
        }
        else if (mSyncState.stopFrame != mInternalState.stopFrame)
        {
            SendCommand((float) timestamp, PACKET_COMMAND_FRAME_STOP, mInternalState.stopFrame);
        }
        else if (mSyncState.speed != mInternalState.speed)
        {
            SendCommand((float) timestamp, PACKET_COMMAND_SPEED_NDX, (int)mInternalState.speed);
        }
        else if (mSyncState.take != mInternalState.take)
        {
            SendCommand((float) timestamp, PACKET_COMMAND_TAKE_NDX, (int)mInternalState.take);
        }
        else if (mSyncState.liveMode != mInternalState.liveMode)
        {
            SendCommand((float) timestamp, PACKET_COMMAND_LIVE, (int)mInternalState.liveMode);
        }
        else if (takeDeletePress && takeDeletePress != mLastTakeDeletePress)
        {
            SendCommand((float) timestamp, PACKET_COMMAND_TAKE_REM, (int) mInternalState.take);
        }
        else if (newCameraPress && newCameraPress != mLastNewCameraPress)
        {
            SendCommand((float) timestamp, PACKET_COMMAND_CAMERA_NEW, 1);
        }

        //

        mLastTakeDeletePress = takeDeletePress;
        mLastSyncPress = syncPress;
        mLastRecordPress = recordPress;
        mLastRewindPress = rewindPress;
        mLastStopPress = stopPress;
        mLastPlayPress = playPress;
        mLastLoopPress = loopPress;
        mLastNewCameraPress = newCameraPress;

        //
        //

        for (int i=0; i<6; ++i)
            mCameraTrigger[i] = (triggers[i]) ? 1.0f : 0.0f;
    }

    bool MotionTrackingApp::SendCommand(float timestamp, int commandId, int commandValue)
    {
        bool lSuccess = false;

        //if (false == mSocketSender.IsOpen())
        //    return lSuccess;

        if (false == mIsSynced)
            return lSuccess;

        Network::CPacketCommand cmd;

        cmd.header.magic[0] = PACKET_MAGIC_1;
        cmd.header.magic[1] = PACKET_MAGIC_2;
        cmd.header.magic[2] = PACKET_MAGIC_3;

        cmd.header.timestamp = timestamp;
        cmd.header.reason = PACKET_REASON_COMMAND;

        cmd.body.command = (unsigned char) commandId;
        cmd.body.value = commandValue;

        Network::Address address(mSyncAddress.GetD(), mSyncAddress.GetC(), mSyncAddress.GetB(), mSyncAddress.GetA(), NETWORK_MOBU_TRACKER);
        lSuccess = mSocketSender.Send(address, &cmd, sizeof(Network::CPacketCommand));

        if (true == lSuccess)
        {
            mBytesSend.addvalue( sizeof(Network::CPacketCommand), timestamp );
            LOGI("Command %d has been sent", (int) cmd.body.command);
        }

        return lSuccess;
    }

    bool MotionTrackingApp::SendFeedback(float timestamp)
    {
        bool lSuccess = false;

        //if (false == mSocketSender.IsOpen())
         //   return lSuccess;

        if (false == mIsSynced)
            return lSuccess;

        Network::CHeader packet;

        packet.magic[0] = PACKET_MAGIC_1;
        packet.magic[1] = PACKET_MAGIC_2;
        packet.magic[2] = PACKET_MAGIC_3;

        packet.timestamp = timestamp;
        packet.reason = PACKET_REASON_FEEDBACK;

        //mSyncAddress.Set(192, 168, 1, 97, 8887);
        //lSuccess = mSocketSender.Send(mSyncAddress, &packet, sizeof(Network::CHeader));

        if (true == lSuccess)
        {
            mBytesSend.addvalue( sizeof(Network::CHeader), timestamp );
        }

        return lSuccess;
    }

    bool MotionTrackingApp::SendImageFeedback(double timestamp)
    {
        bool lSuccess = false;

        //if (false == mSocketSender.IsOpen())
        //    return lSuccess;

        if (false == mIsSynced)
            return lSuccess;

        if (0.0 == mLastImageFeedbackTimestamp)
            mLastImageFeedbackTimestamp = timestamp;

        if (timestamp - mLastImageFeedbackTimestamp < 1.0)
            return false;

        Network::CHeader packet;

        packet.magic[0] = PACKET_MAGIC_1;
        packet.magic[1] = PACKET_MAGIC_2;
        packet.magic[2] = PACKET_MAGIC_3;

        packet.timestamp = timestamp;
        packet.reason = PACKET_REASON_FEEDBACK;

        //Network::Address address(mSyncAddress);
       // address.SetPortOnly(NETWORK_IMAGE_SERVER_PORT);
        //lSuccess = mSocketSender.Send(address, &packet, sizeof(Network::CHeader));

        if (true == lSuccess)
        {
            mBytesSend.addvalue( sizeof(Network::CHeader), timestamp );
        }

        mLastImageFeedbackTimestamp = timestamp;
        return lSuccess;
    }

}  // namespace tango_motion_tracking
