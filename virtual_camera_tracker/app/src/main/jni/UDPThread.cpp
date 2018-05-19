/*
 * UDPThread.cpp
 *
 *  Created on: 2015. 3. 3.
 *      Author: youngmok
 */

#include "UDPThread.h"
#include "NetworkTango.h"

#include <stdio.h>
#include <sys/socket.h>
#include <sys/un.h>
#include <stdio.h>
#include <unistd.h>

#include "strings.h"
#include "string.h"

#include <tango-gl/util.h>

#include <sched.h>

namespace Network
{

	CUDPThread::CUDPThread(int entryFuncid, int _port, int _bufferLen)
        : mEntryFuncId(entryFuncid)
        , mSocketPort(_port)
    {
		mBufferLen = _bufferLen;
		mBuffer = new unsigned char[mBufferLen];

		// pthread initialization
		_thread = pthread_self(); // get pthread ID
		//pthread_setschedprio(_thread, SCHED_FIFO); // setting priority

        pthread_rwlock_init(&_rwlock, NULL);

		// log file initialization
		//if ( UDP_FILE_LOG_FLAG == 1)	ofp = fopen("sensordata.txt", "w");

	}

	CUDPThread::~CUDPThread()
	{
		pthread_join(_thread, NULL);    // close the thread

        pthread_rwlock_destroy(&_rwlock);

        mSocket.Close();
		//fclose(ofp);					// close log file

		if (mBuffer) {
			delete[] mBuffer;
			mBuffer = nullptr;
		}
	}

	double CUDPThread::getTimeElapsed(struct timeval end, struct timeval start) {
		return (end.tv_sec - start.tv_sec) + (end.tv_usec - start.tv_usec) / 1000000.00;
	}

/** Returns true if the thread was successfully started, false if there was an error starting the thread */
	bool CUDPThread::StartInternalThread() {
		return (pthread_create(&_thread, NULL, InternalThreadEntryFunc, this) == 0);
	}

/** Will not return until the internal thread has exited. */
	void CUDPThread::WaitForInternalThreadToExit() {
		(void) pthread_join(_thread, NULL);
	}

	bool CUDPThread::CheckMagicNumber(const unsigned char *buffer) {
		return (PACKET_MAGIC_1 == buffer[0] && PACKET_MAGIC_2 == buffer[1]
				&& PACKET_MAGIC_3 == buffer[2]);
	}

    void CUDPThread::EmptyBuffer()
    {
        int bytes_received = 1;
        Address sender;

        while (bytes_received > 0)
        {

            bytes_received = mSocket.Receive(sender, mBuffer, mBufferLen);

            if (bytes_received >= sizeof(CHeader) && bytes_received < mBufferLen) {
                if (true == CheckMagicNumber(mBuffer)) {

                    CHeader *pheader = (CHeader *) mBuffer;

                    if (mLastTimeStamp < pheader->timestamp)
                    {
                        mLastTimeStamp = pheader->timestamp;
                        mAddressRecv = sender;
                    }
                }
            }
        }
    }
/*
	void CUDPThread::InternalThreadEntry()
    {
		gettimeofday(&tvalInit, NULL);

        // clear a UDP packets queue before we start
        mSocket.Open(mSocketPort, false);

        if (false == mSocket.IsOpen())
        {
            return;
        }

        Address sender;
        int bytes_received = 1;

        //
        EmptyBuffer();

        LOGI("[UDP Thread] Ready for running");

		for (;;)
        {
            LOGI("[UDP Thread] OnPrepare");
            //OnPrepare(1);


            LOGI("[UDP Thread] Running");

            while(bytes_received)
            {
                bytes_received = mSocket.Receive(sender, mBuffer, mBufferLen);

                if (bytes_received >= sizeof(CPacketHeader) && bytes_received < mBufferLen)
                {
                    //LOGI("[UDP Thread] received a packet!");

                    if (true == CheckMagicNumber(mBuffer) )
                    {
                        CPacketHeader *pheader = (CPacketHeader*) mBuffer;

                        unsigned char *ptr = mBuffer + sizeof(CPacketHeader);
                        const int len = bytes_received - sizeof(CPacketHeader);

                        OnPacketReceived(sender, pheader, len, ptr);
                    }
                }
            }

            // time to upload a result
            LOGI("[UDP Thread] OnExchange");
            //OnExchange();

			if (UDP_FILE_LOG_FLAG == 1) {
				gettimeofday(&tvalNow, NULL);
				double time_elapsed = getTimeElapsed(tvalNow, tvalInit);
				//fprintf(ofp, "%lf\t%s\n", time_elapsed,mesg);
			}
		}
	};
*/

	void *CUDPThread::InternalThreadEntryFunc(void *This) {


        switch(((CUDPThread *) This)->GetEntryFuncId() )
        {
            case 1:
                ((CUDPThreadSync*)This)->InternalThreadEntry();
                break;
            case 2:
                ((CUDPThreadImage*)This)->InternalThreadEntry();
                break;
        }
		//((CUDPThread *) This)->InternalThreadEntry();
		return NULL;
	}


    /////////////////////////////////////////////////////////////////////////
    //

    CUDPThreadSync::CUDPThreadSync()
        : CUDPThread(1, NETWORK_SYNC_PORT, MAX_UDP_BUFFER_SIZE)
    {
        mLastTimeStamp = 0.0f;

        //
        mCameraInfoCounter = 0;
        mLastCameraInfoTimestamp = 0.0;

        mReceivedData.timestamp = 0.0;
        mReceivedData.numStates = 0;
        mReceivedData.camInfoTimeStamp = 0.0;
    }

    void CUDPThreadSync::InternalThreadEntry()
    {
        gettimeofday(&tvalInit, NULL);

        // clear a UDP packets queue before we start
        mSocket.Open(mSocketPort, false);

        if (false == mSocket.IsOpen())
        {
            return;
        }

        Address sender;
        int bytes_received = 1;

        //
        EmptyBuffer();

        static int enterId = 0;

        LOGI("[UDP Thread Sync] Ready for running");

        enterId += 1;

        LOGI("[UDP Thread Sync] OnPrepare - %d", enterId);
        mReceivedData.numStates = 0;
        mReceivedData.numRegPackets = 0;
        mReceivedData.numCameraInfo = 0;

        for (;;)
        {

            bytes_received = mSocket.Receive(sender, mBuffer, mBufferLen);

            if (bytes_received >= sizeof(CHeader) && bytes_received < mBufferLen)
            {
                //LOGI("[UDP Thread] received a packet!");

                if (true == CheckMagicNumber(mBuffer) )
                {
                    //LOGE("[UDP Thread Sync] Packet received");


                    CHeader *pheader = (CHeader*) mBuffer;

                    unsigned char *ptr = mBuffer + sizeof(CHeader);
                    const int len = bytes_received - sizeof(CHeader);

                    OnPacketReceived(sender, pheader, len, ptr);
                }
            } else {

                // time to upload a result
                //enterId += 1;
                //LOGI("[UDP Thread Sync] OnExchange - %d", enterId);
                if (mReceivedData.numStates > 0 || mReceivedData.numCameraInfo > 0
                    || mReceivedData.numRegPackets > 0) {


                    // DONE: map writting access
                    if (0 == pthread_rwlock_trywrlock(&_rwlock)) {
                        mSyncState = mReceivedData.lastState;
                        mLastTimeStamp = mReceivedData.timestamp;
                        mAddressRecv = mReceivedData.sender;

                        // DONE:
                        if (mReceivedData.numCameraInfo > 0) {
                            mLastCameraInfoTimestamp = mReceivedData.camInfoTimeStamp;
                            mCameraInfoCounter = mReceivedData.camInfoCounter;

                            mCameraInfo.resize(mReceivedData.cameraInfo.size());

                            for (size_t i=0; i<mCameraInfo.size(); ++i)
                            {
                                mCameraInfo[i] = mReceivedData.cameraInfo[i];
                            }
                        }

                        // DONE: unmap writting access
                        pthread_rwlock_unlock(&_rwlock);
                    }
                }


                //LOGI("[UDP Thread Sync] OnPrepare - %d", enterId);
                mReceivedData.numStates = 0;
                mReceivedData.numRegPackets = 0;
                mReceivedData.numCameraInfo = 0;

                sched_yield();
            }

        }
    }



    bool CUDPThreadSync::OnPacketReceived(Address &sender, CHeader *pHeader,
                                          unsigned int bodyLen, unsigned char *bodyData)
    {
        bool lSuccess = false;

        if (PACKET_REASON_CONTROL == pHeader->reason)
        {
            // is packet recent ?
            if (mReceivedData.timestamp < pHeader->timestamp && bodyLen == sizeof(CSyncControl))
            {
                memcpy(&mReceivedData.lastState, bodyData, sizeof(CSyncControl));

                mReceivedData.timestamp = pHeader->timestamp;
                mReceivedData.sender = sender;

                mReceivedData.numStates += 1;
            }

            lSuccess = true;
        }
        else if (PACKET_REASON_REGISTER == pHeader->reason)
        {

            if (mReceivedData.timestamp < pHeader->timestamp)
            {
                mReceivedData.timestamp = pHeader->timestamp;
                mReceivedData.sender = sender;

                mReceivedData.numRegPackets += 1;
            }

            lSuccess = true;
        }
        else if (PACKET_REASON_CAMERA_INFO)
        {
            CCameraInfo *packetData = (CCameraInfo*) bodyData;

            if (mReceivedData.camInfoTimeStamp <= (double)pHeader->timestamp)
            {
                if (mReceivedData.camInfoTimeStamp < pHeader->timestamp)
                    mReceivedData.camInfoCounter = 0;

                int totalCount = packetData->totalCount;
                mReceivedData.cameraInfo.resize(totalCount);
                mReceivedData.cameraInfo[packetData->id] = *packetData;
                mReceivedData.camInfoCounter += 1;
                mReceivedData.camInfoTimeStamp = (double)pHeader->timestamp;

                mReceivedData.numCameraInfo += 1;
            }

            lSuccess = true;
        }

        return lSuccess;
    }

    bool CUDPThreadSync::GetSyncData(const float localtimestamp, float &synctimestamp, Address &lastAddress, CSyncControl &sync)
    {
        bool lSuccess = false;

        // DONE: map access
        if (localtimestamp < mLastTimeStamp)
        {
            if (0 == pthread_rwlock_tryrdlock(&_rwlock) )
            {
                memcpy(&sync, &mSyncState, sizeof(CSyncControl));
                synctimestamp = mLastTimeStamp;
                lastAddress = mAddressRecv;

                lSuccess = true;

                // DONE: map access
                pthread_rwlock_unlock(&_rwlock);
            }
        }

        return lSuccess;
    }

    bool CUDPThreadSync::GetCameraData(double &synctimestamp, int &syncloadedcounter, std::vector<CCameraInfo> &cameraVector)
    {
        bool lSuccess = false;

        // DONE: map access
        if (synctimestamp != mLastCameraInfoTimestamp || syncloadedcounter != mCameraInfoCounter)
        {
            if (0 == pthread_rwlock_tryrdlock(&_rwlock) )
            {
                cameraVector.resize( mCameraInfo.size() );

                memcpy(cameraVector.data(), mCameraInfo.data(), sizeof(CCameraInfo)*mCameraInfo.size());

                synctimestamp = mLastCameraInfoTimestamp;
                syncloadedcounter = mCameraInfoCounter;

                lSuccess = true;

                // DONE: map access
                pthread_rwlock_unlock(&_rwlock);
            }
        }

        return lSuccess;
    }

    bool CUDPThreadSync::GetLastSender(float &lasttimestamp, Network::Address &lastAddress)
    {
        bool lSuccess = false;

        // DONE: map access
        if (0 == pthread_rwlock_tryrdlock(&_rwlock) )
        {
            lasttimestamp = mLastTimeStamp;
            lastAddress = mAddressRecv;

            lSuccess = true;

            // DONE: map access
            pthread_rwlock_unlock(&_rwlock);
        }

        return lSuccess;
    }

    ///////////////////////////////////////////////////////////////////////////
    //

    CUDPThreadImage::CUDPThreadImage()
        : CUDPThread(2, NETWORK_IMAGE_PORT, MAX_UDP_BUFFER_SIZE)
    {
        AllocImageData(recvBuffer[0]);
        AllocImageData(recvBuffer[1]);
        AllocImageData(mLastReceivedData);

        mLastTimeStamp = 0.0f;
        mLastReceivedData.lastTimestamp = 0.0f;
    }

    CUDPThreadImage::~CUDPThreadImage()
    {
        FreeImageData(recvBuffer[0]);
        FreeImageData(recvBuffer[1]);
        FreeImageData(mLastReceivedData);
    }

    void CUDPThreadImage::AllocImageData(ImageData &data)
    {
        data.data = new unsigned char[MAX_IMAGE_SIZE+16];
    }

    void CUDPThreadImage::FreeImageData(ImageData &data)
    {
        if (nullptr != data.data)
        {
            delete [] data.data;
            data.data = nullptr;
        }
    }

    void CUDPThreadImage::CopyImageData(ImageData &dst, const ImageData &src)
    {
        dst.lastTimestamp = src.lastTimestamp;
        dst.numberOfReceivedTiles = src.numberOfReceivedTiles;
        memcpy(dst.data, src.data, sizeof(unsigned char)*src.header.dataSize);
        dst.header = src.header;
    }

    void CUDPThreadImage::ClearImageData(ImageData &data)
    {
        memset(data.data, 0, sizeof(unsigned char)*(MAX_IMAGE_SIZE+16));
        data.numberOfReceivedTiles = 0;

        for (int i=0; i<MAX_NUMBER_OF_TILES; ++i)
            data.receivedTiles[i] = false;
    }


    void CUDPThreadImage::InternalThreadEntry()
    {
        gettimeofday(&tvalInit, NULL);

        // clear a UDP packets queue before we start
        mSocket.Open(mSocketPort, false);

        if (false == mSocket.IsOpen())
        {
            return;
        }

        Address sender;
        int bytes_received = 1;

        //
        EmptyBuffer();

        static int enterId = 0;

        LOGI("[UDP Thread Image] Ready for running");

        enterId += 1;

        LOGI("[UDP Thread Image] OnPrepare - %d", enterId);
        currIndex = 0;

        recvBuffer[0].totalTiles = 1;
        recvBuffer[0].numberOfReceivedTiles = 0;
        recvBuffer[0].lastTimestamp = 0.0f;

        recvBuffer[1].totalTiles = 1;
        recvBuffer[1].numberOfReceivedTiles = 0;
        recvBuffer[1].lastTimestamp = 0.0f;

        //ClearImageData(recvBuffer[0]);
        //ClearImageData(recvBuffer[1]);

        for (;;)
        {
            bytes_received = mSocket.Receive(sender, mBuffer, mBufferLen);

            if (bytes_received > 0)
            {
                if (bytes_received >= sizeof(CHeader) && bytes_received < mBufferLen) {
                    //LOGI("[UDP Thread Image] received a packet!");

                    if (true == CheckMagicNumber(mBuffer)) {
                        //LOGE("[UDP Thread Image] Packet received");

                        CHeader *pheader = (CHeader *) mBuffer;

                        unsigned char *ptr = mBuffer + sizeof(CHeader);
                        const int len = bytes_received - sizeof(CHeader);

                        OnPacketReceived(sender, pheader, len, ptr);
                    }
                }
            }
            // time to upload a result
            else if ( recvBuffer[0].numberOfReceivedTiles == recvBuffer[0].totalTiles
                 || recvBuffer[1].numberOfReceivedTiles == recvBuffer[1].totalTiles)
            {
                //LOGE("[UDP Thread Image] image received, %.2f and %.2f !!!", recvBuffer[0].lastTimestamp, recvBuffer[1].lastTimestamp);

                if (0 == pthread_rwlock_trywrlock(&_rwlock) ) {

                    if (recvBuffer[0].numberOfReceivedTiles == recvBuffer[0].totalTiles) {

                        CopyImageData(mLastReceivedData, recvBuffer[0]);

                        recvBuffer[0].totalTiles = 1;
                        recvBuffer[0].numberOfReceivedTiles = 0;
                        recvBuffer[0].lastTimestamp = 0.0f;

                    }
                    else if (recvBuffer[1].numberOfReceivedTiles == recvBuffer[1].totalTiles) {

                        CopyImageData(mLastReceivedData, recvBuffer[1]);

                        recvBuffer[1].totalTiles = 1;
                        recvBuffer[1].numberOfReceivedTiles = 0;
                        recvBuffer[1].lastTimestamp = 0.0f;
                    }

                    pthread_rwlock_unlock(&_rwlock);

                    //usleep(300);
                    sched_yield();
                }

            }
        }
    }


    bool CUDPThreadImage::OnPacketReceived(Address &sender, CHeader *pHeader,
                                           unsigned int bodyLen, unsigned char *bodyData)
    {
        bool lSuccess = false;

        if (PACKET_REASON_IMAGE == pHeader->reason && bodyLen == sizeof(CImageHeader))
        {
            // is packet recent ?
            if (mLastTimeStamp < pHeader->timestamp)
            {
                CImageHeader *imageHeader = (CImageHeader*) bodyData;

                currIndex = (currIndex > 0) ? 0 : 1;

                recvBuffer[currIndex].lastTimestamp = pHeader->timestamp;
                recvBuffer[currIndex].header = *imageHeader;
                recvBuffer[currIndex].numberOfReceivedTiles = 0;
                // TODO: recvBuffer[currIndex].totalTiles = imageHeader->numTiles;

                mLastTimeStamp = pHeader->timestamp;
                mAddressRecv = sender;
            }
            else if (mLastTimeStamp == pHeader->timestamp)
            {
                // situation when tile we have received before the header
                CImageHeader *imageHeader = (CImageHeader*) bodyData;

                recvBuffer[currIndex].header = *imageHeader;

                mLastTimeStamp = pHeader->timestamp;
                mAddressRecv = sender;
/* TODO:
                if (mLastTimeStamp > 0.0f
                    && recvBuffer[currIndex].header.numTiles == recvBuffer[currIndex].numberOfReceivedTiles)
                {
                    recvBuffer[currIndex].lastTimestamp = mLastTimeStamp;
                    currIndex = (currIndex > 0) ? 0 : 1;
                }
                */
            }

            lSuccess = true;
        }
        else if (PACKET_REASON_IMAGE_TILE == pHeader->reason)
        {
            CImageTileHeader    *pImageTile = (CImageTileHeader*) bodyData;

            //LOGI("> tile %d", pImageTile->tileIndex);

            if (recvBuffer[0].lastTimestamp == pHeader->timestamp ) {
                // DONE: receiving a tile for a previously received header

                if (recvBuffer[0].totalTiles == pImageTile->tileCount)
                {

                    const int ndx = (int) pImageTile->tileIndex;

                    {
                        recvBuffer[0].receivedTiles[ndx] = true;
                        recvBuffer[0].numberOfReceivedTiles += 1;
                        recvBuffer[0].totalTiles = pImageTile->tileCount;

                        unsigned char *dstptr = recvBuffer[0].data + pImageTile->tileOffset;
                        unsigned char *srcptr = bodyData + sizeof(CImageTileHeader);

                        // tileSize is the same except last tile, it could be in [1; tileSize]
                        memcpy(dstptr, srcptr, sizeof(unsigned char) * pImageTile->tileSize);
                    }
                }
            }
            else if (recvBuffer[1].lastTimestamp == pHeader->timestamp )
            {
                // DONE: receiving a tile for a previously received header

                if (recvBuffer[1].totalTiles == pImageTile->tileCount)
                {

                    const int ndx = (int) pImageTile->tileIndex;

                    {
                        recvBuffer[1].receivedTiles[ndx] = true;
                        recvBuffer[1].numberOfReceivedTiles += 1;
                        recvBuffer[1].totalTiles = pImageTile->tileCount;

                        unsigned char *dstptr = recvBuffer[1].data + pImageTile->tileOffset;
                        unsigned char *srcptr = bodyData + sizeof(CImageTileHeader);

                        // tileSize is the same except last tile, it could be in [1; tileSize]
                        memcpy(dstptr, srcptr, sizeof(unsigned char) * pImageTile->tileSize);
                    }
                }
            }
            else if ( mLastTimeStamp < pHeader->timestamp)
            {
                currIndex = (currIndex > 0) ? 0 : 1;

                const int ndx = (int) pImageTile->tileIndex;

                recvBuffer[currIndex].receivedTiles[ndx] = true;
                recvBuffer[currIndex].numberOfReceivedTiles = 1;
                recvBuffer[currIndex].totalTiles = pImageTile->tileCount;
                recvBuffer[currIndex].lastTimestamp = pHeader->timestamp;

                unsigned char *dstptr = recvBuffer[currIndex].data + pImageTile->tileOffset;
                unsigned char *srcptr = bodyData + sizeof(CImageTileHeader);

                // tileSize is the same except last tile, it could be in [1; tileSize]
                memcpy(dstptr, srcptr, sizeof(char)*pImageTile->tileSize);

                mLastTimeStamp = pHeader->timestamp;
                mAddressRecv = sender;
            }

            lSuccess = true;
        }

        return lSuccess;
    }


    bool CUDPThreadImage::GetLastImageData(float localtimestamp, float &lastreceivedtimestamp, Address &lastAddress, CImageHeader &header, char *buffer, int bufferLen)
    {
        bool lSuccess = false;

        // DONE: map access
        if (0 == pthread_rwlock_tryrdlock(&_rwlock) )
        {
            //LOGI("check time stamp - %.2f", mLastReceivedData.lastTimestamp);

            if (localtimestamp < mLastReceivedData.lastTimestamp)
            {
                //LOGI("get last image success");

                if (bufferLen >= MAX_IMAGE_SIZE)
                {
                    memcpy(buffer, mLastReceivedData.data, sizeof(char)*mLastReceivedData.header.dataSize);
                    header = mLastReceivedData.header;
                    lastreceivedtimestamp = mLastReceivedData.lastTimestamp;
                    lastAddress = mAddressRecv;

                    //LOGE("[Thread Image] copy last image data!");
                    lSuccess = true;
                }
            }

            // DONE: map access
            pthread_rwlock_unlock(&_rwlock);
        }

        return lSuccess;
    }

}