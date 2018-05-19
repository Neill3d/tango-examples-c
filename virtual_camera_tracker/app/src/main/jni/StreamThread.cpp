/*
 * StreamThread.cpp
 *
 * Created on: 2018. 3. 3
 *  Sergei Solokhin (Neill3d)
 *
 * based on
 *  Created on: 2015. 3. 3.
 *      Author: youngmok
 */

#include "StreamThread.h"
#include "NetworkTango.h"

#include <stdio.h>

#ifdef WIN32
#include <winsock.h>

#define LOGE	printf
#define LOGI	printf
#else
#include <sys/socket.h>
#include <netinet/tcp.h>
#include <sys/un.h>
#include <unistd.h>
#include "strings.h"
#include <tango-gl/util.h>
#include <errno.h>
#endif

#include "string.h"

#include "zlib.h"

#include <chrono>
#include <thread>
#include <atomic>

#define NETWROK_SERVER_PORT	8889
#define NETWORK_MOBU_PORT	8886
#define NETWORK_RATE		60		// 60 frames per second


typedef std::chrono::high_resolution_clock high_resolution_clock;
typedef std::chrono::milliseconds milliseconds;

namespace Network
{

    std::atomic<uint32_t>       g_syncAddress;

    // sync
    double          g_lastSyncTimestamp;
    CSyncControl    g_lastSync;

    // image
    double          g_lastImageTimestamp;
    CImageHeader       g_lastImageHeader;
    CImageTileHeader    g_lastImageTileHeader;
    std::vector<unsigned char>   g_lastImageData;   // buffer for storing tiles into

    // cameras
    double              g_lastCamerasTimestamp;
    std::vector<CCameraInfo>    g_lastCameras;

    double GetLastSyncCamerasTimeStamp()
    {
        return g_lastCamerasTimestamp;
    }
    const std::vector<CCameraInfo> &GetLastSyncCamerasVector()
    {
        return g_lastCameras;
    }

    uint32_t GetLastSyncAddress()
    {
        uint32_t address = g_syncAddress;
        return address;
    }
    void SetLastSyncAddress(uint32_t address)
    {
        g_syncAddress = address;
    }

    double GetLastSyncTimestamp()
    {
        return g_lastSyncTimestamp;
    }

    void GetLastSyncState(CSyncControl &state)
    {
        state = g_lastSync;
    }

    double GetLastImageTimestamp()
    {
        return g_lastImageTimestamp;
    }

    void GetLastImageHeader(CImageHeader &header)
    {
        header = g_lastImageHeader;
    }
    size_t GetLastImageSize()
    {
        return g_lastImageData.size();
    }
    void GetLastImageData(unsigned char *buffer)
    {
        memcpy(buffer, g_lastImageData.data(), sizeof(unsigned char)*g_lastImageData.size());
    }

    /////////////////////////////
    // Exchange
/*
    struct PoseHolder
    {
        double timestamp;
        Network::CDeviceData    pose;

        PoseHolder()
        {
            timestamp = 0.0;
        }
    };

    PoseHolder g_poseHolders[3];
    std::atomic<uint32_t>   g_poseIndex(0);

    bool ExchangeWriteDeviceData(double timestamp, Network::CDeviceData &data)
    {
        uint32_t current = g_poseIndex;

        // triple buffer
        current = current + 1;
        if (current > 2)
            current = 0;

        g_poseHolders[current].timestamp = timestamp;
        g_poseHolders[current].pose = data;

        g_poseIndex = current;

        return true;
    }

    bool ExchangeReadDeviceData(double &lastreadStamp, Network::CDeviceData &data)
    {
        bool lSuccess = false;
        uint32_t current = g_poseIndex;

        if (g_poseHolders[current].timestamp > 0.0 && g_poseHolders[current].timestamp > lastreadStamp)
        {
            data = g_poseHolders[current].pose;
            lastreadStamp = g_poseHolders[current].timestamp;
            lSuccess = true;
        }

        return lSuccess;
    }
*/
	//struct timeval tvalNow, tvalInit;    // for time stamp

#ifdef WIN32
	void bzero(char *b, size_t length)
	{
		memset(b, 0, length);
	}
#endif
    int NewTCPSocket()
    {
        int tcpSocket = socket(PF_INET, SOCK_STREAM, 0);
        if (-1 == tcpSocket)
            LOGE("[CStreamThread] Failed to open a stream socket!\n");

        return tcpSocket;
    }

    void CloseTCPSocket(int socketHandle)
    {
#ifdef WIN32
		if (socketHandle >= 0)
			closesocket(socketHandle);
#else
        if (socketHandle >= 0)
            close(socketHandle);
#endif
    }

    bool BindSocketToPort(int sd, unsigned short port)
    {
        struct sockaddr_in address;
        memset(&address, 0, sizeof(address));
        address.sin_family = PF_INET;
        address.sin_addr.s_addr = htonl(INADDR_ANY);
        address.sin_port = htons(port);
        LOGI("[CStreamThread] Binding to %d\n", port);

        if (-1 == ::bind(sd, (struct sockaddr*) &address, sizeof(address))) {
            LOGE("[CStreamThread] failed to bind to port %d\n", errno);
            return false;
        }

        return true;
    }

    bool ListenOnSocket(int sd, int backlog)
    {
        if (-1 == listen(sd, backlog))
        {
            LOGE("[CStreamThread] failed to listen on socket %d\n", errno);
            return false;
        }
        return true;
    }

    int StartServer(unsigned short port)
    {
        int lsocket = 0;

        lsocket = NewTCPSocket();

        if (lsocket > 0)
        {
            if (false == BindSocketToPort(lsocket, port))
            {
                CloseTCPSocket(lsocket);
                return 0;
            }

            if (false == ListenOnSocket(lsocket, 1))
            {
                CloseTCPSocket(lsocket);
                return 0;
            }
        }
/*
        // disable Nagle algorithm
        int value = 1;
        setsockopt(lsocket, IPPROTO_TCP, TCP_NODELAY, &value, sizeof(int));
*/
        return lsocket;
    }

    void MoveLastBytes(unsigned char *ptr, int curpos, int bytesToCopy)
    {
        if (bytesToCopy > 0)
        {
            for (int j=0; j<bytesToCopy; ++j)
                ptr[j] = ptr[j+curpos];
        }
    }

    enum
    {
        PROCESS_STREAM_WAITING,
        PROCESS_STREAM_HEADER,
        PROCESS_STREAM_SYNC,
        PROCESS_STREAM_IMAGE,
        PROCESS_STREAM_TILE,
        PROCESS_STREAM_IMAGE_RAW
    };


    bool ProcessStream2(int iLength, int &bufferOffset, CHeader &lHeader, std::vector<unsigned char> &buffer)
    {

        int lStatus = PROCESS_STREAM_WAITING;

        int curpos = 0;
        unsigned char *readPtr = buffer.data();


        while (curpos < iLength)
        {
            unsigned char *curptr = readPtr + curpos;
            int bytesToCopy = iLength - curpos;

            switch(lStatus)
            {
                case PROCESS_STREAM_WAITING:

                    if (bytesToCopy >= 3)
                    {
                        if (CheckMagicNumber(curptr))
                        {
                            if (bytesToCopy >= sizeof(CHeader))
                            {
                                memcpy(&lHeader, curptr, sizeof(CHeader));

                                g_lastSyncTimestamp = (double) lHeader.timestamp;

                                curpos += sizeof(CHeader);
                                lStatus = PROCESS_STREAM_HEADER;

                            } else
                            {
                                MoveLastBytes(buffer.data(), curpos, bytesToCopy);
                                bufferOffset = bytesToCopy;
                                return false;
                            }
                        } else
                            curpos += 1;
                    }
                    else curpos += 1;

                    break;

                case PROCESS_STREAM_HEADER:
                    if (PACKET_REASON_REGISTER == lHeader.reason || PACKET_REASON_FEEDBACK == lHeader.reason)
                    {
                        LOGI("[CStreamThread] received an invitation %.2f\n", lHeader.timestamp);
                        curpos += 1;
                        lStatus = PROCESS_STREAM_WAITING;
                    }
                    else if (PACKET_REASON_CONTROL == lHeader.reason)
                    {
                        if (bytesToCopy >= sizeof(CSyncControl))
                        {
                            //LOGI("received a control %.2f\n", lHeader.timestamp);

                            memcpy(&g_lastSync, curptr, sizeof(CSyncControl) );

                            curpos += sizeof(CSyncControl);
                            lStatus = PROCESS_STREAM_WAITING;

                            g_lastSyncTimestamp = lHeader.timestamp;

                        } else if (bytesToCopy > 0)
                        {
                            MoveLastBytes(buffer.data(), curpos, bytesToCopy);
                            bufferOffset  =bytesToCopy;
                            return false;
                        }
                    }
                    else if (PACKET_REASON_IMAGE == lHeader.reason)
                    {
                        if (bytesToCopy >= sizeof(CImageHeader))
                        {
                            LOGI("[CStreamThread] received an image %.2f\n", lHeader.timestamp);

                            memcpy(&g_lastImageHeader, curptr, sizeof(CImageHeader));

                            // prepare space for tiles
                            g_lastImageData.resize( (size_t)g_lastImageHeader.dataSize );

                            curpos += sizeof(CImageHeader);
                            lStatus = PROCESS_STREAM_IMAGE_RAW;
                        }
                        else if (bytesToCopy > 0)
                        {
                            MoveLastBytes(buffer.data(), curpos, bytesToCopy);
                            bufferOffset  =bytesToCopy;
                            return false;
                        }
                    }
                    if (PACKET_REASON_IMAGE_TILE == lHeader.reason)
                    {
                        if (bytesToCopy >= sizeof(CImageTileHeader))
                        {
                            memcpy(&g_lastImageTileHeader, curptr, sizeof(CImageTileHeader));

                            curpos += sizeof(CImageTileHeader);
                            lStatus = PROCESS_STREAM_TILE;
                        }
                        else if (bytesToCopy > 0)
                        {
                            MoveLastBytes(buffer.data(), curpos, bytesToCopy);
                            bufferOffset = bytesToCopy;
                            return false;
                        }
                    }
                    else if (PACKET_REASON_CAMERA_INFO == lHeader.reason)
                    {
                        if (bytesToCopy >= sizeof(CCameraInfo))
                        {
                            CCameraInfo *pNewInfo = (CCameraInfo*) curptr;

                            g_lastCameras.resize(pNewInfo->totalCount);

                            unsigned char *dst = (unsigned char*) g_lastCameras.data() + pNewInfo->id * sizeof(CCameraInfo);
                            memcpy(dst, curptr, sizeof(CCameraInfo));

                            curpos += sizeof(CCameraInfo);
                            lStatus = PROCESS_STREAM_WAITING;

                            g_lastCamerasTimestamp = lHeader.timestamp;
                        }
                        else if (bytesToCopy > 0)
                        {
                            MoveLastBytes(buffer.data(), curpos, bytesToCopy);
                            bufferOffset  =bytesToCopy;
                            return false;
                        }
                    }
                    break;
                case PROCESS_STREAM_IMAGE_RAW:
                    if ( (0 == g_lastImageHeader.compressed && bytesToCopy >= g_lastImageHeader.dataSize )
                            || (g_lastImageHeader.compressed > 0 && bytesToCopy >= g_lastImageHeader.compressed))
                    {
                        // put tile into an image data set
                        unsigned char *dstptr = g_lastImageData.data();
                        unsigned char *srcptr = curptr;

                        if (g_lastImageHeader.compressed > 0)
                        {
                            uLongf dstLen = g_lastImageData.size();
                            int err = uncompress(dstptr, &dstLen, srcptr, (uLong)g_lastImageHeader.compressed);

                            //
                            curpos += g_lastImageHeader.compressed;

                            if (err == Z_OK && dstLen == g_lastImageHeader.dataSize)
                            {
                                g_lastImageTimestamp = lHeader.timestamp;
                            } else{
                                LOGE("[CStreamThread] failed to use compressed image, err %d, size %d", err, dstLen);
                            }
                        } else{
                            // tileSize is the same except last tile, it could be in [1; tileSize]
                            memcpy(dstptr, srcptr, sizeof(unsigned char) * g_lastImageHeader.dataSize);

                            //
                            curpos += g_lastImageHeader.dataSize;

                            // check if collect all tiles or timestamp if wrong, then reset reading

                            lStatus = PROCESS_STREAM_WAITING;
                            g_lastImageTimestamp = lHeader.timestamp;
                        }

                    }
                    else if (bytesToCopy > 0)
                    {
                        MoveLastBytes(buffer.data(), curpos, bytesToCopy);
                        bufferOffset = bytesToCopy;
                        return false;
                    }
                    break;
                default:
                    curpos += 1;
            }
        }

        bufferOffset = 0;
        return true;
    }


    bool ProcessStream3(int sd, int startlength, int &bufferOffset, CHeader &lHeader, std::vector<unsigned char> &buffer)
    {

        int lStatus = PROCESS_STREAM_WAITING;

        int curpos = 0;
        unsigned char *readPtr = buffer.data();

        int len = startlength;
        int readingMore = 0;
        bool readingUsed = false;

        while (curpos < len)
        {
            unsigned char *curptr = readPtr + curpos;

            if (readingMore > 0)
            {
                int res = recv(sd, (char*)(readPtr + len), readingMore, MSG_WAITALL);

                if (res < 0)
                {
                    return false;
                }
                len += res; // we have readed more bytes
                readingMore = 0;
                readingUsed = true;
            }


            int bytesToCopy = len - curpos;

            switch(lStatus)
            {
                case PROCESS_STREAM_WAITING:

                    if (bytesToCopy >= 3)
                    {
                        if (CheckMagicNumber(curptr))
                        {
                            if (bytesToCopy >= sizeof(CHeader))
                            {
                                memcpy(&lHeader, curptr, sizeof(CHeader));

                                g_lastSyncTimestamp = (double) lHeader.timestamp;

                                curpos += sizeof(CHeader);
                                lStatus = PROCESS_STREAM_HEADER;

                            } else
                            {
                                LOGI("[CStreamThread] waiting for header");
                                MoveLastBytes(buffer.data(), curpos, bytesToCopy);
                                bufferOffset = bytesToCopy;
                                return false;
                            }
                        } else
                            curpos += 1;
                    }
                    else curpos += 1;

                    break;

                case PROCESS_STREAM_HEADER:
                    if (PACKET_REASON_REGISTER == lHeader.reason || PACKET_REASON_FEEDBACK == lHeader.reason)
                    {
                        LOGI("[CStreamThread] received an invitation %.2f\n", lHeader.timestamp);
                        curpos += 1;
                        lStatus = PROCESS_STREAM_WAITING;
                    }
                    else if (PACKET_REASON_CONTROL == lHeader.reason)
                    {
                        if (bytesToCopy >= sizeof(CSyncControl))
                        {
                            //LOGI("received a control %.2f\n", lHeader.timestamp);

                            memcpy(&g_lastSync, curptr, sizeof(CSyncControl) );

                            curpos += sizeof(CSyncControl);
                            lStatus = PROCESS_STREAM_WAITING;

                            g_lastSyncTimestamp = lHeader.timestamp;

                        } else if (bytesToCopy > 0)
                        {
                            LOGI("[CStreamThread] waiting for control");
                            MoveLastBytes(buffer.data(), curpos, bytesToCopy);
                            bufferOffset  =bytesToCopy;
                            return false;
                        }
                    }
                    else if (PACKET_REASON_IMAGE == lHeader.reason)
                    {
                        if (bytesToCopy >= sizeof(CImageHeader))
                        {
                            LOGI("[CStreamThread] received an image %.2f\n", lHeader.timestamp);

                            memcpy(&g_lastImageHeader, curptr, sizeof(CImageHeader));

                            // prepare space for tiles
                            g_lastImageData.resize( (size_t)g_lastImageHeader.dataSize );

                            curpos += sizeof(CImageHeader);
                            lStatus = PROCESS_STREAM_IMAGE_RAW;
                        }
                        else if (bytesToCopy > 0)
                        {
                            LOGI("[CStreamThread] waiting for image");
                            MoveLastBytes(buffer.data(), curpos, bytesToCopy);
                            bufferOffset  =bytesToCopy;
                            return false;
                        }
                    }
                    if (PACKET_REASON_IMAGE_TILE == lHeader.reason)
                    {
                        if (bytesToCopy >= sizeof(CImageTileHeader))
                        {
                            memcpy(&g_lastImageTileHeader, curptr, sizeof(CImageTileHeader));

                            curpos += sizeof(CImageTileHeader);
                            lStatus = PROCESS_STREAM_TILE;
                        }
                        else if (bytesToCopy > 0)
                        {
                            MoveLastBytes(buffer.data(), curpos, bytesToCopy);
                            bufferOffset = bytesToCopy;
                            return false;
                        }
                    }
                    else if (PACKET_REASON_CAMERA_INFO == lHeader.reason)
                    {
                        if (bytesToCopy >= sizeof(CCameraInfo))
                        {
                            CCameraInfo *pNewInfo = (CCameraInfo*) curptr;

                            g_lastCameras.resize(pNewInfo->totalCount);

                            unsigned char *dst = (unsigned char*) g_lastCameras.data() + pNewInfo->id * sizeof(CCameraInfo);
                            memcpy(dst, curptr, sizeof(CCameraInfo));

                            curpos += sizeof(CCameraInfo);
                            lStatus = PROCESS_STREAM_WAITING;

                            g_lastCamerasTimestamp = lHeader.timestamp;
                        }
                        else if (bytesToCopy > 0)
                        {
                            MoveLastBytes(buffer.data(), curpos, bytesToCopy);
                            bufferOffset  =bytesToCopy;
                            return false;
                        }
                    }
                    break;
                case PROCESS_STREAM_IMAGE_RAW:

                    if ( (0 == g_lastImageHeader.compressed && bytesToCopy >= g_lastImageHeader.dataSize )
                         || (g_lastImageHeader.compressed > 0 && bytesToCopy >= g_lastImageHeader.compressed))
                    {
                        // put tile into an image data set
                        unsigned char *dstptr = g_lastImageData.data();
                        unsigned char *srcptr = curptr;

                        if (g_lastImageHeader.compressed > 0)
                        {
                            uLongf dstLen = g_lastImageData.size();

                            int err = uncompress(dstptr, &dstLen, srcptr, (uLong)g_lastImageHeader.compressed);
                            LOGI("[CStreamThread] received %d, %d, %d", dstptr[0], dstptr[1], dstptr[2]);
                            //
                            curpos += g_lastImageHeader.compressed;

                            if (Z_OK == err && dstLen == g_lastImageHeader.dataSize)
                            {
                                g_lastImageTimestamp = lHeader.timestamp;
                                LOGI("[CStreamThread] image received %d", dstLen);
                            } else{
                                LOGE("[CStreamThread] failed to use compressed image, err %d, compressed %d, size %d", err, g_lastImageHeader.compressed, dstLen);
                            }

                            lStatus = PROCESS_STREAM_WAITING;

                        } else{
                            // tileSize is the same except last tile, it could be in [1; tileSize]
                            memcpy(dstptr, srcptr, sizeof(unsigned char) * g_lastImageHeader.dataSize);

                            //
                            curpos += g_lastImageHeader.dataSize;

                            // check if collect all tiles or timestamp if wrong, then reset reading

                            lStatus = PROCESS_STREAM_WAITING;
                            g_lastImageTimestamp = lHeader.timestamp;
                        }

                    }
                    else if (bytesToCopy > 0)
                    {
                        MoveLastBytes(buffer.data(), curpos, bytesToCopy);
                        bufferOffset = bytesToCopy;
                        return false;
                    }
                    /*
                    if (bytesToCopy >= g_lastImageHeader.dataSize)
                    {
                        // put tile into an image data set
                        unsigned char *dstptr = g_lastImageData.data();
                        unsigned char *srcptr = curptr;

                        // tileSize is the same except last tile, it could be in [1; tileSize]
                        memcpy(dstptr, srcptr, sizeof(unsigned char) * g_lastImageHeader.dataSize);

                        //
                        curpos += g_lastImageHeader.dataSize;

                        // check if collect all tiles or timestamp if wrong, then reset reading

                        lStatus = PROCESS_STREAM_WAITING;
                        g_lastImageTimestamp = lHeader.timestamp;

                    }
                    else if (bytesToCopy > 0)
                    {
                        if (false == readingUsed)
                        {
                            readingMore = g_lastImageHeader.dataSize - bytesToCopy;
                            continue;
                        } else{
                            LOGI("[CStreamThread] waiting for image data");
                            MoveLastBytes(buffer.data(), curpos, bytesToCopy);
                            bufferOffset = bytesToCopy;
                            return false;
                        }
                    }
                     */
                    break;
                default:
                    curpos += 1;
            }
        }

        bufferOffset = 0;
        return true;
    }

    int tempSocket = 0;

    bool SendPoseImid(double timestamp, CDeviceData &lCameraPose)
    {
        if (0 == tempSocket)
            return false;
#ifdef WIN32
		int sendFlag = MSG_DONTROUTE;
#else
        int sendFlag = MSG_NOSIGNAL;
#endif
        CHeader lHeader;
        Network::FillBufferWithHeader((unsigned char*)&lHeader, PACKET_REASON_CAMERA, timestamp);

        int iSendResult = send(tempSocket, (char*) &lHeader, sizeof(Network::CHeader), sendFlag);

        if (-1 == iSendResult)
        {
            LOGE("[CStreamThread] send failed %d\n", errno);
            return false;
        }

        char *ptr = (char*) &lCameraPose;
        int numberOfBlocks = sizeof(Network::CDeviceData) / 8;

        for (int i=0; i<numberOfBlocks; ++i, ptr += 8)
        {
            iSendResult = send(tempSocket, ptr, 8, sendFlag);

            if (-1 == iSendResult)
            {
                LOGE("[CStreamThread] send failed %d\n", errno);
                return false;
            }
        }
        return true;
    }

	void MainServerFunc()
    {
		high_resolution_clock::time_point _start;
		_start = high_resolution_clock::now();

        SetLastSyncAddress(0);

        //
		int mSocket = StartServer(NETWROK_SERVER_PORT);

        if (mSocket <= 0)
        {
            return;
        }

        while(1)
        {
            LOGI("[CStreamThread] Waiting for connections\n");

            sockaddr_in     clientAddr;
            
#ifdef WIN32
			int socklen = sizeof(sockaddr_in);
			memset(&clientAddr, 0, sizeof(sockaddr_in));
#else
			socklen_t       socklen = sizeof(sockaddr_in);
            bzero((char*) &clientAddr, sizeof(sockaddr_in));
#endif

            int lSocket = accept(mSocket, (sockaddr*)&clientAddr, &socklen);
            if (lSocket >= 0)
            {
                sockaddr_in	lAddr;

                std::vector<unsigned char>  buffer(65536 + 8, 0);

                //sDataBuffer mDataBuffer;
#ifdef WIN32
				memset(&lAddr, 0, sizeof(sockaddr_in));
#else
                bzero((char *)&lAddr, sizeof(sockaddr_in));
#endif

                if( getsockname(lSocket, (struct sockaddr*)&lAddr, &socklen) < 0 )
                {
                    LOGE("[CStreamThread] failed to get sockname\n");
                    return;
                }

                LOGI("[CStreamThread] Connection established\n");
                //ServerStartedOn = (double)GetNanoSeconds();

                // DONE: change sync state and exchange client address with UI
                SetLastSyncAddress(clientAddr.sin_addr.s_addr);

				_start = high_resolution_clock::now();

                tempSocket = lSocket;

                int iResult, iSendResult;

                CHeader     feedbackPacket;
                FillBufferWithHeader((unsigned char*)&feedbackPacket, PACKET_REASON_FEEDBACK, 0.0f);

                int bufferOffset = 0;

                double timestamp = 0.0;

                Network::CHeader    recvHeader;
                Network::CHeader    lHeader;

                recvHeader.reason = 0;
                recvHeader.timestamp = 0.0f;

                double lastPoseStamp = 0.0;
                Network::CDeviceData lCameraPose;

                char data[8];
#ifdef WIN32
				const int sendFlag = MSG_DONTROUTE;
#else
                const int sendFlag = MSG_NOSIGNAL;
#endif
                // reading a big image data + header + image header
                unsigned int readCapacity = (unsigned)buffer.size() / 2 + 24; // 1k reading  buffer.size() - bufferOffset;

                for (;;)
                {
                    unsigned char *readPtr = buffer.data() + bufferOffset;

                    // recv
                    iResult = recv(lSocket, (char*) readPtr, readCapacity, 0);

                    bool hasChange = false;

                    if (iResult > 0)
                    {
                        //processStream(iResult, bufferOffset, buffer);
                        if (true == ProcessStream3(lSocket, iResult, bufferOffset, recvHeader, buffer) )
                        {
                            hasChange = true;
                            timestamp = g_lastSyncTimestamp;
                        }

                    }
                    else if (iResult == 0)
                    {
                        // closing a connection
                    }
                    else {
                        LOGE("[CStreamThread] recv failed %d\n", errno);
                        break;
                    }


                    if (false == hasChange)
                        timestamp += 0.001;
                    bool hasPose = false;
                    /*
                    bool hasPose = ExchangeReadDeviceData(lastPoseStamp, lCameraPose );

                    while (false == hasPose)
                    {
                        std::this_thread::yield();
                        hasPose = ExchangeReadDeviceData(lastPoseStamp, lCameraPose );
                    }
*/
                    if (hasPose)
                    {

                        Network::FillBufferWithHeader((unsigned char*)&lHeader, PACKET_REASON_CAMERA, timestamp);

                        iSendResult = send(lSocket, (char*) &lHeader, sizeof(Network::CHeader), sendFlag);

                        if (-1 == iSendResult)
                        {
                            LOGE("[CStreamThread] send failed %d\n", errno);
                            break;
                        }

                        char *ptr = (char*) &lCameraPose;
                        int numberOfBlocks = sizeof(Network::CDeviceData) / 8;

                        for (int i=0; i<numberOfBlocks; ++i, ptr += 8)
                        {
                            iSendResult = send(lSocket, ptr, 8, sendFlag);

                            if (-1 == iSendResult)
                            {
                                LOGE("[CStreamThread] send failed %d\n", errno);
                                break;
                            }
                        }

                    }
                    else
                    {
                        // send from queue
                        Network::FillBufferWithHeader((unsigned char*)&lHeader, PACKET_REASON_FEEDBACK, timestamp);

                        iSendResult = send(lSocket, (char*) &lHeader, sizeof(Network::CHeader), sendFlag);

                        if (-1 == iSendResult)
                        {
                            LOGE("[CStreamThread] send failed %d\n", errno);
                            break;
                        }
                    }


                    // sleep or yield
                    // NETWORK_RATE
                    //std::this_thread::yield();
					std::this_thread::sleep_for(std::chrono::milliseconds(33));

                    //usleep( 100 );
                }
            }

            shutdown(lSocket, 2);

#ifdef WIN32
			closesocket(lSocket);
#else
            close(lSocket);
#endif
            
			milliseconds elapsed = std::chrono::duration_cast<milliseconds>(high_resolution_clock::now() - _start);
            //LOGI("Connection closed, connection time = %d ms\n", elapsed);

            // update UI
            SetLastSyncAddress(0);
        }

	}

}