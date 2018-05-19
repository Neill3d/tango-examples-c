/*
 * UDPThread.h
 *
 *  Created on: 2015. 3. 3.
 *      Author: youngmok
 */

#ifndef UDPTHREAD_H_
#define UDPTHREAD_H_

#define UDP_FILE_LOG_FLAG 0

#include <iostream> 		// cout
#include <sstream>  		// parsing
#include <sys/socket.h>
#include <netinet/in.h>
#include <fcntl.h>
#include <stdio.h>			// for File output
#include <sys/time.h>		// Time measurement

#include <pthread.h>

#include <vector>
#include "NetworkTango.h"
#include "NetworkUtils.h"

using namespace std;

namespace Network {

#define NETWORK_MOBU_TRACKER	9001
#define NETWORK_MOBU_COMMANDS	8887

#define NETWORK_SYNC_PORT       8889
#define NETWORK_IMAGE_PORT      8886
#define NETWORK_IMAGE_SERVER_PORT	8885
//#define NETWORK_PACKET_SIZE     1500

    //////////////////////////////////////////////////////////////////////////////////
    // CUDPThread - class for managing a datagram receiving in a separate thread

	class CUDPThread
    {
	protected:

		int 	mEntryFuncId;

		//FILE *ofp;				// File log
		pthread_t 				_thread;        // thread

		pthread_rwlock_t		_rwlock;

		struct timeval tvalNow, tvalInit;    // for time stamp
		double getTimeElapsed(struct timeval end, struct timeval start); // For time stamp

	protected:

        int          mSocketPort;
        Socket      mSocket;

        // receiving buffer
        unsigned int 	mBufferLen;
        unsigned char   *mBuffer;        // UDP

        // last correct receiving address
		float            mLastTimeStamp;
        Address     	mAddressRecv;

		//void InternalThreadEntry();

		bool CheckMagicNumber(const unsigned char *buffer);

		void EmptyBuffer();

	public:

		//! a constructor
		CUDPThread(int entryFuncId = 0, int _port = 8889, int _bufferLen=1500);

		//! a destructor
		virtual ~CUDPThread();

		bool StartInternalThread();
		void WaitForInternalThreadToExit();

		static void *InternalThreadEntryFunc(void *This);

		const int GetEntryFuncId() const {
			return mEntryFuncId;
		}

	};

    ////////////////////////////////////////////////////////////////////////////////
    // class for receiving and manage the scene sync states

    class CUDPThreadSync : public CUDPThread
    {
    protected:

        // last sync point
		int 			mCameraInfoCounter;
		double			mLastCameraInfoTimestamp;

		struct ReceivedData
		{
			double 		timestamp;
			Address		sender;

			int 	numRegPackets;

			int 					numStates;	// last received states
			CSyncControl 			lastState;

			int 		numCameraInfo;	// packets count
			double 		camInfoTimeStamp;
			int 		camInfoCounter;
			std::vector<CCameraInfo>		cameraInfo;
		};

		ReceivedData			mReceivedData;

        CSyncControl			mSyncState;

        // last received scene information
        std::vector<CStaticName>        mTakeNames;
		std::vector<CCameraInfo>		mCameraInfo;


        bool OnPacketReceived(Address &sender, CHeader *pHeader,
                                      unsigned int bodyLen, unsigned char *bodyData);


    public:

		//! a constructor
		CUDPThreadSync();

		void InternalThreadEntry();

        // TODO: interface to query a recevied image, to query a received mobu sync packet

        bool GetSyncData(const float localtimestamp, float &synctimestamp, Address &lastAddress, CSyncControl &sync);
        //bool GetTakeNames(std::vector<std::string> &names);
        //bool GetCameraNames(std::vector<std::string> &names);

		bool GetCameraData(double &synctimestamp, int &syncloadedcounter, std::vector<CCameraInfo> &cameraVector);

        bool GetLastSender(float &lasttimestamp, Network::Address &lastAddress);


        //bool SendCameraData(const CPacketCamera &data);
        //bool SendTrigger(const int triggerId);
    };

    ///////////////////////////////////////////////////////////////////////////
    // class for receiving and manage viewport image in tiles

    // thread will receive one image header (per 5 seconds) to connect
    //  if connection occurs, image thread should send feedback to image server one per second


    class CUDPThreadImage : public CUDPThread
    {
	private:

		// make an image from tiles
		struct ImageData
		{
			CImageHeader		header;

			//
			float 				lastTimestamp;

			// allocated buffer for put all tiles in
			unsigned char       *data;

			// mask for updated tiles
            int     totalTiles;
			int 	numberOfReceivedTiles;
			bool 	receivedTiles[MAX_NUMBER_OF_TILES];

			ImageData()
			{
				data = nullptr;
				numberOfReceivedTiles = 0;
				lastTimestamp = 0.0f;
			}
		};

		// TODO: compute number of tiles inside the image

        int                     currIndex;

		ImageData              recvBuffer[2];   // buffer to compose image from tiles

		ImageData               mLastReceivedData;  // last fully composed image

		void AllocImageData(ImageData &data);
		void FreeImageData(ImageData &data);

		void CopyImageData(ImageData &dst, const ImageData &src);
		void ClearImageData(ImageData &data);

    protected:

        bool OnPacketReceived(Address &sender, CHeader *pHeader,
                                      unsigned int bodyLen, unsigned char *bodyData);


    public:

		//! a constructor
		CUDPThreadImage();
		//! a destructor
		virtual ~CUDPThreadImage();

		void InternalThreadEntry();

        bool GetLastImageData(float localtimestamp, float &lastreceivedtimestamp, Address &lastAddress, CImageHeader &header, char *buffer, int bufferLen);
    };

}

#endif /* UDPTHREAD_H_ */
