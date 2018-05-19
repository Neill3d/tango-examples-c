/*
 * StreamThread.h
 *
 * Created on: 2018.3.3
 *  Sergei Solokhin (Neill3d)
 *
 * based on
 *  Created on: 2015. 3. 3.
 *      Author: youngmok
 */

#ifndef STREAM_THREAD_H_
#define STREAM_THREAD_H_


#include <iostream> 		// cout
#include <sstream>  		// parsing

#include <fcntl.h>
#include <stdio.h>			// for File output

#include <vector>

#include "NetworkTango.h"

using namespace std;

namespace Network {

    //////////////////////////////////////////////////////////////////////////////////
    
	int NewTCPSocket();
	void CloseTCPSocket(int socketHandle);
	bool BindSocketToPort(int sd, unsigned short port);
	bool ListenOnSocket(int sd, int backlog);

	int StartServer(unsigned short port);

	bool ProcessStream(int bytesRead, int &bufferOffset, std::vector<unsigned char> &buffer);

	// run it inside a thread
	void MainServerFunc();

    uint32_t GetLastSyncAddress();

    double GetLastSyncTimestamp();
    void GetLastSyncState(CSyncControl &state);

    double GetLastImageTimestamp();
    void GetLastImageHeader(CImageHeader &header);
    size_t GetLastImageSize();
    void GetLastImageData(unsigned char *buffer);

    //

    double GetLastSyncCamerasTimeStamp();
    const std::vector<CCameraInfo> &GetLastSyncCamerasVector();

/*
    bool ExchangeWriteDeviceData(double timestamp, Network::CDeviceData &data);
    bool ExchangeReadDeviceData(double &lastreadStamp, Network::CDeviceData &data);
*/
    //
    bool SendPoseImid(double timestamp, CDeviceData &lCameraPose);
}

#endif /* UDPTHREAD_H_ */
