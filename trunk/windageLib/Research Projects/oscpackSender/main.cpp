/* ========================================================================
 * PROJECT: windage Library
 * ========================================================================
 * This work is based on the original windage Library developed by
 *   Woonhyuk Baek (wbaek@gist.ac.kr / windage@live.com)
 *   Woontack Woo (wwoo@gist.ac.kr)
 *   U-VR Lab, GIST of Gwangju in Korea.
 *   http://windage.googlecode.com/
 *   http://uvr.gist.ac.kr/
 *
 * Copyright of the derived and new portions of this work
 *     (C) 2009 GIST U-VR Lab.
 *
 * This framework is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This framework is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this framework; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 * For further information please contact 
 *   Woonhyuk Baek
 *   <windage@live.com>
 *   GIST U-VR Lab.
 *   Department of Information and Communication
 *   Gwangju Institute of Science and Technology
 *   1, Oryong-dong, Buk-gu, Gwangju
 *   South Korea
 * ========================================================================
 ** @author   Woonhyuk Baek
 * ======================================================================== */

#include <iostream>
#include <windows.h>
#include <process.h>
#include <omp.h>

#include "osc/OscOutboundPacketStream.h"
#include "ip/UdpSocket.h"

#pragma comment(lib, "WS2_32.lib")
#pragma comment(lib, "winmm.lib")

const char* ADDRESS = "127.0.0.1";
const int PORT = 7239;
const int OUTPUT_BUFFER_SIZE = 1024;

unsigned int WINAPI SystemSendingThread(void* pArg)
{
	UdpTransmitSocket* transmitSocket = (UdpTransmitSocket*)pArg;

	char buffer[OUTPUT_BUFFER_SIZE];
	osc::OutboundPacketStream pSystem(buffer, OUTPUT_BUFFER_SIZE);

	int index = 0;
	while(true)
	{
		pSystem << osc::BeginBundleImmediate
			<< osc::BeginMessage("automatical-message") 
			<< index
			<< osc::EndMessage << osc::EndBundle;
		transmitSocket->Send( pSystem.Data(), pSystem.Size() );
		pSystem.Clear();

		Sleep(1000);
		index++;
	}
}

void main()
{
	UdpTransmitSocket transmitSocket(IpEndpointName( ADDRESS, PORT ));

	char buffer[OUTPUT_BUFFER_SIZE];
	osc::OutboundPacketStream pUser(buffer, OUTPUT_BUFFER_SIZE);

	std::cout << "sending for input on port " << PORT << "...\n";
	std::cout << "press ctrl-c to end" << std::endl;
	
	HANDLE nHandle = (HANDLE)_beginthreadex(NULL, 0, SystemSendingThread, (void*)&transmitSocket, 0, NULL);

	while(true)
	{
		char message[100];
		std::cin >> message;

		pUser << osc::BeginBundleImmediate
			<< osc::BeginMessage("user-message")
			<< message
			<< osc::EndMessage << osc::EndBundle;
		transmitSocket.Send( pUser.Data(), pUser.Size() );
		pUser.Clear();
	}
}