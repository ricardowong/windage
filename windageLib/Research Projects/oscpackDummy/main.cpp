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

#include "osc/OscReceivedElements.h"
#include "osc/OscPrintReceivedElements.h"

#include "ip/UdpSocket.h"
#include "ip/PacketListener.h"

#pragma comment(lib, "WS2_32.lib")
#pragma comment(lib, "winmm.lib")

const int PORT=7239;

class DumpPacketListener : public PacketListener
{
protected:

	virtual void ProcessPacket(const char* data, int size, const IpEndpointName& remoteEndpoint)
	{
		char address[100];
		remoteEndpoint.AddressAndPortAsString(address);
		std::cout << "User IP : " << address << std::endl;
		std::cout << osc::ReceivedPacket(data, size);
	}

};

void main()
{
	DumpPacketListener listener;
	UdpListeningReceiveSocket s(IpEndpointName( IpEndpointName::ANY_ADDRESS, PORT ), &listener);

	std::cout << "listening for input on port " << PORT << "...\n";
	std::cout << "press ctrl-c to end" << std::endl;

	s.RunUntilSigInt();

	std::cout << "finishing" << std::endl;
}