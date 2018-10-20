//
// Copyright (C) 2012 Stefan Joerer <stefan.joerer@uibk.ac.at>
// Copyright (C) 2014 Michele Segata <segata@ccs-labs.org>
//
// Documentation for these modules is at http://veins.car2x.org/
//
// This program is free software; you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation; either version 2 of the License, or
// (at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with this program; if not, write to the Free Software
// Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
//

package org.car2x.veins.modules.application.platooning.protocols;

import org.car2x.veins.modules.application.platooning.protocols.BaseProtocol;

simple DrDccBeaconing like BaseProtocol
{
    parameters:
        @class(DrDccBeaconing);
        volatile double beaconingInterval @unit(seconds);// = default(0.1 seconds);
        //period of time between the recording of two statistics
        //double statisticsPeriod @unit("s");// = default(1s);
        //priority (AC) for the messages. 0 = AC_BK, 3 = AC_VO
        int priority;// = default(0);
        //size of platooning messages
        int packetSize;// = default(25);
        int headerLength @unit("bit") = default(0bit);
//        //leader transmission power
//        double leaderTxPower @unit("mW") = default(100mW);
        //bitrate in bps
        int bitrate @unit("bps") = default(6Mbps);
        int level1bitrate @unit("bps") = 3Mbps;
        int level2bitrate @unit("bps") = 9Mbps;
        int level3bitrate @unit("bps") = 12Mbps;
        int level4bitrate @unit("bps") = 18Mbps;
        int level5bitrate @unit("bps") = 24Mbps;
        //followers transmission power
        //double followerTxPower @unit("mW") = default(10mW);
        //determine wheter tx power control should be used. if set to false, followers
        //will use the transmission power of the leader
        //int useTxPowerControl = default(0);
        //send actual or controller-computed acceleration
        bool useControllerAcceleration = default(true);

        double neighborExpiry @unit("s") = default(1.0s);  // time after which a neighbor entry expire
    gates:
        input lowerLayerIn;
        output lowerLayerOut;
        input upperLayerIn[10];
        output upperLayerOut[10];
        input lowerControlIn;
        output lowerControlOut;
        input upperControlIn;
        output upperControlOut;
}
