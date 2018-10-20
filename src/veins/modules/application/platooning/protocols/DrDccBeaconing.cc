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

#include "veins/modules/application/platooning/protocols/DrDccBeaconing.h"
#include "omnetpp/cstddev.h"

#include "veins/modules/utility/Consts80211p.h"
#include "veins/modules/utility/ConstsPhy.h"
#include "veins/modules/mac/ieee80211p/Mac1609_4.h"
#include "veins/modules/messages/PhyControlMessage_m.h"
//register the module class to omnetpp
Define_Module(DrDccBeaconing);

namespace{
    simtime_t NDL_time;
    simtime_t NDL_minDccSampling;
    simtime_t DCC_measure_interval_Tm;

    const double NDL_maxChannelLoad = 0.6;
    const double NDL_refChannelLoad = 0.4;
    const double NDL_minChannelLoad = 0.15;

}

DrDccBeaconing::DrDccBeaconing() :
    BaseProtocol() {

    NDL_time = 1.00;
    NDL_minDccSampling = 1.00;
    DCC_measure_interval_Tm = 1.00;

    doMeasureEvt = new cMessage("doMeasure");
    doDccEvt = new cMessage("doDcc");
}

void DrDccBeaconing::initialize(int stage) {
    BaseProtocol::initialize(stage);
    std::string myName = getParentModule()->getFullName();

    if(stage == 0) {

        lastBeaconAt = -1;
        //get gates
        lowerLayerIn = findGate("lowerLayerIn");
        lowerLayerOut = findGate("lowerLayerOut");

        //get pointer to mac
        mac = FindModule<Mac1609_4 *>::findSubModule(getParentModule());

        beaconingInterval = SimTime(par("beaconingInterval").doubleValue());
        bitrate = par("bitrate").doubleValue();
        level1bitrate = par("level1bitrate").doubleValue();
        level2bitrate = par("level2bitrate").doubleValue();
        level3bitrate = par("level3bitrate").doubleValue();
        level4bitrate = par("level4bitrate").doubleValue();
        level5bitrate = par("level5bitrate").doubleValue();

//        std::cout << level3bitrate <<std::endl;

        channelLoad.reset(NDL_time / DCC_measure_interval_Tm);

        beaconIntervalIdOut.setName("beaconIntervalId");
        beaconIntervalOut.setName("beaconInterval");

        idleChannel = true;
        dccStartBusy = simTime();
        dccStartIdle = simTime();
        dccBusyTime = SimTime(0);
        dccIdleTime = SimTime(0);
        lastBusyTime = std::pair<simtime_t, simtime_t> (0, 0);
        DBG << "DccBeaconing has been initialized!" << std::endl;

        simtime_t firstMeasureAt = simTime() + uniform(0, DCC_measure_interval_Tm);
        simtime_t firstDccAt = firstMeasureAt + uniform(0, NDL_minDccSampling - DCC_measure_interval_Tm);
        scheduleAt(firstDccAt, sendBeacon);
        //first measurement
        scheduleAt(firstMeasureAt, doMeasureEvt);
        scheduleAt(firstDccAt, doDccEvt);
        std::cout<< "drrrr"<<std::endl;

    }
    if (stage == 1){
        mac->setMCS(getMCS(bitrate, BW_OFDM_10_MHZ));
        mac->setMCS(getMCS(level1bitrate, BW_OFDM_10_MHZ));
        mac->setMCS(getMCS(level2bitrate, BW_OFDM_10_MHZ));
        mac->setMCS(getMCS(level3bitrate, BW_OFDM_10_MHZ));
        mac->setMCS(getMCS(level4bitrate, BW_OFDM_10_MHZ));
        mac->setMCS(getMCS(level5bitrate, BW_OFDM_10_MHZ));
         }
}

void DrDccBeaconing::finish() {
    if (sendBeacon -> isScheduled()) cancelEvent(sendBeacon);
    if (doMeasureEvt->isScheduled()) cancelEvent(doMeasureEvt);
    if (doDccEvt->isScheduled()) cancelEvent(doDccEvt);
}


DrDccBeaconing::~DrDccBeaconing() {
    delete sendBeacon;
    delete doMeasureEvt;
    delete doDccEvt;
}

void DrDccBeaconing::handleSelfMsg(cMessage* msg) {
    //handleSelMsg() and received() used to receive cMessage

    BaseProtocol::handleSelfMsg(msg);

    if (msg == sendBeacon) {
        // send beacon
        sendPlatooningMessage(-1);

        //record actual beacon interval
        if (lastBeaconAt != -1){
        beaconIntervalOut.record(simTime() - lastBeaconAt);
        beaconIntervalIdOut.record(myId);
        }
        lastBeaconAt = simTime();
        // schedule next beacon
        scheduleAt(simTime() + beaconingInterval * uniform(1.00, 1.01), sendBeacon);
        //currentPacketIntervalOut.record(currentPacketInterval);

    }

    else if (msg == doMeasureEvt) {
        doMeasure();
    }
    else if (msg == doDccEvt) {
        doDcc();
    }

}

void DrDccBeaconing::doMeasure() {

    // calculate channel busy ratio (busyTime/totalTime)
    simtime_t idle = dccIdleTime;
    simtime_t busy = dccBusyTime;
    if (idleChannel) {
        idle += std::max(simtime_t(0), simTime() - dccStartIdle);
    }
    else {
        busy += std::max(simtime_t(0), simTime() - dccStartBusy);
    }

    std::pair<simtime_t, simtime_t> busyTime = std::pair<simtime_t, simtime_t>(busy, idle + busy);
    double busyRatio = (busyTime.first - lastBusyTime.first).dbl()/(busyTime.second - lastBusyTime.second).dbl();
    //busyRatioOut.record(busyRatio);

    lastBusyTime = busyTime;

    channelLoad.insert(busyRatio);

    scheduleAt(simTime() + DCC_measure_interval_Tm, doMeasureEvt);
}

void DrDccBeaconing::doDcc() {

    double clMin = channelLoad.min();
    double clMax = channelLoad.max();

    UnicastMessage *unicast;
    unicast = new UnicastMessage("", BEACON_TYPE);
    unicast->setDestination(-1);
    unicast->setPriority(priority);

    PlatooningBeacon *pkt;
    pkt = new PlatooningBeacon();

    PhyControlMessage *ctrl = new PhyControlMessage();
    if (clMin >= NDL_maxChannelLoad){
        ctrl->setMcs(getMCS(level3bitrate, BW_OFDM_10_MHZ));
    }
    else if ((clMax < NDL_maxChannelLoad) && (clMin >= NDL_refChannelLoad)){
        ctrl->setMcs(getMCS(level2bitrate, BW_OFDM_10_MHZ));
    }
    else if ((clMax < NDL_refChannelLoad) && (clMin >= NDL_minChannelLoad)){
        ctrl->setMcs(getMCS(bitrate, BW_OFDM_10_MHZ));
    }
    else if (clMax < NDL_refChannelLoad){
        ctrl->setMcs(getMCS(level1bitrate, BW_OFDM_10_MHZ));
    }
    pkt->setControlInfo(ctrl);
    unicast->encapsulate(pkt);
    sendDown(unicast);

    scheduleAt(simTime() + NDL_minDccSampling, doDccEvt);

}
void DrDccBeaconing::messageReceived(PlatooningBeacon *pkt, UnicastMessage *unicast) {
   //nothing to do
}



void DrDccBeaconing::channelBusyStart() {
    dccIdleTime += simTime() - dccStartIdle;
    dccStartBusy = simTime();
    idleChannel = false;
}

void DrDccBeaconing::channelIdleStart() {

    dccBusyTime += simTime() - dccStartBusy;
    dccStartIdle = simTime();
    idleChannel = true;

}
