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

#include "veins/modules/application/platooning/protocols/TxDccBeaconing.h"
#include "omnetpp/cstddev.h"

#include "veins/modules/utility/Consts80211p.h"
#include "veins/modules/utility/ConstsPhy.h"
#include "veins/modules/mac/ieee80211p/Mac1609_4.h"
#include "veins/modules/messages/PhyControlMessage_m.h"
//register the module class to omnetpp
Define_Module(TxDccBeaconing);

namespace{
    simtime_t NDL_time;
    simtime_t NDL_minDccSampling;
    simtime_t DCC_measure_interval_Tm;

    const double NDL_maxChannelLoad = 0.4;
    const double NDL_minChannelLoad = 0.2;
}

TxDccBeaconing::TxDccBeaconing() :
    BaseProtocol() {

    NDL_time = 1.00;
    NDL_minDccSampling = 1.00;
    DCC_measure_interval_Tm = 1.00;

    doMeasureEvt = new cMessage("doMeasure");
    doDccEvt = new cMessage("doDcc");
}

void TxDccBeaconing::initialize(int stage) {
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
        TxPower = par("TxPower").doubleValue();
        level2TxPower  = par("level2TxPower").doubleValue();
        level3TxPower  = par("level3TxPower").doubleValue();
        level4TxPower  = par("level4TxPower").doubleValue();

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
        std::cout<<"txxxx"<<endl;
    }
    if (stage == 1){
        mac->setTxPower(TxPower);
        mac->setTxPower(level2TxPower);
        mac->setTxPower(level3TxPower);
        mac->setTxPower(level4TxPower);
         }
}

void TxDccBeaconing::finish() {
    if (sendBeacon -> isScheduled()) cancelEvent(sendBeacon);
    if (doMeasureEvt->isScheduled()) cancelEvent(doMeasureEvt);
    if (doDccEvt->isScheduled()) cancelEvent(doDccEvt);
}


TxDccBeaconing::~TxDccBeaconing() {
    delete sendBeacon;
    delete doMeasureEvt;
    delete doDccEvt;
}

void TxDccBeaconing::handleSelfMsg(cMessage* msg) {
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

void TxDccBeaconing::doMeasure() {

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

void TxDccBeaconing::doDcc() {

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
        ctrl->setTxPower_mW(level4TxPower);
    }
    else if ((clMin < NDL_maxChannelLoad) && (clMin >= NDL_minChannelLoad)){
        ctrl->setTxPower_mW(level3TxPower);
    }
    else if (clMax < NDL_minChannelLoad){
        ctrl->setTxPower_mW(level2TxPower);
    }
    pkt->setControlInfo(ctrl);
    unicast->encapsulate(pkt);
    sendDown(unicast);

    scheduleAt(simTime() + NDL_minDccSampling, doDccEvt);

}
void TxDccBeaconing::messageReceived(PlatooningBeacon *pkt, UnicastMessage *unicast) {
   //nothing to do
}



void TxDccBeaconing::channelBusyStart() {
    dccIdleTime += simTime() - dccStartIdle;
    dccStartBusy = simTime();
    idleChannel = false;
}

void TxDccBeaconing::channelIdleStart() {

    dccBusyTime += simTime() - dccStartBusy;
    dccStartIdle = simTime();
    idleChannel = true;

}
