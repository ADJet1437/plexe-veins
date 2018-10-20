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

#include "veins/modules/application/platooning/protocols/DccBeaconing.h"
#include "omnetpp/cstddev.h"
//register the module class to omnetpp
Define_Module(DccBeaconing);

namespace {
    simtime_t NDL_minPacketInterval;
    simtime_t NDL_refPacketInterval;
    simtime_t NDL_maxPacketInterval;
    simtime_t NDL_timeUp;
    simtime_t NDL_timeDown;
    simtime_t DCC_measure_interval_Tm;
    simtime_t NDL_minDccSampling;
    const double NDL_minChannelLoad = 0.15;
    const double NDL_maxChannelLoad = 0.40;

    const size_t STATE_RELAXED = 0;
    const size_t STATE_ACTIVE = 1;
    const size_t STATE_RESTRICTED = 2;
    const size_t STATE_UNDEF = 3;
}

DccBeaconing::DccBeaconing() :
    BaseProtocol() {

    NDL_minPacketInterval = 0.04;
    NDL_refPacketInterval = 0.50;
    NDL_maxPacketInterval = 1.00;
    NDL_timeUp = 1.00;
    NDL_timeDown = 5.00;
    DCC_measure_interval_Tm = 1.00;
    NDL_minDccSampling = 1.00;

    assert(NDL_timeUp.raw() % DCC_measure_interval_Tm.raw() == 0);
    assert(NDL_timeDown.raw() % DCC_measure_interval_Tm.raw() == 0);
    assert(NDL_minDccSampling >= DCC_measure_interval_Tm);

    states.insert(std::make_pair(STATE_RELAXED, State(NDL_minPacketInterval)));
    states.insert(std::make_pair(STATE_ACTIVE, State(NDL_refPacketInterval)));
    states.insert(std::make_pair(STATE_RESTRICTED, State(NDL_maxPacketInterval)));

    doMeasureEvt = new cMessage("doMeasure");
    doDccEvt = new cMessage("doDcc");
}

void DccBeaconing::initialize(int stage) {
    BaseProtocol::initialize(stage);


    std::string myName = getParentModule()->getFullName();

    if(stage == 0) {
        using namespace std;
        cout << "start using DCC" << endl;
        currentState = STATE_UNDEF;
        currentPacketInterval = 0;
        setCurrentState(STATE_RELAXED);

        channelLoadInTimeUp.reset(NDL_timeUp / DCC_measure_interval_Tm);
        channelLoadInTimeDown.reset(NDL_timeDown / DCC_measure_interval_Tm);

        lastBeaconAt = -1;

        neighborExpiry = par("neighborExpiry");
        neighbors.clear();

        beaconIntervalIdOut.setName("beaconIntervalId");
        beaconIntervalOut.setName("beaconInterval");
        //busyRatioOut.setName("busyRatio");
        //currentPacketIntervalOut.setName("currentPacketInterval");
        //currentStateOut.setName("currentState");
        //IRTOut.setName("InterReceptionTime");

        idleChannel = true;
        dccStartBusy = simTime();
        dccStartIdle = simTime();
        dccBusyTime = SimTime(0);
        dccIdleTime = SimTime(0);
        lastBusyTime = std::pair<simtime_t, simtime_t> (0, 0);

        DBG << "DccBeaconing has been initialized!" << std::endl;
    }
    if (stage == 1){
           startCommunications();
         }
}

void DccBeaconing::startCommunications() {
    // startup protocol
    simtime_t firstMeasureAt = simTime() + uniform(0, DCC_measure_interval_Tm);
    simtime_t firstDccAt = firstMeasureAt + uniform(0, NDL_minDccSampling - DCC_measure_interval_Tm);
    scheduleAt(firstDccAt, sendBeacon);
    //first measurement
    scheduleAt(firstMeasureAt, doMeasureEvt);
    scheduleAt(firstDccAt, doDccEvt);
}

void DccBeaconing::finish() {
    if (sendBeacon -> isScheduled()) cancelEvent(sendBeacon);
    if (doMeasureEvt->isScheduled()) cancelEvent(doMeasureEvt);
    if (doDccEvt->isScheduled()) cancelEvent(doDccEvt);
}


DccBeaconing::~DccBeaconing() {
    delete sendBeacon;
    delete doMeasureEvt;
    delete doDccEvt;
}

void DccBeaconing::handleSelfMsg(cMessage* msg) {
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
        scheduleAt(simTime() + currentPacketInterval * uniform(1.00, 1.01), sendBeacon);
        //currentPacketIntervalOut.record(currentPacketInterval);

    }

    else if (msg == doMeasureEvt) {
        doMeasure();
    }
    else if (msg == doDccEvt) {
        doDcc();
    }

}

void DccBeaconing::messageReceived(PlatooningBeacon *pkt, UnicastMessage *unicast){
    //nothing to do for messageReceived
}

void DccBeaconing::doMeasure() {

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

    channelLoadInTimeUp.insert(busyRatio);
    channelLoadInTimeDown.insert(busyRatio);

    scheduleAt(simTime() + DCC_measure_interval_Tm, doMeasureEvt);
}

void DccBeaconing::doDcc() {
    double clMinInTimeUp = channelLoadInTimeUp.min();
    double clMaxInTimeDown = channelLoadInTimeDown.max();

    if ((currentState == STATE_RELAXED) && (clMinInTimeUp >= NDL_minChannelLoad)) setCurrentState(STATE_ACTIVE);
    else if ((currentState == STATE_RESTRICTED) && (clMaxInTimeDown < NDL_maxChannelLoad)) setCurrentState(STATE_ACTIVE);
    else if ((currentState == STATE_ACTIVE) && (clMaxInTimeDown < NDL_minChannelLoad)) setCurrentState(STATE_RELAXED);
    else if ((currentState == STATE_ACTIVE) && (clMinInTimeUp >= NDL_maxChannelLoad)) setCurrentState(STATE_RESTRICTED);

    scheduleAt(simTime() + NDL_minDccSampling, doDccEvt);
}

void DccBeaconing::setCurrentState(size_t state) {
    currentState = state;
    simtime_t stateInterval = states.find(currentState)->second.asPacketInterval;
    if (stateInterval != 0) {
        currentPacketInterval = stateInterval;
    }
    if (sendBeacon->isScheduled()){
        // re-schedule next beacon
        simtime_t newScheduleTime = std::max(simTime(), lastBeaconAt + currentPacketInterval * uniform(1.00, 1.01));
        cancelEvent(sendBeacon);
        scheduleAt(newScheduleTime, sendBeacon);
    }
}


void DccBeaconing::channelBusyStart() {
    dccIdleTime += simTime() - dccStartIdle;
    dccStartBusy = simTime();
    idleChannel = false;
}

void DccBeaconing::channelIdleStart() {

    dccBusyTime += simTime() - dccStartBusy;
    dccStartIdle = simTime();
    idleChannel = true;

}
