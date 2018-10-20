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

#include "veins/modules/application/platooning/protocols/DccBeaconing_new.h"
#include "omnetpp/cstddev.h"
//define_Module(classname)
Define_Module(DccBeaconing_new);

namespace {
    simtime_t NDL_minPacketInterval;
    simtime_t NDL_secPacketInterval;
    simtime_t NDL_refPacketInterval;
    simtime_t NDL_thiPacketInterval;
    simtime_t NDL_maxPacketInterval;
    simtime_t NDL_timeUp;
    simtime_t NDL_timeDown;
    simtime_t DCC_measure_interval_Tm;
    simtime_t NDL_minDccSampling;
    const double NDL_firChannelLoad = 0.15;
    const double NDL_minChannelLoad = 0.25;
    const double NDL_maxChannelLoad = 0.35;
    const double NDL_lastChannelLoad = 0.40;

    const size_t STATE_0 = 0;
    const size_t STATE_1 = 1;
    const size_t STATE_2 = 2;
    const size_t STATE_3 = 3;
    const size_t STATE_4 = 4;
    const size_t STATE_UNDEF = 5;
}

DccBeaconing_new::DccBeaconing_new() :
    BaseProtocol() {

    NDL_minPacketInterval = 0.04;
    NDL_secPacketInterval = 0.1;
    NDL_refPacketInterval = 0.3;
    NDL_thiPacketInterval = 0.5;
    NDL_maxPacketInterval = 1.0;
    NDL_timeUp = 1.00;
    NDL_timeDown = 5.00;
    DCC_measure_interval_Tm = 1.00;
    NDL_minDccSampling = 1.00;

    assert(NDL_timeUp.raw() % DCC_measure_interval_Tm.raw() == 0);
    assert(NDL_timeDown.raw() % DCC_measure_interval_Tm.raw() == 0);
    assert(NDL_minDccSampling >= DCC_measure_interval_Tm);

    new_states.insert(std::make_pair(STATE_0, State(NDL_minPacketInterval)));
    new_states.insert(std::make_pair(STATE_1, State(NDL_secPacketInterval)));
    new_states.insert(std::make_pair(STATE_2, State(NDL_refPacketInterval)));
    new_states.insert(std::make_pair(STATE_3, State(NDL_thiPacketInterval)));
    new_states.insert(std::make_pair(STATE_4, State(NDL_maxPacketInterval)));

    doNewMeasureEvt = new cMessage("doMeasure");
    doNewDccEvt = new cMessage("doDcc");
}

void DccBeaconing_new::initialize(int stage) {
    BaseProtocol::initialize(stage);

    std::string myName = getParentModule()->getFullName();

    if(stage == 0) {


        new_currentState = STATE_UNDEF;
        currentPacketInterval = 0;
        setCurrentState(STATE_0);
        using namespace std;
        cout << "start using DCC new" << endl;

        new_channelLoadInTimeUp.reset(NDL_timeUp / DCC_measure_interval_Tm);
        new_channelLoadInTimeDown.reset(NDL_timeDown / DCC_measure_interval_Tm);

        new_lastBeaconAt = -1;

        beaconIntervalIdOut.setName("beaconIntervalId");
        beaconIntervalOut.setName("beaconInterval");

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

void DccBeaconing_new::startCommunications() {
    // startup protocol
    simtime_t firstMeasureAt = simTime() + uniform(0, DCC_measure_interval_Tm);
    simtime_t firstDccAt = firstMeasureAt + uniform(0, NDL_minDccSampling - DCC_measure_interval_Tm);
    scheduleAt(firstDccAt, sendBeacon);
    //first measurement
    scheduleAt(firstMeasureAt, doNewMeasureEvt);
    scheduleAt(firstDccAt, doNewDccEvt);


}

void DccBeaconing_new::finish() {
    if (sendBeacon -> isScheduled()) cancelEvent(sendBeacon);
    if (doNewMeasureEvt->isScheduled()) cancelEvent(doNewMeasureEvt);
    if (doNewDccEvt->isScheduled()) cancelEvent(doNewDccEvt);

}

DccBeaconing_new::~DccBeaconing_new() {
    delete sendBeacon;
    delete doNewMeasureEvt;
    delete doNewDccEvt;
}

void DccBeaconing_new::handleSelfMsg(cMessage* msg) {

    BaseProtocol::handleSelfMsg(msg);

    if (msg == sendBeacon) {
        // send beacon
        sendPlatooningMessage(-1);
        //SimTime beginTime = SimTime(uniform(0.001, currentPacketInterval));
        //record actual beacon interval
        if (new_lastBeaconAt != -1){
        beaconIntervalOut.record(simTime() - new_lastBeaconAt);
        beaconIntervalIdOut.record(myId);
        }
        new_lastBeaconAt = simTime();
        // schedule next beacon
        scheduleAt(simTime() + currentPacketInterval * uniform(1.00, 1.01), sendBeacon);
        //currentPacketIntervalOut.record(currentPacketInterval);

    }

    else if (msg == doNewMeasureEvt) {
        doMeasure();
    }
    else if (msg == doNewDccEvt) {
        doDcc();
    }

}

void DccBeaconing_new::doMeasure() {

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

    new_channelLoadInTimeUp.insert(busyRatio);
    new_channelLoadInTimeDown.insert(busyRatio);


    scheduleAt(simTime() + DCC_measure_interval_Tm, doNewMeasureEvt);
}

void DccBeaconing_new::doDcc() {
    double clMinInTimeUp = new_channelLoadInTimeUp.min();
    double clMaxInTimeDown = new_channelLoadInTimeDown.max();
    //state up
    if ((new_currentState == STATE_0 ) && ( clMinInTimeUp >= NDL_firChannelLoad) && (clMinInTimeUp < NDL_minChannelLoad)) setCurrentState(STATE_1);
    else if ((new_currentState == STATE_0 ) && (clMinInTimeUp >= NDL_minChannelLoad) && (clMinInTimeUp < NDL_maxChannelLoad)) setCurrentState(STATE_2);
    else if ((new_currentState == STATE_0 ) && (clMinInTimeUp >= NDL_maxChannelLoad) && (clMinInTimeUp < NDL_lastChannelLoad)) setCurrentState(STATE_3);
    else if ((new_currentState == STATE_0 ) && (clMinInTimeUp >= NDL_lastChannelLoad)) setCurrentState(STATE_4);

    else if ((new_currentState == STATE_1 ) && (clMinInTimeUp >= NDL_minChannelLoad) && (clMinInTimeUp < NDL_maxChannelLoad)) setCurrentState(STATE_2);
    else if ((new_currentState == STATE_1 ) && (clMinInTimeUp >= NDL_maxChannelLoad) && (clMinInTimeUp < NDL_lastChannelLoad)) setCurrentState(STATE_3);
    else if ((new_currentState == STATE_1) && (clMinInTimeUp >= NDL_lastChannelLoad)) setCurrentState(STATE_4);

    else if ((new_currentState == STATE_2 ) && (clMinInTimeUp >= NDL_maxChannelLoad) && (clMinInTimeUp < NDL_lastChannelLoad)) setCurrentState(STATE_3);
    else if ((new_currentState == STATE_2) && (clMinInTimeUp >= NDL_lastChannelLoad)) setCurrentState(STATE_4);

    else if ((new_currentState == STATE_3) && (clMinInTimeUp >= NDL_lastChannelLoad)) setCurrentState(STATE_4);
    //state down
    else if ((new_currentState == STATE_4) && (clMaxInTimeDown < NDL_lastChannelLoad) && (clMaxInTimeDown >= NDL_maxChannelLoad)) setCurrentState(STATE_3);
    else if ((new_currentState == STATE_4) && (clMaxInTimeDown < NDL_maxChannelLoad) && (clMaxInTimeDown >= NDL_minChannelLoad)) setCurrentState(STATE_2);
    else if ((new_currentState == STATE_4) && (clMaxInTimeDown < NDL_minChannelLoad) && (clMaxInTimeDown >= NDL_firChannelLoad)) setCurrentState(STATE_1);
    else if ((new_currentState == STATE_4) && (clMaxInTimeDown < NDL_firChannelLoad)) setCurrentState(STATE_0);

    else if ((new_currentState == STATE_3) && (clMaxInTimeDown < NDL_maxChannelLoad) && (clMaxInTimeDown >= NDL_minChannelLoad)) setCurrentState(STATE_2);
    else if ((new_currentState == STATE_3) && (clMaxInTimeDown < NDL_minChannelLoad) && (clMaxInTimeDown >= NDL_firChannelLoad)) setCurrentState(STATE_1);
    else if ((new_currentState == STATE_3) && (clMaxInTimeDown < NDL_firChannelLoad)) setCurrentState(STATE_0);

    else if ((new_currentState == STATE_2) && (clMaxInTimeDown < NDL_minChannelLoad) && (clMaxInTimeDown >= NDL_firChannelLoad)) setCurrentState(STATE_1);
    else if ((new_currentState == STATE_2) && (clMaxInTimeDown < NDL_firChannelLoad)) setCurrentState(STATE_0);

    else if ((new_currentState == STATE_1) && (clMaxInTimeDown < NDL_firChannelLoad)) setCurrentState(STATE_0);

    scheduleAt(simTime() + NDL_minDccSampling, doNewDccEvt);
    //currentStateOut.record(currentState);
}


void DccBeaconing_new::setCurrentState(size_t state) {
    new_currentState = state;
    simtime_t stateInterval = new_states.find(new_currentState)->second.asPacketInterval;
    if (stateInterval != 0) {
        currentPacketInterval = stateInterval;
    }

    //if (communicationsStarted && sendBeacon->isScheduled())
    if (sendBeacon->isScheduled()){
        // re-schedule next beacon
        simtime_t newScheduleTime = std::max(simTime(), new_lastBeaconAt + currentPacketInterval * uniform(1.00, 1.01));
        cancelEvent(sendBeacon);
        scheduleAt(newScheduleTime, sendBeacon);

    }
}

void DccBeaconing_new::messageReceived(PlatooningBeacon *pkt, UnicastMessage *unicast) {
//Nothing to do

}

void DccBeaconing_new::channelBusyStart() {

    dccIdleTime += simTime() - dccStartIdle;
    dccStartBusy = simTime();
    idleChannel = false;

}

void DccBeaconing_new::channelIdleStart() {

    dccBusyTime += simTime() - dccStartBusy;
    dccStartIdle = simTime();
    idleChannel = true;

}
