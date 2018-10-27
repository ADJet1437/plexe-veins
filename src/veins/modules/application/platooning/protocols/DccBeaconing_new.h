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

//#ifndef DCCBEACONING_NEW_H_
//#define DCCBEACONING_NEW_H_

#include "veins/modules/application/platooning/protocols/BaseProtocol.h"

#include <algorithm>

#ifndef DBG
#define DBG EV
#endif

#define DYNAMIC_BEACONING_SNR_THRESHOLD 50.
#define DYNAMIC_BEACONING_NEIGHBOR_MAX_COUNT 50.

class DccBeaconing_new : public BaseProtocol {
    public:
        class NeighborEntry {
            public:
                int id;
                double dist;
                simtime_t createdAt;
                simtime_t lastUpdate;
        };

        DccBeaconing_new();
        virtual void initialize(int stage);
        virtual void finish();
        virtual ~DccBeaconing_new();

    protected:
        typedef std::map<int, NeighborEntry> Neighbors;
        typedef std::set<int> TotalNeighbors;

        class State {
            public:
                simtime_t asPacketInterval;
                State(simtime_t asPacketInterval) :
                    asPacketInterval(asPacketInterval) {
                }
        };

        template<typename T> class RingBuffer {
            public:
                size_t maxSize;
                std::list<T> entries;
                typename std::list<T>::iterator nextInsert;

                RingBuffer(size_t maxSize = 0) :
                    maxSize(0) {
                    reset(maxSize);
                }

                void reset(size_t maxSize) {
                    entries.clear();
                    this->maxSize = maxSize;
                    nextInsert = entries.end();
                }

                void insert(const T& o) {
                    assert(maxSize > 0);
                    if (nextInsert == entries.end()) {
                        if (entries.size() < maxSize) {
                            // special case: RingBuffer not yet full
                            entries.push_back(o);
                            return;
                        }
                        else {
                            nextInsert = entries.begin();
                        }
                    }
                    *nextInsert = o;
                    nextInsert++;
                }

                T min() {
                    return *std::min_element(entries.begin(), entries.end());
                }

                T max() {
                    return *std::max_element(entries.begin(), entries.end());
                }
        };

        typedef std::map<size_t, State> States;
        States new_states;
        size_t new_currentState;
        simtime_t currentPacketInterval;
        cMessage* doNewDccEvt;
        cMessage* doNewMeasureEvt;
//        cMessage* sendBeacon;

        RingBuffer<double> new_channelLoadInTimeUp;
        RingBuffer<double> new_channelLoadInTimeDown;

        void handleSelfMsg(cMessage* msg);
        virtual void startCommunications();
        virtual void messageReceived(PlatooningBeacon *pkt, UnicastMessage *unicast);

        //override superclass methods to get channel busy info
        virtual void channelBusyStart();
        virtual void channelIdleStart();

    private:

        SimTime dccIdleTime, dccBusyTime, dccStartIdle, dccStartBusy;
        bool idleChannel;

    protected:

        void doMeasure();
        void doDcc();
        void setCurrentState(size_t state);

        simtime_t new_lastBeaconAt;
        SimTime a, b, InterReceptionTime;

        Neighbors neighbors;
        simtime_t neighborExpiry;

        std::pair<simtime_t, simtime_t> lastBusyTime;

        cOutVector beaconIntervalIdOut;
        cOutVector beaconIntervalOut;

};

