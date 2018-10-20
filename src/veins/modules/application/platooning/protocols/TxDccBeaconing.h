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

#ifndef TXDCCBEACONING_H_
#define TXDCCBEACONING_H_

#include "veins/modules/application/platooning/protocols/BaseProtocol.h"
#include "veins/modules/mac/ieee80211p/Mac1609_4.h"

#include <algorithm>

#ifndef DBG
#define DBG EV
#endif

#define DYNAMIC_BEACONING_SNR_THRESHOLD 50.
#define DYNAMIC_BEACONING_NEIGHBOR_MAX_COUNT 50.

class TxDccBeaconing : public BaseProtocol {
    public:
//        class NeighborEntry {
//            public:
//                int id;
//                double dist;
//                simtime_t createdAt;
//                simtime_t lastUpdate;
//        };
//        class State{
//            typedef AS_PARAMETER<double>       NDL_TYPE_TXPOWER_DBM;
//            AS_PARAMETER<double> asTxPower_dBm;
//
//            State(NDL_TYPE_TXPOWER_DBM asTxPower_dBm) :
//                asTxPower_dBm(asTxPower_dBm){
//            }
//        }

        TxDccBeaconing();
        virtual void initialize(int stage);
        virtual void finish();
        virtual ~TxDccBeaconing();

    protected:
//        typedef std::map<int, NeighborEntry> Neighbors;
//        typedef std::set<int> TotalNeighbors;

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
        States states;
        cMessage* doDccEvt;
        cMessage* doMeasureEvt;

//        RingBuffer<double> channelLoadInTimeUp;
//        RingBuffer<double> channelLoadInTimeDown;
        RingBuffer<double> channelLoad;

        void handleSelfMsg(cMessage* msg);
//        virtual void startCommunications();
        void messageReceived(PlatooningBeacon *pkt, UnicastMessage *unicast);
        //override superclass methods to get channel busy info
        virtual void channelBusyStart();
        virtual void channelIdleStart();

    private:

        SimTime dccIdleTime, dccBusyTime, dccStartIdle, dccStartBusy;
        bool idleChannel;
        int TxPower, level2TxPower,level3TxPower, level4TxPower,level5TxPower;

    protected:

        void doMeasure();
        void doDcc();
//        void setCurrentState(size_t state);

        simtime_t lastBeaconAt;

        std::pair<simtime_t, simtime_t> lastBusyTime;

        cOutVector beaconIntervalIdOut;
        cOutVector beaconIntervalOut;

        Mac1609_4 *mac;

};

#endif /* DRDCCBEACONING_H_ */
