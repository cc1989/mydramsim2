/*********************************************************************************
*  Copyright (c) 2010-2011, Elliott Cooper-Balis
*                             Paul Rosenfeld
*                             Bruce Jacob
*                             University of Maryland 
*                             dramninjas [at] gmail [dot] com
*  All rights reserved.
*  
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions are met:
*  
*     * Redistributions of source code must retain the above copyright notice,
*        this list of conditions and the following disclaimer.
*  
*     * Redistributions in binary form must reproduce the above copyright notice,
*        this list of conditions and the following disclaimer in the documentation
*        and/or other materials provided with the distribution.
*  
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
*  ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
*  WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
*  DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
*  FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
*  DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
*  SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
*  OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
*  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*********************************************************************************/






#ifndef RWCMDQUEUE_H
#define RWCMDQUEUE_H

//CommandQueue.h
//
//Header
//

#include "BusPacket.h"
#include "BankState.h"
#include "Transaction.h"
#include "SystemConfiguration.h"
#include "SimulatorObject.h"

using namespace std;

namespace DRAMSim
{
class RWCommandQueue : public SimulatorObject
{
	RWCommandQueue();
	ostream &dramsim_log;
public:
	//typedefs
	typedef vector<BusPacket *> BusPacket1D;
	typedef vector<BusPacket1D> BusPacket2D;
	typedef vector<BusPacket2D> BusPacket3D;
	typedef vector<BusPacket3D> BusPacket4D;

	//functions
	RWCommandQueue(vector< vector<BankState> > &states, ostream &dramsim_log);
	virtual ~RWCommandQueue(); 

	void enqueue(bool isWrite, BusPacket *newBusPacket);
	bool pop(BusPacket **busPacket);
	bool hasRoomFor(bool isWrite, unsigned numberToEnqueue, unsigned rank, unsigned bank);
	vector<BusPacket *> &getCommandQueue();
	bool isIssuable(BusPacket *busPacket);
	bool isEmpty(unsigned rank);
	void needRefresh(unsigned rank);
	void print();
	void update(); //SimulatorObject requirement
	bool scheduleParbs(BusPacket **busPacket);

	
	BusPacket1D readQueues; // 1D array of BusPacket pointers
	BusPacket1D writeQueues; // 1D array of BusPacket pointers
	vector< vector<BankState> > &bankStates;
private:
	void nextRankAndBank(unsigned &rank, unsigned &bank);
	//fields
	unsigned refreshRank;
	bool refreshWaiting;

	vector< vector<unsigned> > tFAWCountdown;
	vector< vector<unsigned> > rowAccessCounters;

	bool sendAct;
	bool preIsWrite;
	bool curIsWrite;
	unsigned nextRankPRE;
	unsigned nextBankPRE;
	BusPacket4D reqsMarkedInBankPerThread;
	vector< unsigned > reqsMarkedPerThread;
	vector< unsigned > threadPriority;
	vector< unsigned > maxRulPerthread;
	vector< vector<bool> > bankAccessFlag;
	unsigned totalMarkedRequests;
};
}

#endif

