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








//RWCommandQueue.cpp
//
//Class file for command queue object
//

#include "RWCommandQueue.h"
#include "MemoryController.h"
#include <assert.h>

using namespace DRAMSim;

RWCommandQueue::RWCommandQueue(vector< vector<BankState> > &states, ostream &dramsim_log_) :
		dramsim_log(dramsim_log_),
		bankStates(states),
		refreshRank(0),
		refreshWaiting(false),
		sendAct(true),
		preIsWrite(false),
		curIsWrite(false),
		nextRankPRE(0),
		nextBankPRE(0)
{
	//set here to avoid compile errors
	currentClockCycle = 0;

	//vector of counters used to ensure rows don't stay open too long
	rowAccessCounters = vector< vector<unsigned> >(NUM_RANKS, vector<unsigned>(NUM_BANKS,0));
	bankAccessFlag = vector< vector<bool> >(NUM_RANKS, vector<bool>(NUM_BANKS,false));
	for (size_t i = 0; i < NUM_THREAD; i++)
	{
		BusPacket3D bp3d = BusPacket3D();
		for (size_t j = 0; j < NUM_RANKS; j++)
		{
			BusPacket2D bp2d = BusPacket2D();
			for (size_t k = 0; k < NUM_BANKS; k++)
			{
				BusPacket1D bp1d = BusPacket1D();
				bp2d.push_back(bp1d);
			}
			bp3d.push_back(bp2d);
		}
		reqsMarkedInBankPerThread.push_back(bp3d);
	}
	reqsMarkedPerThread = vector<unsigned>(NUM_THREAD, 0);
	threadPriority = vector<unsigned>(NUM_THREAD, 0);
	maxRulPerthread = vector<unsigned>(NUM_THREAD, 0);
	totalMarkedRequests = 0;	
	//FOUR-bank activation window
	//	this will count the number of activations within a given window
	//	(decrementing counter)
	//
	//countdown vector will have decrementing counters starting at tFAW
	//  when the 0th element reaches 0, remove it
	tFAWCountdown.reserve(NUM_RANKS);
	for (size_t i=0;i<NUM_RANKS;i++)
	{
		//init the empty vectors here so we don't seg fault later
		tFAWCountdown.push_back(vector<unsigned>());
	}
}
RWCommandQueue::~RWCommandQueue()
{
	//ERROR("COMMAND QUEUE destructor");
	
	for (size_t i=0; i< readQueues.size(); i++)
		delete readQueues[i];
	for (size_t i=0; i< writeQueues.size(); i++)
		delete writeQueues[i];
}
//Adds a command to appropriate queue
void RWCommandQueue::enqueue(bool isWrite, BusPacket *newBusPacket)
{
	if (isWrite)
	{
		writeQueues.push_back(newBusPacket);
		if (writeQueues.size()>WRITE_CMD_QUEUE_DEPTH)
		{
			ERROR("== Error - Enqueued more than allowed in write command queue");
			ERROR("						Need to call .hasRoomFor(int numberToEnqueue, unsigned rank, unsigned bank) first");
			exit(0);
		}
	}
	else
	{
		readQueues.push_back(newBusPacket);
		if (readQueues.size()>READ_CMD_QUEUE_DEPTH)
		{
			ERROR("== Error - Enqueued more than allowed in read command queue");
			ERROR("						Need to call .hasRoomFor(int numberToEnqueue, unsigned rank, unsigned bank) first");
			exit(0);
		}
	}
}

vector<BusPacket *> & RWCommandQueue::getCommandQueue()
{
	if (curIsWrite)
		return writeQueues;
	return readQueues;
}
//par-bs scheduling
bool RWCommandQueue::scheduleParbs(BusPacket **busPacket)
{
	if (totalMarkedRequests == 0)
	{
		//新的batch开始	
		//标记请求
		PRINT("新的batch开始");
		while (!readQueues.empty())	
		{
			BusPacket* bp = readQueues[0];	
			if (reqsMarkedInBankPerThread[bp->threadId][bp->rank][bp->bank].size() < MARKING_CAP)
			{
				reqsMarkedInBankPerThread[bp->threadId][bp->rank][bp->bank].push_back(bp);
				reqsMarkedPerThread[bp->threadId]++;
				totalMarkedRequests++;
				readQueues.erase(readQueues.begin());
			}
		}
		//统计Max Rule
		for (size_t i = 0;  i < NUM_THREAD; i++)
		{
			maxRulPerthread[i] = 0;
			for (size_t j = 0; j < NUM_RANKS; j++)
				for (size_t k = 0; k < NUM_BANKS; k++)
					if (reqsMarkedInBankPerThread[i][j][k].size() > maxRulPerthread[i])
						maxRulPerthread[i] = reqsMarkedInBankPerThread[i][j][k].size();
		}
		//获取排名
		threadPriority.clear();
		for (size_t i = 0; i < NUM_THREAD; i++)
		{
			unsigned min = maxRulPerthread[0];	
			size_t p = 0;
			for (size_t j = 1; j < NUM_THREAD; j++)
				if (maxRulPerthread[j] < min)
				{
					min = maxRulPerthread[j];	
					p = j;
				}
			for (size_t j = 0; j < NUM_THREAD; j++)	
				if (j != p && maxRulPerthread[j] == min && reqsMarkedPerThread[j] < reqsMarkedPerThread[p])
					p = j;
			threadPriority.push_back(p);
			maxRulPerthread[p] = 0xffff;
		}
	}
	PRINT("print marked queue");
	for (size_t i = 0; i < NUM_THREAD; i++)
	{
		PRINT(reqsMarkedPerThread[i]);
		for (size_t j = 0; j < NUM_RANKS; j++)
			for (size_t k = 0; k < NUM_BANKS; k++)
				for (size_t ii = 0; ii < reqsMarkedInBankPerThread[i][j][k].size(); ii++)
					reqsMarkedInBankPerThread[i][j][k][ii]->print();
		PRINT("\n");
	}
	bool sendingREForPRE = false;
	if (refreshWaiting)
	{
		bool sendREF = true;
		//make sure all banks idle and timing met for a REF
		for (size_t b=0;b<NUM_BANKS;b++)
		{
			//if a bank is active we can't send a REF yet
			if (bankStates[refreshRank][b].currentBankState == RowActive)
			{
				sendREF = false;
				bool closeRow = true;
				//search for commands going to an open row
				for (size_t t = 0; t < NUM_THREAD; t++)
				{
					vector <BusPacket *> &refreshQueue = reqsMarkedInBankPerThread[threadPriority[t]][refreshRank][b];
					if (refreshQueue.empty())
						continue;
					for (size_t j=0;j<refreshQueue.size();j++)
					{
						BusPacket *packet = refreshQueue[j];
						//if a command in the queue is going to the same row . . .
						if (refreshRank == packet->rank && bankStates[refreshRank][b].openRowAddress == packet->row &&
								b == packet->bank)
						{
							// . . . and is not an activate . . .
							if (packet->busPacketType != ACTIVATE)
							{
								closeRow = false;
								// . . . and can be issued . . .
								if (isIssuable(packet))
								{
									//send it out
									*busPacket = packet;
									refreshQueue.erase(refreshQueue.begin()+j);
									reqsMarkedPerThread[threadPriority[t]]--;
									totalMarkedRequests--;
									sendingREForPRE = true;
								}
								break;
							}
							else //command is an activate，优先刷新
							{
								//if we've encountered another act, no other command will be of interest
								break;
							}
						}
					}
					break;
				}
				//if the bank is open and we are allowed to close it, then send a PRE
				if (closeRow && currentClockCycle >= bankStates[refreshRank][b].nextPrecharge)
				{
					rowAccessCounters[refreshRank][b]=0;
					*busPacket = new BusPacket(PRECHARGE, 0, 0, 0, refreshRank, b, 0, dramsim_log);
					sendingREForPRE = true;
				}
				break;
			}
			//	NOTE: the next ACT and next REF can be issued at the same
			//				point in the future, so just use nextActivate field instead of
			//				creating a nextRefresh field
			else if (bankStates[refreshRank][b].nextActivate > currentClockCycle) //and this bank doesn't have an open row
			{
				sendREF = false;
				break;
			}
		}

		//if there are no open banks and timing has been met, send out the refresh
		//	reset flags and rank pointer
		if (sendREF && bankStates[refreshRank][0].currentBankState != PowerDown)
		{
			*busPacket = new BusPacket(REFRESH, 0, 0, 0, refreshRank, 0, 0, dramsim_log);
			refreshRank = -1;
			refreshWaiting = false;
			sendingREForPRE = true;
		}
	}
	if (sendingREForPRE)
		return true;
	for (size_t i = 0; i < NUM_RANKS; i++)
		for (size_t j = 0; j < NUM_BANKS; j++)
			bankAccessFlag[i][j] = false;
	//根据排名发送包
	bool foundIssuable = false;
	for (size_t i = 0; i < NUM_THREAD; i++)	
	{
		unsigned threadId = threadPriority[i];	
		//这个线程还有包
		if (reqsMarkedPerThread[threadId] > 0)
		{
			for (size_t j = 0; j < NUM_RANKS; j++)		
			{
				if (j == refreshRank && refreshWaiting)
					continue;
				for(size_t k = 0; k < NUM_BANKS; k++)	
				{
					if (bankAccessFlag[j][k])  //前面已经有请求了
						continue;
					vector<BusPacket *> &queue = reqsMarkedInBankPerThread[threadId][j][k];
					//make sure there is something there first
					if (!queue.empty())
					{
						PRINT("queue not empty! " << bankStates[j][k].nextRead);
						queue[0]->print();
						bankAccessFlag[j][k] = true;
						for (size_t ii =0; ii < queue.size(); ii++)
						{
							BusPacket *packet = queue[ii];
							if (isIssuable(packet))
							{
								//check for dependencies
								bool dependencyFound = false;
								for (size_t jj = 0; jj < ii; jj++)
								{
									BusPacket *prevPacket = queue[jj];
									if (prevPacket->busPacketType != ACTIVATE && 
											prevPacket->row == packet->row)
									{
										dependencyFound = true;
										break;
									}
								}
								if (dependencyFound) continue;
								*busPacket = packet;
								//if the bus packet before is an activate, that is the act that was
								//	paired with the column access we are removing, so we have to remove
								//	that activate as well (check i>0 because if i==0 then theres nothing before it)
								if (ii > 0 && queue[ii-1]->busPacketType == ACTIVATE)
								{
									rowAccessCounters[(*busPacket)->rank][(*busPacket)->bank]++;
									reqsMarkedPerThread[threadId] -= 2;
									totalMarkedRequests -= 2;
									// i is being returned, but i-1 is being thrown away, so must delete it here 
									delete (queue[ii-1]);

									// remove both i-1 (the activate) and i (we've saved the pointer in *busPacket)
									queue.erase(queue.begin()+ii-1,queue.begin()+ii+1);
								}
								else // there's no activate before this packet
								{
									//or just remove the one bus packet
									reqsMarkedPerThread[threadId]--;
									totalMarkedRequests--;
									queue.erase(queue.begin()+ii);
								}

								foundIssuable = true;
								break;
							}
						}
					}
					if (foundIssuable)
						break;
				}
				if (foundIssuable)
					break;
			}
			if (foundIssuable)
				break;
		}
		//当前线程没有请求，处理下一个线程请求
	}
	if (foundIssuable)
	{
		if ((*busPacket)->busPacketType==ACTIVATE)
		{
			tFAWCountdown[(*busPacket)->rank].push_back(tFAW);
		}
		return true;
	}
	/*
	//给那些没有等待的marked请求的bank发送未标记缓存中的请求，根据fr-fcfs	
	for (size_t i = 0; i < NUM_RANKS; i++)	
	{
		std::cout << "给那些没有等待的marked请求的bank发送未标记缓存中的请求，根据fr-fcfs" << std::endl;
		if (i == refreshRank && refreshWaiting)
			continue;
		for (size_t j = 0; j < NUM_BANKS; j++)	
		{
			if (bankAccessFlag[i][j] == false)
			{
				vector<BusPacket *>& queue= getCommandQueue();			
				for (size_t k = 0; k < queue.size(); k++)
				{
					BusPacket *packet = queue[k];
					if (!(packet->rank == i && packet->bank == j))
						continue;
					if (isIssuable(packet))
					{
						//check for dependencies
						bool dependencyFound = false;
						for (size_t ii = 0; ii < k; ii++)
						{
							BusPacket *prevPacket = queue[ii];
							if (prevPacket->busPacketType != ACTIVATE && 
									prevPacket->rank == packet->rank &&
									prevPacket->bank == packet->bank &&
									prevPacket->row == packet->row)
							{
								dependencyFound = true;
								break;
							}
						}
						if (dependencyFound) continue;

						*busPacket = packet;

						//if the bus packet before is an activate, that is the act that was
						//	paired with the column access we are removing, so we have to remove
						//	that activate as well (check i>0 because if i==0 then theres nothing before it)
						if (k >0 && queue[k-1]->busPacketType == ACTIVATE)
						{
							rowAccessCounters[(*busPacket)->rank][(*busPacket)->bank]++;
							// i is being returned, but i-1 is being thrown away, so must delete it here 
							delete (queue[k-1]);

							// remove both i-1 (the activate) and i (we've saved the pointer in *busPacket)
							queue.erase(queue.begin()+k-1,queue.begin()+k+1);
						}
						else // there's no activate before this packet
						{
							//or just remove the one bus packet
							queue.erase(queue.begin()+k);
						}

						foundIssuable = true;
						break;
					}
				}
			}
			if (foundIssuable)
				break;
		}
		if (foundIssuable)
			break;
	}
	if (foundIssuable)
	{
		if ((*busPacket)->busPacketType==ACTIVATE)
		{
			tFAWCountdown[(*busPacket)->rank].push_back(tFAW);
		}
		return true;
	}*/
	//没有可发送的包，看看能否发送precharge
	for (size_t i = 0; i < NUM_RANKS; i++)
		for (size_t j = 0; j < NUM_BANKS; j++)
			bankAccessFlag[i][j] = false;
	bool sendingPRE = false;
	for (size_t i = 0; i < NUM_THREAD; i++)	
	{
		unsigned threadId = threadPriority[i];	
		//这个线程还有包
		if (reqsMarkedPerThread[threadId] > 0)
		{
			for (size_t j = 0; j < NUM_RANKS; j++)		
			{
				for(size_t k = 0; k < NUM_BANKS; k++)	
				{
					vector<BusPacket *> &queue = reqsMarkedInBankPerThread[threadId][j][k];
					if (queue.empty())
						continue;
					if (bankAccessFlag[j][k])  //前面已经访问了,不能关闭
						continue;
					bankAccessFlag[j][k] = true;
					bool found = false;
					//if there is something going to that bank and row, then we don't want to send a PRE
					if (bankStates[j][k].currentBankState == RowActive) 
					{
						for (size_t ii = 0; ii < queue.size(); ii++)
						{
							if (queue[ii]->row == bankStates[j][k].openRowAddress)
							{
								found = true;
								break;
							}
						}
						//if nothing found going to that bank and row or too many accesses have happend, close it
						if (!found || rowAccessCounters[j][k]==TOTAL_ROW_ACCESSES)
						{
							if (currentClockCycle >= bankStates[j][k].nextPrecharge)
							{
								sendingPRE = true;
								rowAccessCounters[j][k] = 0;
								*busPacket = new BusPacket(PRECHARGE, 0, 0, 0, j, k, 0, dramsim_log);
								break;
							}
						}
					}
				}
				if (sendingPRE)
					break;
			}
			if (sendingPRE)
				break;
		}
	}
	/*
	if (sendingPRE)
		return true;
	for (size_t i = 0; i < NUM_RANKS; i++)	
	{
		for (size_t j = 0; j < NUM_BANKS; j++)	
		{
			if (bankAccessFlag[i][j] == false)
			{
				if (bankStates[i][j].currentBankState == RowActive) 
				{
					bool found = false;
					vector<BusPacket*>& queue = getCommandQueue();
					for (size_t ii = 0; ii < queue.size(); ii++)
					{
						if (queue[ii]->rank == i && queue[ii]->bank == j && 
								queue[ii]->row == bankStates[i][j].openRowAddress)
						{
							found = true;
							break;
						}
					}
					//if nothing found going to that bank and row or too many accesses have happend, close it
					if (!found || rowAccessCounters[i][j]==TOTAL_ROW_ACCESSES)
					{
						if (currentClockCycle >= bankStates[i][j].nextPrecharge)
						{
							sendingPRE = true;
							rowAccessCounters[i][j] = 0;
							*busPacket = new BusPacket(PRECHARGE, 0, 0, 0, i, j, 0, dramsim_log);
							break;
						}
					}
				}
			}
		}
		if (sendingPRE)
			break;
	}*/
	if (!sendingPRE)
		return false;
	return true;
}

//Removes the next item from the command queue based on the system's
//command scheduling policy
bool RWCommandQueue::pop(BusPacket **busPacket)
{
	PRINT("pop");
	//this can be done here because pop() is called every clock cycle by the parent MemoryController
	//	figures out the sliding window requirement for tFAW
	//
	//deal with tFAW book-keeping
	//	each rank has it's own counter since the restriction is on a device level
	for (size_t i=0;i<NUM_RANKS;i++)
	{
		//decrement all the counters we have going
		for (size_t j=0;j<tFAWCountdown[i].size();j++)
		{
			tFAWCountdown[i][j]--;
		}

		//the head will always be the smallest counter, so check if it has reached 0
		if (tFAWCountdown[i].size()>0 && tFAWCountdown[i][0]==0)
		{
			tFAWCountdown[i].erase(tFAWCountdown[i].begin());
		}
	}

	/* Now we need to find a packet to issue. When the code picks a packet, it will set
		 *busPacket = [some eligible packet]
		 
		 First the code looks if any refreshes need to go
		 Then it looks for data packets
		 Otherwise, it starts looking for rows to close (in open page)
	*/
	if ((readQueues.empty() && totalMarkedRequests == 0) || writeQueues.size() >= WRITE_HIGHT_THEROLD || 
			(preIsWrite && writeQueues.size() >= WRITE_LOW_THEROLD))
		preIsWrite = curIsWrite = true;
	else
		preIsWrite = curIsWrite = false;
	if (rowBufferPolicy==ClosePage)
	{
	}
	else if (rowBufferPolicy==OpenPage)
	{
		if (!curIsWrite && schedulingPolicy == PARBS)
			return scheduleParbs(busPacket);
		bool sendingREForPRE = false;
		if (refreshWaiting)
		{
			bool sendREF = true;
			//make sure all banks idle and timing met for a REF
			for (size_t b=0;b<NUM_BANKS;b++)
			{
				//if a bank is active we can't send a REF yet
				if (bankStates[refreshRank][b].currentBankState == RowActive)
				{
					sendREF = false;
					bool closeRow = true;
					//search for commands going to an open row
					vector <BusPacket *> &refreshQueue = getCommandQueue();

					for (size_t j=0;j<refreshQueue.size();j++)
					{
						BusPacket *packet = refreshQueue[j];
						//if a command in the queue is going to the same row . . .
						if (refreshRank == packet->rank && bankStates[refreshRank][b].openRowAddress == packet->row &&
								b == packet->bank)
						{
							// . . . and is not an activate . . .
							if (packet->busPacketType != ACTIVATE)
							{
								closeRow = false;
								// . . . and can be issued . . .
								if (isIssuable(packet))
								{
									//send it out
									*busPacket = packet;
									refreshQueue.erase(refreshQueue.begin()+j);
									sendingREForPRE = true;
								}
								break;
							}
							else //command is an activate，优先刷新
							{
								//if we've encountered another act, no other command will be of interest
								break;
							}
						}
					}

					//if the bank is open and we are allowed to close it, then send a PRE
					if (closeRow && currentClockCycle >= bankStates[refreshRank][b].nextPrecharge)
					{
						rowAccessCounters[refreshRank][b]=0;
						*busPacket = new BusPacket(PRECHARGE, 0, 0, 0, refreshRank, b, 0, dramsim_log);
						sendingREForPRE = true;
					}
					break;
				}
				//	NOTE: the next ACT and next REF can be issued at the same
				//				point in the future, so just use nextActivate field instead of
				//				creating a nextRefresh field
				else if (bankStates[refreshRank][b].nextActivate > currentClockCycle) //and this bank doesn't have an open row
				{
					sendREF = false;
					break;
				}
			}

			//if there are no open banks and timing has been met, send out the refresh
			//	reset flags and rank pointer
			if (sendREF && bankStates[refreshRank][0].currentBankState != PowerDown)
			{
				*busPacket = new BusPacket(REFRESH, 0, 0, 0, refreshRank, 0, 0, dramsim_log);
				refreshRank = -1;
				refreshWaiting = false;
				sendingREForPRE = true;
			}
		}
		//看看能否找到可发送的包
		if (!sendingREForPRE)
		{
			bool foundIssuable = false;
			vector<BusPacket *> &queue = getCommandQueue();
			//make sure there is something there first
			if (!queue.empty())
			{
				//search from the beginning to find first issuable bus packet
				for (size_t i=0;i<queue.size();i++)
				{
					BusPacket *packet = queue[i];
					if (packet->rank == refreshRank && refreshWaiting)
						continue;
					if (isIssuable(packet))
					{
						//check for dependencies
						bool dependencyFound = false;
						for (size_t j=0;j<i;j++)
						{
							BusPacket *prevPacket = queue[j];
							if (prevPacket->busPacketType != ACTIVATE && 
									prevPacket->rank == packet->rank &&
									prevPacket->bank == packet->bank &&
									prevPacket->row == packet->row)
							{
								dependencyFound = true;
								break;
							}
						}
						if (dependencyFound) continue;

						*busPacket = packet;

						//if the bus packet before is an activate, that is the act that was
						//	paired with the column access we are removing, so we have to remove
						//	that activate as well (check i>0 because if i==0 then theres nothing before it)
						if (i>0 && queue[i-1]->busPacketType == ACTIVATE)
						{
							rowAccessCounters[(*busPacket)->rank][(*busPacket)->bank]++;
							// i is being returned, but i-1 is being thrown away, so must delete it here 
							delete (queue[i-1]);

							// remove both i-1 (the activate) and i (we've saved the pointer in *busPacket)
							queue.erase(queue.begin()+i-1,queue.begin()+i+1);
						}
						else // there's no activate before this packet
						{
							//or just remove the one bus packet
							queue.erase(queue.begin()+i);
						}

						foundIssuable = true;
						break;
					}
				}
			}
			//看看能否关闭某行
			//if nothing was issuable, see if we can issue a PRE to an open bank
			//	that has no other commands waiting
			if (!foundIssuable)
			{
				//search for banks to close
				bool sendingPRE = false;
				vector <BusPacket *> &queue = getCommandQueue();
				unsigned startingRank = nextRankPRE;
				unsigned startingBank = nextBankPRE;
				do
				{
					bool found = false;
					//if there is something going to that bank and row, then we don't want to send a PRE
					if (bankStates[nextRankPRE][nextBankPRE].currentBankState == RowActive) 
					{
						for (size_t i = 0; i < queue.size(); i++)
						{
							if (queue[i]->rank == nextRankPRE && queue[i]->bank == nextBankPRE && 
									queue[i]->row == bankStates[nextRankPRE][nextBankPRE].openRowAddress)
							found = true;
							break;
						}
						//if nothing found going to that bank and row or too many accesses have happend, close it
						if (!found || rowAccessCounters[nextRankPRE][nextBankPRE]==TOTAL_ROW_ACCESSES)
						{
							if (currentClockCycle >= bankStates[nextRankPRE][nextBankPRE].nextPrecharge)
							{
								sendingPRE = true;
								rowAccessCounters[nextRankPRE][nextBankPRE] = 0;
								*busPacket = new BusPacket(PRECHARGE, 0, 0, 0, nextRankPRE, nextBankPRE, 0, dramsim_log);
								break;
							}
						}
					}
					nextRankAndBank(nextRankPRE, nextBankPRE);
				}
				while (!(startingRank == nextRankPRE && startingBank == nextBankPRE));
				if (!sendingPRE)
					return false;
			}
		}
		
	}

	//sendAct is flag used for posted-cas
	//  posted-cas is enabled when AL>0
	//  when sendAct is true, when don't want to increment our indexes
	//  so we send the column access that is paid with this act
	if (AL>0 && sendAct)
	{
		sendAct = false;
	}
	else
	{
		sendAct = true;
		//nextRankAndBank(nextRank, nextBank);
	}

	//if its an activate, add a tfaw counter
	if ((*busPacket)->busPacketType==ACTIVATE)
	{
		tFAWCountdown[(*busPacket)->rank].push_back(tFAW);
	}

	return true;
}

//check if a rank/bank queue has room for a certain number of bus packets
bool RWCommandQueue::hasRoomFor(bool isWrite, unsigned numberToEnqueue, unsigned rank, unsigned bank)
{
	if (isWrite)
		return (WRITE_CMD_QUEUE_DEPTH - writeQueues.size() >= numberToEnqueue);
	return (READ_CMD_QUEUE_DEPTH - readQueues.size() >= numberToEnqueue);
}

//prints the contents of the command queue
void RWCommandQueue::print()
{
	
	PRINT(endl << "== Printing Read Queue" );
	PRINT(" =   size : " << readQueues.size() );
	for (size_t i=0;i<readQueues.size();i++)
	{
		PRINT(" ");
		readQueues[i]->print();
	}
	PRINT(endl << "== Printing Write Queue" );
	PRINT(" =   size : " << writeQueues.size() );
	for (size_t i=0;i<writeQueues.size();i++)
	{
		PRINT(" ");
		writeQueues[i]->print();
	}
}


//checks if busPacket is allowed to be issued
bool RWCommandQueue::isIssuable(BusPacket *busPacket)
{
	switch (busPacket->busPacketType)
	{
	case REFRESH:

		break;
	case ACTIVATE:
		if ((bankStates[busPacket->rank][busPacket->bank].currentBankState == Idle ||
		        bankStates[busPacket->rank][busPacket->bank].currentBankState == Refreshing) &&
		        currentClockCycle >= bankStates[busPacket->rank][busPacket->bank].nextActivate &&
		        tFAWCountdown[busPacket->rank].size() < 4)
		{
			return true;
		}
		else
		{
			return false;
		}
		break;
	case WRITE:
	case WRITE_P:
		if (bankStates[busPacket->rank][busPacket->bank].currentBankState == RowActive &&
		        currentClockCycle >= bankStates[busPacket->rank][busPacket->bank].nextWrite &&
		        busPacket->row == bankStates[busPacket->rank][busPacket->bank].openRowAddress &&
		        rowAccessCounters[busPacket->rank][busPacket->bank] < TOTAL_ROW_ACCESSES)
		{
			return true;
		}
		else
		{
			return false;
		}
		break;
	case READ_P:
	case READ:
		if (bankStates[busPacket->rank][busPacket->bank].currentBankState == RowActive &&
		        currentClockCycle >= bankStates[busPacket->rank][busPacket->bank].nextRead &&
		        busPacket->row == bankStates[busPacket->rank][busPacket->bank].openRowAddress &&
		        rowAccessCounters[busPacket->rank][busPacket->bank] < TOTAL_ROW_ACCESSES)
		{
			return true;
		}
		else
		{
			return false;
		}
		break;
	case PRECHARGE:
		if (bankStates[busPacket->rank][busPacket->bank].currentBankState == RowActive &&
		        currentClockCycle >= bankStates[busPacket->rank][busPacket->bank].nextPrecharge)
		{
			return true;
		}
		else
		{
			return false;
		}
		break;
	default:
		ERROR("== Error - Trying to issue a crazy bus packet type : ");
		busPacket->print();
		exit(0);
	}
	return false;
}

//tells the command queue that a particular rank is in need of a refresh
void RWCommandQueue::needRefresh(unsigned rank)
{
	refreshWaiting = true;
	refreshRank = rank;
}

void RWCommandQueue::update()
{
	//do nothing since pop() is effectively update(),
	//needed for SimulatorObject
	//TODO: make RWCommandQueue not a SimulatorObject
}

bool RWCommandQueue::isEmpty(unsigned rank)
{
	for(size_t i = 0; i < readQueues.size(); i++)
		if (readQueues[i]->rank == rank)
			return false;
	for(size_t i = 0; i < writeQueues.size(); i++)
		if (writeQueues[i]->rank == rank)
			return false;
	return true;
}

void RWCommandQueue::nextRankAndBank(unsigned &rank, unsigned &bank)
{
	bank++;
	if (bank == NUM_BANKS)
	{
		bank = 0;
		rank++;
		if (rank == NUM_RANKS)
			rank = 0;
	}
}
