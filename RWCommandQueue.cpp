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
#include <assert.h>
#include "MemoryController.h"

using namespace DRAMSim;

RWCommandQueue::RWCommandQueue(MemoryController* parent, vector< vector<BankState> > &states, ostream &dramsim_log_) :
		dramsim_log(dramsim_log_),
		bankStates(states),
		nextBank(0),
		nextRank(0),
		nextBankPRE(0),
		nextRankPRE(0),
		refreshRank(0),
		refreshWaiting(false),
		sendAct(true),
		curIsWrite(false),
		needToChange(false),
		totalReadRequests(0)
{
	parentMC = parent;
	//set here to avoid compile errors
	currentClockCycle = 0;

	//vector of counters used to ensure rows don't stay open too long
	rowAccessCounters = vector< vector<unsigned> >(NUM_RANKS, vector<unsigned>(NUM_BANKS,0));

	//create queue based on the structure we want
	BusPacket1D actualQueue;
	BusPacket2D perBankQueue = BusPacket2D();
	queues = BusPacket3D();
	for (size_t rank=0; rank<NUM_RANKS; rank++)
	{
		//this loop will run only once for per-rank and NUM_BANKS times for per-rank-per-bank
		for (size_t bank=0; bank<NUM_BANKS; bank++)
		{
			actualQueue	= BusPacket1D();
			perBankQueue.push_back(actualQueue);
		}
		queues.push_back(perBankQueue);
	}
	//par-bs
	reqsMarkedPerThread = vector<unsigned>(NUM_THREAD, 0);
	threadPriority = vector<unsigned>(NUM_THREAD, 0);
	maxRulePerthread = vector<unsigned>(NUM_THREAD, 0);
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
	for (size_t r=0; r< NUM_RANKS; r++)
	{
		for (size_t b=0; b<NUM_BANKS; b++) 
		{
			for (size_t i=0; i<queues[r][b].size(); i++)
			{
				delete(queues[r][b][i]);
			}
			queues[r][b].clear();
		}
	}
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
		totalReadRequests++;
		//判断当前请求是否是延迟敏感型线程的请求,如果是，将该请求放入敏感请求队列
		for (int i = 0; i < parentMC->lsNum; i++)
			if (parentMC->lsThread[i] == newBusPacket->threadId)
			{
				PRINT("延迟敏感型线程请求")
				lsQueue.push_back(newBusPacket);
				newBusPacket->priority = i;
				newBusPacket->marked = true;
				totalMarkedRequests++;
				reqsMarkedPerThread[newBusPacket->threadId]++;
				return;
			}
		unsigned rank = newBusPacket->rank;
		unsigned bank = newBusPacket->bank;
		queues[rank][bank].push_back(newBusPacket);
		if (queues[rank][bank].size()>CMD_QUEUE_DEPTH)
		{
			ERROR("== Error - Enqueued more than allowed in command queue");
			ERROR("						Need to call .hasRoomFor(int numberToEnqueue, unsigned rank, unsigned bank) first");
			exit(0);
		}
	}
}

//PBFMS
bool RWCommandQueue::schedulePBFMS(BusPacket **busPacket)
{
	if (totalReadRequests == 0)  //队列中没有读请求
		return false;
	if (totalMarkedRequests == 0)
	{
		//新的batch开始	
		//标记请求,统计max rule
		//PRINT("新的batch开始");
		for (size_t i = 0; i < NUM_THREAD; i++)
			maxRulePerthread[i] = 0;
		for (size_t i = 0; i < NUM_RANKS; i++)	
			for (size_t j = 0; j < NUM_BANKS; j++)
			{
				vector<BusPacket*> &queue = getCommandQueue(false, i, j);	
				for (size_t k = 0; k < NUM_THREAD; k++)
				{
					unsigned perBankCount = 0;
					for (size_t ii = 0; ii < queue.size(); ii++)
						if (queue[ii]->threadId == k)
						{
							perBankCount++;
							queue[ii]->marked = true;
							if (perBankCount == MARKING_CAP)
								break;
						}
					if (perBankCount > maxRulePerthread[k])
						maxRulePerthread[k] = perBankCount;
					reqsMarkedPerThread[k] += perBankCount;
					totalMarkedRequests += perBankCount;
				}
			}

		for (size_t i = 0; i < NUM_THREAD; i++)
			if (maxRulePerthread[i] == 0)
				maxRulePerthread[i] = MARKING_CAP + 1;

		threadPriority.clear();
		//延迟敏感型线程排名
		for (int i = 0; i < parentMC->lsNum; i++)
			threadPriority.push_back(parentMC->lsThread[i]);

		//获取其它排名
		for (size_t i = 0; i < NUM_THREAD -  parentMC->lsNum; i++)
		{
			unsigned min = maxRulePerthread[0];	
			size_t p = 0;
			for (size_t j = 1; j < NUM_THREAD; j++)
				if (maxRulePerthread[j] < min)
				{
					min = maxRulePerthread[j];	
					p = j;
				}
			for (size_t j = 0; j < NUM_THREAD; j++)	
				if (j != p && maxRulePerthread[j] == min && reqsMarkedPerThread[j] < reqsMarkedPerThread[p])
					p = j;
			threadPriority.push_back(p);
			maxRulePerthread[p] = MARKING_CAP + 2;
		}
		/*for (int i = 0; i < parentMC->lsThread.size(); i++)
		{
			int p;
			for (int j = 0; j < threadPriority.size(); j++)	
				if (parentMC->lsThread[i] == threadPriority[j])
				{
					p = j;
					break;
				}
			for (int j = i; j < p; j++)
				threadPriority[j + 1] = threadPriority[j];
			threadPriority[i] = parentMC->lsThread[i];
		}*/
		//设置每个请求中的优先级,将优先级高的请求放到队列前面
		for (size_t j = 0; j < NUM_RANKS; j++)
			for (size_t k = 0; k < NUM_BANKS; k++)
			{
				vector<BusPacket*> &queue = getCommandQueue(false, j, k);	
				unsigned hasThread = 0; //放到前面的请求个数
				for (size_t i = parentMC->lsNum; i < NUM_THREAD; i++)
				{
					unsigned threadId = threadPriority[i];	
					size_t ii = hasThread;
					while (ii < queue.size())
					{
						if (queue[ii]->threadId == threadId && queue[ii]->marked)
						{
							BusPacket* temp = queue[ii];
							if (ii > 0)
							{
								queue.erase(queue.begin() + ii);
								queue.insert(queue.begin() + hasThread, temp);
							}
							hasThread++;
						}
						ii++;
					}
				}
			}

	}
	if (lsQueue.size())  //有延迟敏感的请求
	{
		for (size_t i = 0; i < lsQueue.size(); i++)	
		{
			vector <BusPacket *> &queue = getCommandQueue(false, lsQueue[i]->rank, lsQueue[i]->bank);	
			lsQueue[i]->print();
			if (queue.empty())
				queue.push_back(lsQueue[i]);
			else
			{
				size_t j = 0;
				bool flag = false;
				for (; j < queue.size(); j++)
					if (!queue[j]->marked || queue[j]->priority < lsQueue[i]->priority)
					{
						queue.insert(queue.begin() + j, lsQueue[i]);
						flag = true;
						break;
					}	
				if (!flag)
					queue.push_back(lsQueue[i]);
			}
		}
		lsQueue.clear();
		this->print();
	}
	//线程优先级
	/*for (size_t i = 0; i < NUM_THREAD; i++)
		PRINT(threadPriority[i]);*/
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
				vector <BusPacket *> &refreshQueue = getCommandQueue(false, refreshRank,b);
				//队列中是否有标记的包
				bool hasMarkedPacket = false;
				for (size_t j = 0; j < refreshQueue.size(); j++)
					if (refreshQueue[j]->marked)
					{
						hasMarkedPacket = true;
						break;
					}
				for (size_t j=0;j<refreshQueue.size();j++)
				{
					BusPacket *packet = refreshQueue[j];
					if (hasMarkedPacket && packet->marked == false)
						continue;
					//if a command in the queue is going to the same row . . .
					if (bankStates[refreshRank][b].openRowAddress == packet->row &&
							 refreshRank == packet->rank && b == packet->bank)
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
								if (hasMarkedPacket)
								{
									reqsMarkedPerThread[packet->threadId]--;
									totalMarkedRequests--;
								}
								if (!curIsWrite)
									totalReadRequests--;
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
	bool hasTosendRead = false;
	bool foundIssuable = false;
	if (needToChange && !sendingREForPRE)
	{
		for (unsigned r = 0; r < NUM_RANKS; r++)		
		{
			for (unsigned b = 0; b < NUM_BANKS; b++)
			{
				if (bankStates[r][b].currentBankState == RowActive)
				{
					//search for commands going to an open row
					vector <BusPacket *> &queue = getCommandQueue(false, r, b);
					//队列中是否有标记的包
					bool hasMarkedPacket = false;
					for (size_t j = 0; j < queue.size(); j++)
						if (queue[j]->marked)
						{
							hasMarkedPacket = true;
							break;
						}
					for (size_t j=0;j< queue.size();j++)
					{
						BusPacket *packet = queue[j];
						if (hasMarkedPacket && packet->marked == false)
							break;
						//if a command in the queue is going to the same row . . .
						if (bankStates[r][b].openRowAddress == packet->row &&
								 r== packet->rank && b == packet->bank)
						{
							// . . . and is not an activate . . .
							if (packet->busPacketType != ACTIVATE)
							{
								hasTosendRead = true;
								// . . . and can be issued . . .
								if (isIssuable(packet))
								{
									//send it out
									*busPacket = packet;
									queue.erase(queue.begin()+j);
									if (hasMarkedPacket)
									{
										reqsMarkedPerThread[packet->threadId]--;
										totalMarkedRequests--;
									}
									totalReadRequests--;
									foundIssuable = true;
								}
								break;
							}
							else //command is an activate，优先转化为读
							{
								//if we've encountered another act, no other command will be of interest
								break;
							}
						}
					}
				}
				if (hasTosendRead)
					break;
			}
			if (hasTosendRead)
				break;
		}
	}
	//1.每个bank中的请求，如果有被标记，则标记请求行命中优先，否则行命中请求优先
	//2.标记请求中，高优先级优先
	//看看能否找到可发送的包
	if (!needToChange && !sendingREForPRE)
	{
		unsigned startingRank = nextRank;
		unsigned startingBank = nextBank;
		do // round robin over queues
		{
			vector<BusPacket *> &queue = getCommandQueue(false, nextRank,nextBank);
			bool hasMarkedPacket = false;
			for (size_t i = 0; i < queue.size(); i++)
				if (queue[i]->marked)
				{
					hasMarkedPacket = true;
					break;
				}
			//make sure there is something there first
			if (!queue.empty() && !((nextRank == refreshRank) && refreshWaiting))
			{
				//search from the beginning to find first issuable bus packet
				//row hit优先
				//PRINT("row hit优先");
				for (size_t i=0;i<queue.size();i++)
				{
					BusPacket *packet = queue[i];
					if (hasMarkedPacket && packet->marked == false) //marked优先
						continue;
					if (packet->rank == nextRank && packet->bank == nextBank && isIssuable(packet))
					{
						//check for dependencies
						bool dependencyFound = false;
						for (size_t j=0;j<i;j++)
						{
							BusPacket *prevPacket = queue[j];
							if (hasMarkedPacket && prevPacket->marked == false) //marked优先
								continue;
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
						if (i>0 && queue[i-1]->busPacketType == ACTIVATE)
						{
							rowAccessCounters[(*busPacket)->rank][(*busPacket)->bank]++;
							// i is being returned, but i-1 is being thrown away, so must delete it here 
							delete (queue[i-1]);

							// remove both i-1 (the activate) and i (we've saved the pointer in *busPacket)
							queue.erase(queue.begin()+i-1,queue.begin()+i+1);
							if (hasMarkedPacket)
							{
								reqsMarkedPerThread[(*busPacket)->threadId] -= 2;
								totalMarkedRequests -= 2;
							}
							if (!curIsWrite)
								totalReadRequests -= 2;
						}
						else // there's no activate before this packet
						{
							//or just remove the one bus packet
							BusPacket *next = queue[i + 1];
							queue.erase(queue.begin()+i);
							//当前发送的active是没有标记的包,将后面配对的读请求调到队列最前面
							if (hasMarkedPacket == false && (*busPacket)->busPacketType == ACTIVATE)
							{
								//PRINT("未标记请求的ACTIVE发送");
								queue.erase(queue.begin()+i);
								queue.insert(queue.begin(), next);
							}
							if (hasMarkedPacket)
							{
								reqsMarkedPerThread[(*busPacket)->threadId]--;
								totalMarkedRequests--;
							}
							if (!curIsWrite)
								totalReadRequests--;
						}

						foundIssuable = true;
						break;
					}
				}
			}	
			//if we found something, break out of do-while
			if (foundIssuable) break;

			nextRankAndBank(nextRank, nextBank); 
			if (startingRank == nextRank && startingBank == nextBank)
			{
				break;
			}
		}
		while (true);
	}

	//可以转换为写了	
	if (needToChange && !hasTosendRead)
	{
		needToChange = false;
		curIsWrite = true;
	}

	//看看能否关闭某行
	//if nothing was issuable, see if we can issue a PRE to an open bank
	//	that has no other commands waiting
	if (!sendingREForPRE && !foundIssuable)
	{
		//search for banks to close
		bool sendingPRE = false;
		unsigned startingRank = nextRankPRE;
		unsigned startingBank = nextBankPRE;

		do // round robin over all ranks and banks
		{
			vector <BusPacket *> &queue = getCommandQueue(false, nextRankPRE, nextBankPRE);
			bool found = false;
			//check if bank is open
			if (bankStates[nextRankPRE][nextBankPRE].currentBankState == RowActive)
			{
				bool hasMarkedPacket = false;
				for (size_t i = 0; i < queue.size(); i++)
					if (queue[i]->marked)
					{
						hasMarkedPacket = true;
						break;
					}
				//优先关闭优先级高的请求对应的bank
				unsigned firstThread = 0xff;
				for (size_t i=0;i<queue.size();i++)
				{
					if (hasMarkedPacket && queue[i]->marked == false)  //标记的请求优先
						break;
					if (queue[i]->marked && firstThread == 0xff)
						firstThread = queue[i]->threadId;
					//PRINT("高优先级请求优先");
					if (hasMarkedPacket  && queue[i]->threadId != firstThread)
						break;
					if (queue[i]->rank == nextRankPRE && queue[i]->bank == nextBankPRE &&
							queue[i]->row == bankStates[nextRankPRE][nextBankPRE].openRowAddress)
					{
						found = true;
						break;
					}
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

		//if no PREs could be sent, just return false
		if (!sendingPRE) return false;
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
		nextRankAndBank(nextRank, nextBank);
	}

	//if its an activate, add a tfaw counter
	if ((*busPacket)->busPacketType==ACTIVATE)
	{
		tFAWCountdown[(*busPacket)->rank].push_back(tFAW);
	}

	return true;
}
//par-bs scheduling
bool RWCommandQueue::scheduleParbs(BusPacket **busPacket)
{
	if (totalReadRequests == 0)  //队列中没有读请求
		return false;
	if (totalMarkedRequests == 0)
	{
		//新的batch开始	
		//标记请求,统计max rule
		//PRINT("新的batch开始");
		for (size_t i = 0; i < NUM_THREAD; i++)
			maxRulePerthread[i] = 0;
		for (size_t i = 0; i < NUM_RANKS; i++)	
			for (size_t j = 0; j < NUM_BANKS; j++)
			{
				vector<BusPacket*> &queue = getCommandQueue(false, i, j);	
				for (size_t k = 0; k < NUM_THREAD; k++)
				{
					unsigned perBankCount = 0;
					for (size_t ii = 0; ii < queue.size(); ii++)
						if (queue[ii]->threadId == k)
						{
							perBankCount++;
							queue[ii]->marked = true;
							if (perBankCount == MARKING_CAP)
								break;
						}
					if (perBankCount > maxRulePerthread[k])
						maxRulePerthread[k] = perBankCount;
					reqsMarkedPerThread[k] += perBankCount;
					totalMarkedRequests += perBankCount;
				}
			}

		for (size_t i = 0; i < NUM_THREAD; i++)
			if (maxRulePerthread[i] == 0)
				maxRulePerthread[i] = MARKING_CAP + 1;

		threadPriority.clear();
		//获取其它排名
		for (size_t i = 0; i < NUM_THREAD -  parentMC->lsNum; i++)
		{
			unsigned min = maxRulePerthread[0];	
			size_t p = 0;
			for (size_t j = 1; j < NUM_THREAD; j++)
				if (maxRulePerthread[j] < min)
				{
					min = maxRulePerthread[j];	
					p = j;
				}
			for (size_t j = 0; j < NUM_THREAD; j++)	
				if (j != p && maxRulePerthread[j] == min && reqsMarkedPerThread[j] < reqsMarkedPerThread[p])
					p = j;
			threadPriority.push_back(p);
			maxRulePerthread[p] = MARKING_CAP + 2;
		}
		//设置每个请求中的优先级,将优先级高的请求放到队列前面
		for (size_t j = 0; j < NUM_RANKS; j++)
			for (size_t k = 0; k < NUM_BANKS; k++)
			{
				vector<BusPacket*> &queue = getCommandQueue(false, j, k);	
				unsigned hasThread = 0; //放到前面的请求个数
				for (size_t i = 0; i < NUM_THREAD; i++)
				{
					unsigned threadId = threadPriority[i];	
					size_t ii = hasThread;
					while (ii < queue.size())
					{
						if (queue[ii]->threadId == threadId && queue[ii]->marked)
						{
							BusPacket* temp = queue[ii];
							if (ii > 0)
							{
								queue.erase(queue.begin() + ii);
								queue.insert(queue.begin() + hasThread, temp);
							}
							hasThread++;
						}
						ii++;
					}
				}
			}

	}
	//线程优先级
	/*for (size_t i = 0; i < NUM_THREAD; i++)
		PRINT(threadPriority[i]);*/
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
				vector <BusPacket *> &refreshQueue = getCommandQueue(false, refreshRank,b);
				//队列中是否有标记的包
				bool hasMarkedPacket = false;
				for (size_t j = 0; j < refreshQueue.size(); j++)
					if (refreshQueue[j]->marked)
					{
						hasMarkedPacket = true;
						break;
					}
				for (size_t j=0;j<refreshQueue.size();j++)
				{
					BusPacket *packet = refreshQueue[j];
					if (hasMarkedPacket && packet->marked == false)
						continue;
					//if a command in the queue is going to the same row . . .
					if (bankStates[refreshRank][b].openRowAddress == packet->row &&
							 refreshRank == packet->rank && b == packet->bank)
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
								if (hasMarkedPacket)
								{
									reqsMarkedPerThread[packet->threadId]--;
									totalMarkedRequests--;
								}
								if (!curIsWrite)
									totalReadRequests--;
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
	bool hasTosendRead = false;
	bool foundIssuable = false;
	//待转换到写
	if (needToChange && !sendingREForPRE)
	{
		for (unsigned r = 0; r < NUM_RANKS; r++)		
		{
			for (unsigned b = 0; b < NUM_BANKS; b++)
			{
				if (bankStates[r][b].currentBankState == RowActive)
				{
					//search for commands going to an open row
					vector <BusPacket *> &queue = getCommandQueue(false, r, b);
					//队列中是否有标记的包
					bool hasMarkedPacket = false;
					for (size_t j = 0; j < queue.size(); j++)
						if (queue[j]->marked)
						{
							hasMarkedPacket = true;
							break;
						}
					for (size_t j=0;j< queue.size();j++)
					{
						BusPacket *packet = queue[j];
						if (hasMarkedPacket && packet->marked == false)
							break;
						//if a command in the queue is going to the same row . . .
						if (bankStates[r][b].openRowAddress == packet->row &&
								 r== packet->rank && b == packet->bank)
						{
							// . . . and is not an activate . . .
							if (packet->busPacketType != ACTIVATE)
							{
								hasTosendRead = true;
								// . . . and can be issued . . .
								if (isIssuable(packet))
								{
									//send it out
									*busPacket = packet;
									queue.erase(queue.begin()+j);
									if (hasMarkedPacket)
									{
										reqsMarkedPerThread[packet->threadId]--;
										totalMarkedRequests--;
									}
									totalReadRequests--;
									foundIssuable = true;
								}
								break;
							}
							else //command is an activate，优先转化为读
							{
								//if we've encountered another act, no other command will be of interest
								break;
							}
						}
					}
				}
				if (hasTosendRead)
					break;
			}
			if (hasTosendRead)
				break;
		}
	}
	//1.每个bank中的请求，如果有被标记，则标记请求行命中优先，否则行命中请求优先
	//2.标记请求中，高优先级优先
	//看看能否找到可发送的包
	if (!needToChange && !sendingREForPRE)
	{
		unsigned startingRank = nextRank;
		unsigned startingBank = nextBank;
		do // round robin over queues
		{
			vector<BusPacket *> &queue = getCommandQueue(false, nextRank,nextBank);
			bool hasMarkedPacket = false;
			for (size_t i = 0; i < queue.size(); i++)
				if (queue[i]->marked)
				{
					hasMarkedPacket = true;
					break;
				}
			//make sure there is something there first
			if (!queue.empty() && !((nextRank == refreshRank) && refreshWaiting))
			{
				//search from the beginning to find first issuable bus packet
				//row hit优先
				//PRINT("row hit优先");
				for (size_t i=0;i<queue.size();i++)
				{
					BusPacket *packet = queue[i];
					if (hasMarkedPacket && packet->marked == false) //marked优先
						continue;
					if (packet->rank == nextRank && packet->bank == nextBank && isIssuable(packet))
					{
						//check for dependencies
						bool dependencyFound = false;
						for (size_t j=0;j<i;j++)
						{
							BusPacket *prevPacket = queue[j];
							if (hasMarkedPacket && prevPacket->marked == false) //marked优先
								continue;
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
						if (i>0 && queue[i-1]->busPacketType == ACTIVATE)
						{
							rowAccessCounters[(*busPacket)->rank][(*busPacket)->bank]++;
							// i is being returned, but i-1 is being thrown away, so must delete it here 
							delete (queue[i-1]);

							// remove both i-1 (the activate) and i (we've saved the pointer in *busPacket)
							queue.erase(queue.begin()+i-1,queue.begin()+i+1);
							if (hasMarkedPacket)
							{
								reqsMarkedPerThread[(*busPacket)->threadId] -= 2;
								totalMarkedRequests -= 2;
							}
							if (!curIsWrite)
								totalReadRequests -= 2;
						}
						else // there's no activate before this packet
						{
							//or just remove the one bus packet
							BusPacket *next = queue[i + 1];
							queue.erase(queue.begin()+i);
							//当前发送的active是没有标记的包,将后面配对的读请求调到队列最前面
							if (hasMarkedPacket == false && (*busPacket)->busPacketType == ACTIVATE)
							{
								//PRINT("未标记请求的ACTIVE发送");
								queue.erase(queue.begin()+i);
								queue.insert(queue.begin(), next);
							}
							if (hasMarkedPacket)
							{
								reqsMarkedPerThread[(*busPacket)->threadId]--;
								totalMarkedRequests--;
							}
							if (!curIsWrite)
								totalReadRequests--;
						}

						foundIssuable = true;
						break;
					}
				}
			}	
			//if we found something, break out of do-while
			if (foundIssuable) break;

			nextRankAndBank(nextRank, nextBank); 
			if (startingRank == nextRank && startingBank == nextBank)
			{
				break;
			}
		}
		while (true);
	}

	//可以转换为写了	
	if (needToChange && !hasTosendRead)
	{
		needToChange = false;
		curIsWrite = true;
	}

	//看看能否关闭某行
	//if nothing was issuable, see if we can issue a PRE to an open bank
	//	that has no other commands waiting
	if (!sendingREForPRE && !foundIssuable)
	{
		//search for banks to close
		bool sendingPRE = false;
		unsigned startingRank = nextRankPRE;
		unsigned startingBank = nextBankPRE;

		do // round robin over all ranks and banks
		{
			vector <BusPacket *> &queue = getCommandQueue(false, nextRankPRE, nextBankPRE);
			bool found = false;
			//check if bank is open
			if (bankStates[nextRankPRE][nextBankPRE].currentBankState == RowActive)
			{
				bool hasMarkedPacket = false;
				for (size_t i = 0; i < queue.size(); i++)
					if (queue[i]->marked)
					{
						hasMarkedPacket = true;
						break;
					}
				//优先关闭优先级高的请求对应的bank
				unsigned firstThread = 0xff;
				for (size_t i=0;i<queue.size();i++)
				{
					if (hasMarkedPacket && queue[i]->marked == false)  //标记的请求优先
						break;
					if (queue[i]->marked && firstThread == 0xff)
						firstThread = queue[i]->threadId;
					//PRINT("高优先级请求优先");
					if (hasMarkedPacket  && queue[i]->threadId != firstThread)
						break;
					if (queue[i]->rank == nextRankPRE && queue[i]->bank == nextBankPRE &&
							queue[i]->row == bankStates[nextRankPRE][nextBankPRE].openRowAddress)
					{
						found = true;
						break;
					}
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

		//if no PREs could be sent, just return false
		if (!sendingPRE) return false;
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
		nextRankAndBank(nextRank, nextBank);
	}

	//if its an activate, add a tfaw counter
	if ((*busPacket)->busPacketType==ACTIVATE)
	{
		tFAWCountdown[(*busPacket)->rank].push_back(tFAW);
	}

	return true;
}

//Removes the next item from the command queue based on the system's
//command scheduling policy
bool RWCommandQueue::pop(BusPacket **busPacket)
{
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
	//判断调度算法
	/*if (schedulingPolicy == PBFMS)	
		return schedulePBFMS(busPacket);*/
	//总原则：转换到写后，尽量让写队列为空
	//1.读请求为0，并且写队列长度超过WRITE_LOW_THEROLD, 由读转换到写
	//2.写队列长度达到WRITE_HIGHT_THEROLD，读队列个数是偶数个，由读转换到写
	//3.上一个请求是写，并且写队列不为空，继续写
	if ((totalReadRequests == 0 && writeQueues.size() >  WRITE_LOW_THEROLD) || (curIsWrite && writeQueues.size() > 0)) 
		curIsWrite = true;
	else if (!curIsWrite && writeQueues.size() >= WRITE_HIGHT_THEROLD) 
		needToChange = true;	
	else
		curIsWrite = false;
	
	if (schedulingPolicy == PBFMS && lsQueue.size() && curIsWrite && writeQueues.size() <= WRITE_LOW_THEROLD)
		needToChange = true;

	//std::cout << "curIsWrite : " << curIsWrite << std::endl;
	if (!curIsWrite && schedulingPolicy == PARBS)
		return scheduleParbs(busPacket);
	else if (!curIsWrite && schedulingPolicy == PBFMS)
		return schedulePBFMS(busPacket);
	if (rowBufferPolicy==ClosePage)
	{
	}
	else if (rowBufferPolicy==OpenPage)
	{
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
					vector <BusPacket *> &refreshQueue = getCommandQueue(curIsWrite, refreshRank,b);

					for (size_t j=0;j<refreshQueue.size();j++)
					{
						BusPacket *packet = refreshQueue[j];
						//if a command in the queue is going to the same row . . .
						if (bankStates[refreshRank][b].openRowAddress == packet->row &&
								 refreshRank == packet->rank && b == packet->bank)
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
		bool hasToSendReadOrWrite = false;
		bool foundIssuable = false;
		if (!sendingREForPRE && needToChange)
		{
			for (unsigned r = 0; r < NUM_RANKS; r++)		
			{
				for (unsigned b = 0; b < NUM_BANKS; b++)
				{
					if (bankStates[r][b].currentBankState == RowActive)
					{
						//search for commands going to an open row
						vector <BusPacket *> &queue = getCommandQueue(curIsWrite,r,b);
						for (size_t j=0;j< queue.size();j++)
						{
							BusPacket *packet = queue[j];
							//if a command in the queue is going to the same row . . .
							if (bankStates[r][b].openRowAddress == packet->row &&
									 r== packet->rank && b == packet->bank)
							{
								// . . . and is not an activate . . .
								if (packet->busPacketType != ACTIVATE)
								{
									hasToSendReadOrWrite= true;
									// . . . and can be issued . . .
									if (isIssuable(packet))
									{
										//send it out
										*busPacket = packet;
										queue.erase(queue.begin()+j);
										foundIssuable = true;
									}
									break;
								}
								else //command is an activate，优先转化为读
								{
									//if we've encountered another act, no other command will be of interest
									break;
								}
							}
						}
					}
					if (hasToSendReadOrWrite)
						break;
				}
				if (hasToSendReadOrWrite)
					break;
			}
		}
		//看看能否找到可发送的包
		if (!needToChange && !sendingREForPRE)
		{
			unsigned startingRank = nextRank;
			unsigned startingBank = nextBank;
			do // round robin over queues
			{
				vector<BusPacket *> &queue = getCommandQueue(curIsWrite, nextRank,nextBank);
				//make sure there is something there first
				if (!queue.empty() && !((nextRank == refreshRank) && refreshWaiting))
				{
					//search from the beginning to find first issuable bus packet
					for (size_t i=0;i<queue.size();i++)
					{
						BusPacket *packet = queue[i];
						if (packet->rank == nextRank && packet->bank == nextBank && isIssuable(packet))
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
								if (!curIsWrite)
									totalReadRequests -= 2;
							}
							else // there's no activate before this packet
							{
								//or just remove the one bus packet
								queue.erase(queue.begin()+i);
								if (!curIsWrite)
									totalReadRequests --;
							}

							foundIssuable = true;
							break;
						}
					}
				}

				//if we found something, break out of do-while
				if (foundIssuable) break;

				nextRankAndBank(nextRank, nextBank); 
				if (startingRank == nextRank && startingBank == nextBank)
				{
					break;
				}
			}
			while (true);
		}
		

		//看看能否关闭某行
		//if nothing was issuable, see if we can issue a PRE to an open bank
		//	that has no other commands waiting
		if (!sendingREForPRE && !foundIssuable)
		{
			//search for banks to close
			bool sendingPRE = false;
			unsigned startingRank = nextRankPRE;
			unsigned startingBank = nextBankPRE;

			do // round robin over all ranks and banks
			{
				vector <BusPacket *> &queue = getCommandQueue(curIsWrite, nextRankPRE, nextBankPRE);
				bool found = false;
				//check if bank is open
				if (bankStates[nextRankPRE][nextBankPRE].currentBankState == RowActive)
				{
					for (size_t i=0;i<queue.size();i++)
					{
						//if there is something going to that bank and row, then we don't want to send a PRE
						if (queue[i]->rank == nextRankPRE && queue[i]->bank == nextBankPRE &&
								queue[i]->row == bankStates[nextRankPRE][nextBankPRE].openRowAddress)
						{
							found = true;
							break;
						}
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
			if (needToChange && !hasToSendReadOrWrite)
			{
				curIsWrite = !curIsWrite;
				needToChange = false;
			}
			//if no PREs could be sent, just return false
			if (!sendingPRE) return false;
		}
		if (needToChange && !hasToSendReadOrWrite)
		{
			curIsWrite = !curIsWrite;
			needToChange = false;
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
		nextRankAndBank(nextRank, nextBank);
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
	vector<BusPacket *> &queue = getCommandQueue(isWrite, rank, bank); 
	return (CMD_QUEUE_DEPTH - queue.size() >= numberToEnqueue);
}

//prints the contents of the command queue
void RWCommandQueue::print()
{
	PRINT("\n== Printing Per Rank, Per Bank Queue" );
	PRINT(endl << "== Printing Read Queue" );
	for (size_t i=0;i<NUM_RANKS;i++)
	{
		PRINT(" = Rank " << i );
		for (size_t j=0;j<NUM_BANKS;j++)
		{
			PRINT("    Bank "<< j << "   size : " << queues[i][j].size() );

			for (size_t k=0;k<queues[i][j].size();k++)
			{
				PRINTN("       " << k << "]");
				queues[i][j][k]->print();
			}
		}
	}

	PRINT(endl << "== Printing Write Queue" );
	for (size_t i=0;i<writeQueues.size();i++)
	{
		PRINT(" ");
		writeQueues[i]->print();
	}
}

/** 
 * return a reference to the queue for a given rank, bank. Since we
 * don't always have a per bank queuing structure, sometimes the bank
 * argument is ignored (and the 0th index is returned 
 */
vector<BusPacket *> &RWCommandQueue::getCommandQueue(bool isWrite, unsigned rank, unsigned bank)
{
	if (!isWrite)
	{
		return queues[rank][bank];
	}
	else
	{
		return writeQueues;
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

//figures out if a rank's queue is empty
bool RWCommandQueue::isEmpty(unsigned rank)
{
	if (curIsWrite)
	{
		for(size_t i = 0; i < writeQueues.size(); i++)
			if (writeQueues[i]->rank == rank)
				return false;
	}
	else
	{
		for (size_t i=0;i<NUM_BANKS;i++)
		{
			if (!queues[rank][i].empty()) return false;
		}
	}
	return true;
}

//tells the command queue that a particular rank is in need of a refresh
void RWCommandQueue::needRefresh(unsigned rank)
{
	refreshWaiting = true;
	refreshRank = rank;
}

void RWCommandQueue::nextRankAndBank(unsigned &rank, unsigned &bank)
{
	//bank-then-rank round robin
	bank++;
	if (bank == NUM_BANKS)
	{
		bank = 0;
		rank++;
		if (rank == NUM_RANKS)
		{
			rank = 0;
		}
	}
}

void RWCommandQueue::update()
{
	//do nothing since pop() is effectively update(),
	//needed for SimulatorObject
	//TODO: make RWCommandQueue not a SimulatorObject
}
