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



//MemoryController.cpp
//
//Class file for memory controller object
//

#include "MemoryController.h"
#include "MemorySystem.h"
#include "AddressMapping.h"
#include "IniReader.h"

#define SEQUENTIAL(rank,bank) (rank*NUM_BANKS)+bank

/* Power computations are localized to MemoryController.cpp */ 
extern unsigned IDD0;
extern unsigned IDD1;
extern unsigned IDD2P;
extern unsigned IDD2Q;
extern unsigned IDD2N;
extern unsigned IDD3Pf;
extern unsigned IDD3Ps;
extern unsigned IDD3N;
extern unsigned IDD4W;
extern unsigned IDD4R;
extern unsigned IDD5;
extern unsigned IDD6;
extern unsigned IDD6L;
extern unsigned IDD7;
extern float Vdd; 

using namespace DRAMSim;

MemoryController::MemoryController(MemorySystem *parent, CSVWriter &csvOut_, ostream &dramsim_log_) :
		dramsim_log(dramsim_log_),
		bankStates(NUM_RANKS, vector<BankState>(NUM_BANKS, dramsim_log)),
		commandQueue(bankStates, dramsim_log_),
		poppedBusPacket(NULL),
		csvOut(csvOut_),
		totalTransactions(0),
		refreshRank(0)
{
	//get handle on parent
	parentMemorySystem = parent;


	//bus related fields
	outgoingCmdPacket = NULL;
	outgoingDataPacket = NULL;
	dataCyclesLeft = 0;
	cmdCyclesLeft = 0;

	//set here to avoid compile errors
	currentClockCycle = 0;
	currentDomain = 0;
	BTAPhase = 0;

        numFakeFS = 0;

	/*
	currentPhase = -1;
	remainingInPhase = 0;

	totalNodes = 0;
	totalFakeReadRequests = 0;
	totalFakeWriteRequests = 0;

	
	iDefenceDomain = std::vector<int>;
	dDefenceDomain = 0;
	old_iDefenceDomain = 999;
	old_dDefenceDomain = 999;
	*/

	//reserve memory for vectors
	transactionQueue.reserve(TRANS_QUEUE_DEPTH);
	defenceQueue.reserve(DEFENCE_QUEUE_DEPTH);
	powerDown = vector<bool>(NUM_RANKS,false);
	grandTotalBankAccesses = vector<uint64_t>(NUM_RANKS*NUM_BANKS,0);
	totalReadsPerBank = vector<uint64_t>(NUM_RANKS*NUM_BANKS,0);
	totalWritesPerBank = vector<uint64_t>(NUM_RANKS*NUM_BANKS,0);
	totalReadsPerRank = vector<uint64_t>(NUM_RANKS,0);
	totalWritesPerRank = vector<uint64_t>(NUM_RANKS,0);

	writeDataCountdown.reserve(NUM_RANKS);
	writeDataToSend.reserve(NUM_RANKS);
	refreshCountdown.reserve(NUM_RANKS);

	//Power related packets
	backgroundEnergy = vector <uint64_t >(NUM_RANKS,0);
	burstEnergy = vector <uint64_t> (NUM_RANKS,0);
	actpreEnergy = vector <uint64_t> (NUM_RANKS,0);
	refreshEnergy = vector <uint64_t> (NUM_RANKS,0);

	totalEpochLatency = vector<uint64_t> (NUM_RANKS*NUM_BANKS,0);

	//staggers when each rank is due for a refresh
	for (size_t i=0;i<NUM_RANKS;i++)
	{
		refreshCountdown.push_back((int)((REFRESH_PERIOD/tCK)/NUM_RANKS)*(i+1));
	}
}

//get a bus packet from either data or cmd bus
void MemoryController::receiveFromBus(BusPacket *bpacket)
{
	if (bpacket->busPacketType != DATA)
	{
		ERROR("== Error - Memory Controller received a non-DATA bus packet from rank");
		bpacket->print();
		exit(0);
	}

	if (DEBUG_BUS)
	{
		PRINTN(" -- MC Receiving From Data Bus : ");
		bpacket->print();
	}

	//add to return read data queue
	returnTransaction.push_back(new Transaction(RETURN_DATA, bpacket->physicalAddress, bpacket->data, -1, -1, false, -1));
	totalReadsPerBank[SEQUENTIAL(bpacket->rank,bpacket->bank)]++;

	// this delete statement saves a mindboggling amount of memory
	delete(bpacket);
}

//sends read data back to the CPU
void MemoryController::returnReadData(const Transaction *trans)
{
	if (parentMemorySystem->ReturnReadData!=NULL)
	{
		(*parentMemorySystem->ReturnReadData)(parentMemorySystem->systemID, trans->address, currentClockCycle);
	}
}

//gives the memory controller a handle on the rank objects
void MemoryController::attachRanks(vector<Rank *> *ranks)
{
	this->ranks = ranks;
}

//gives the memory controller a handle on the rank objects
void MemoryController::initDefence(int domainID)
{
	/* Create bookkeeping maps for convenience */
	numLoops.push_back(this->dag[domainID].size());
	currentLoop.push_back(0);
	currentLoopIteration.push_back(0);

	parentList.push_back(vector<map<int,vector<int>>>());
	childrenList.push_back(vector<map<int,vector<int>>>());
	weightList.push_back(vector<map<int,map<int, int>>>());
	finishTimes.push_back(vector<map<int,uint64_t>>());

	totalNodes.push_back(0);
	totalFakeReadRequests.push_back(0);
	totalFakeWriteRequests.push_back(0);


    // Determine lineages
	for (int i = 0; i < numLoops[domainID]; i++) { // Per loop
		parentList[domainID].push_back(map<int,vector<int>>());
		childrenList[domainID].push_back(map<int,vector<int>>());
		weightList[domainID].push_back(map<int,map<int, int>>());
		finishTimes[domainID].push_back(map<int,uint64_t>());

		for (auto& edge : this->dag[domainID][to_string(i)]["edge"].items()) {
			int srcNode = edge.value()["sourceID"];
			int destNode = edge.value()["destID"];
			parentList[domainID][i][destNode].push_back(srcNode);
			childrenList[domainID][i][srcNode].push_back(destNode);
			weightList[domainID][i][srcNode][destNode] = edge.value()["latency"];
		}

		for (auto& node : this->dag[domainID][to_string(i)]["node"].items()) {
			finishTimes[domainID][i][node.value()["nodeID"]] = std::numeric_limits<uint64_t>::max();
		}
	}

	// Immediately schedule the initial node 
	int scheduledTime = currentClockCycle + 1;
	while (scheduleNode.count(scheduledTime) > 0) scheduledTime++;
	scheduleNode[scheduledTime] = 0;
	scheduleDomain[scheduledTime] = domainID;

	PRINT("Initializing Defence!");
}

void MemoryController::stopDefence()
{
	PRINT("Stopping Defence Deprecated!");
}

//memory controller update
void MemoryController::update()
{

	//PRINT(" ------------------------- [" << currentClockCycle << "/" << nextFRClockCycle << "] -------------------------");

	if (currentClockCycle > nextFRClockCycle) {
		nextFRClockCycle += FIXED_SERVICE_RATE;
                commandQueue.nextFRClockCycle += FIXED_SERVICE_RATE;
	}
	//update bank states
	for (size_t i=0;i<NUM_RANKS;i++)
	{
		for (size_t j=0;j<NUM_BANKS;j++)
		{
			if (bankStates[i][j].stateChangeCountdown>0)
			{
				//decrement counters
				bankStates[i][j].stateChangeCountdown--;

				//if counter has reached 0, change state
				if (bankStates[i][j].stateChangeCountdown == 0)
				{
					switch (bankStates[i][j].lastCommand)
					{
						//only these commands have an implicit state change
					case WRITE_P:
					case READ_P:
						bankStates[i][j].currentBankState = Precharging;
						bankStates[i][j].lastCommand = PRECHARGE;
						bankStates[i][j].stateChangeCountdown = tRP;
						break;

					case REFRESH:
					case PRECHARGE:
						bankStates[i][j].currentBankState = Idle;
						break;
					default:
						break;
					}
				}
			}
		}
	}


	//check for outgoing command packets and handle countdowns
	if (outgoingCmdPacket != NULL)
	{
		cmdCyclesLeft--;
		if (cmdCyclesLeft == 0) //packet is ready to be received by rank
		{
			(*ranks)[outgoingCmdPacket->rank]->receiveFromBus(outgoingCmdPacket);
			outgoingCmdPacket = NULL;
		}
	}

	//check for outgoing data packets and handle countdowns
	if (outgoingDataPacket != NULL)
	{
		dataCyclesLeft--;
		if (dataCyclesLeft == 0)
		{
			//inform upper levels that a write is done
			if (parentMemorySystem->WriteDataDone!=NULL && outgoingDataPacket)
			{
				(*parentMemorySystem->WriteDataDone)(parentMemorySystem->systemID,outgoingDataPacket->physicalAddress, currentClockCycle);
			}

			(*ranks)[outgoingDataPacket->rank]->receiveFromBus(outgoingDataPacket);
			outgoingDataPacket=NULL;
		}
	}


	//if any outstanding write data needs to be sent
	//and the appropriate amount of time has passed (WL)
	//then send data on bus
	//
	//write data held in fifo vector along with countdowns
	if (writeDataCountdown.size() > 0)
	{
		for (size_t i=0;i<writeDataCountdown.size();i++)
		{
			writeDataCountdown[i]--;
		}

		if (writeDataCountdown[0]==0)
		{
			//send to bus and print debug stuff
			if (DEBUG_BUS)
			{
				PRINTN(" -- MC Issuing On Data Bus    : ");
				writeDataToSend[0]->print();
			}

			// queue up the packet to be sent
			if (outgoingDataPacket != NULL)
			{
				ERROR("== Error - Data Bus Collision");
				exit(-1);
			}

			outgoingDataPacket = writeDataToSend[0];
			dataCyclesLeft = BL/2;

			totalTransactions++;
			totalWritesPerBank[SEQUENTIAL(writeDataToSend[0]->rank,writeDataToSend[0]->bank)]++;

			writeDataCountdown.erase(writeDataCountdown.begin());
			writeDataToSend.erase(writeDataToSend.begin());
		}
	}

	//if its time for a refresh issue a refresh
	// else pop from command queue if it's not empty
	if (refreshCountdown[refreshRank]==0)
	{
		commandQueue.needRefresh(refreshRank);
		(*ranks)[refreshRank]->refreshWaiting = true;
		refreshCountdown[refreshRank] =	 REFRESH_PERIOD/tCK;
		refreshRank++;
		if (refreshRank == NUM_RANKS)
		{
			refreshRank = 0;
		}
	}
	//if a rank is powered down, make sure we power it up in time for a refresh
	else if (powerDown[refreshRank] && refreshCountdown[refreshRank] <= tXP)
	{
		(*ranks)[refreshRank]->refreshWaiting = true;
	}

	//pass a pointer to a poppedBusPacket

	//function returns true if there is something valid in poppedBusPacket
	if (commandQueue.pop(&poppedBusPacket))
	{
		if (!poppedBusPacket->isFake && (poppedBusPacket->busPacketType == WRITE || poppedBusPacket->busPacketType == WRITE_P))
		{

			writeDataToSend.push_back(new BusPacket(DATA, poppedBusPacket->physicalAddress, poppedBusPacket->column,
			                                    poppedBusPacket->row, poppedBusPacket->rank, poppedBusPacket->bank,
			                                    poppedBusPacket->data, poppedBusPacket->isFake, poppedBusPacket->securityDomain, dramsim_log));
			writeDataCountdown.push_back(WL);
		}

		//
		//update each bank's state based on the command that was just popped out of the command queue
		//
		//for readability's sake
		unsigned rank = poppedBusPacket->rank;
		unsigned bank = poppedBusPacket->bank;
		switch (poppedBusPacket->busPacketType)
		{
			case READ_P:
			case READ:
				//add energy to account for total
				if (DEBUG_POWER)
				{
					PRINT(" ++ Adding Read energy to total energy");
				}
				burstEnergy[rank] += (IDD4R - IDD3N) * BL/2 * NUM_DEVICES;
				if (poppedBusPacket->busPacketType == READ_P) 
				{
					//Don't bother setting next read or write times because the bank is no longer active
					//bankStates[rank][bank].currentBankState = Idle;
					bankStates[rank][bank].nextActivate = max(currentClockCycle + READ_AUTOPRE_DELAY,
							bankStates[rank][bank].nextActivate);
					bankStates[rank][bank].lastCommand = READ_P;
					bankStates[rank][bank].stateChangeCountdown = READ_TO_PRE_DELAY;
				}
				else if (poppedBusPacket->busPacketType == READ)
				{
					bankStates[rank][bank].nextPrecharge = max(currentClockCycle + READ_TO_PRE_DELAY,
							bankStates[rank][bank].nextPrecharge);
					bankStates[rank][bank].lastCommand = READ;

				}

				for (size_t i=0;i<NUM_RANKS;i++)
				{
					for (size_t j=0;j<NUM_BANKS;j++)
					{
						if (i!=poppedBusPacket->rank)
						{
							//check to make sure it is active before trying to set (save's time?)
							if (bankStates[i][j].currentBankState == RowActive)
							{
								bankStates[i][j].nextRead = max(currentClockCycle + BL/2 + tRTRS, bankStates[i][j].nextRead);
								bankStates[i][j].nextWrite = max(currentClockCycle + READ_TO_WRITE_DELAY,
										bankStates[i][j].nextWrite);
							}
						}
						else
						{
							bankStates[i][j].nextRead = max(currentClockCycle + max(tCCD, BL/2), bankStates[i][j].nextRead);
							bankStates[i][j].nextWrite = max(currentClockCycle + READ_TO_WRITE_DELAY,
									bankStates[i][j].nextWrite);
						}
					}
				}

				if (poppedBusPacket->busPacketType == READ_P)
				{
					//set read and write to nextActivate so the state table will prevent a read or write
					//  being issued (in cq.isIssuable())before the bank state has been changed because of the
					//  auto-precharge associated with this command
					bankStates[rank][bank].nextRead = bankStates[rank][bank].nextActivate;
					bankStates[rank][bank].nextWrite = bankStates[rank][bank].nextActivate;
				}

				break;
			case WRITE_P:
			case WRITE:
				if (poppedBusPacket->busPacketType == WRITE_P) 
				{
					bankStates[rank][bank].nextActivate = max(currentClockCycle + WRITE_AUTOPRE_DELAY,
							bankStates[rank][bank].nextActivate);
					bankStates[rank][bank].lastCommand = WRITE_P;
					bankStates[rank][bank].stateChangeCountdown = WRITE_TO_PRE_DELAY;
				}
				else if (poppedBusPacket->busPacketType == WRITE)
				{
					bankStates[rank][bank].nextPrecharge = max(currentClockCycle + WRITE_TO_PRE_DELAY,
							bankStates[rank][bank].nextPrecharge);
					bankStates[rank][bank].lastCommand = WRITE;
				}


				//add energy to account for total
				if (DEBUG_POWER)
				{
					PRINT(" ++ Adding Write energy to total energy");
				}
				burstEnergy[rank] += (IDD4W - IDD3N) * BL/2 * NUM_DEVICES;

				for (size_t i=0;i<NUM_RANKS;i++)
				{
					for (size_t j=0;j<NUM_BANKS;j++)
					{
						if (i!=poppedBusPacket->rank)
						{
							if (bankStates[i][j].currentBankState == RowActive)
							{
								bankStates[i][j].nextWrite = max(currentClockCycle + BL/2 + tRTRS, bankStates[i][j].nextWrite);
								bankStates[i][j].nextRead = max(currentClockCycle + WRITE_TO_READ_DELAY_R,
										bankStates[i][j].nextRead);
							}
						}
						else
						{
							bankStates[i][j].nextWrite = max(currentClockCycle + max(BL/2, tCCD), bankStates[i][j].nextWrite);
							bankStates[i][j].nextRead = max(currentClockCycle + WRITE_TO_READ_DELAY_B,
									bankStates[i][j].nextRead);
						}
					}
				}

				//set read and write to nextActivate so the state table will prevent a read or write
				//  being issued (in cq.isIssuable())before the bank state has been changed because of the
				//  auto-precharge associated with this command
				if (poppedBusPacket->busPacketType == WRITE_P)
				{
					bankStates[rank][bank].nextRead = bankStates[rank][bank].nextActivate;
					bankStates[rank][bank].nextWrite = bankStates[rank][bank].nextActivate;
				}

				break;
			case ACTIVATE:
				//add energy to account for total
				if (DEBUG_POWER)
				{
					PRINT(" ++ Adding Activate and Precharge energy to total energy");
				}
				actpreEnergy[rank] += ((IDD0 * tRC) - ((IDD3N * tRAS) + (IDD2N * (tRC - tRAS)))) * NUM_DEVICES;

				bankStates[rank][bank].currentBankState = RowActive;
				bankStates[rank][bank].lastCommand = ACTIVATE;
				bankStates[rank][bank].openRowAddress = poppedBusPacket->row;
				bankStates[rank][bank].nextActivate = max(currentClockCycle + tRC, bankStates[rank][bank].nextActivate);
				bankStates[rank][bank].nextPrecharge = max(currentClockCycle + tRAS, bankStates[rank][bank].nextPrecharge);

				//if we are using posted-CAS, the next column access can be sooner than normal operation

				bankStates[rank][bank].nextRead = max(currentClockCycle + (tRCD-AL), bankStates[rank][bank].nextRead);
				bankStates[rank][bank].nextWrite = max(currentClockCycle + (tRCD-AL), bankStates[rank][bank].nextWrite);

				for (size_t i=0;i<NUM_BANKS;i++)
				{
					if (i!=poppedBusPacket->bank)
					{
						bankStates[rank][i].nextActivate = max(currentClockCycle + tRRD, bankStates[rank][i].nextActivate);
					}
				}

				break;
			case PRECHARGE:
				bankStates[rank][bank].currentBankState = Precharging;
				bankStates[rank][bank].lastCommand = PRECHARGE;
				bankStates[rank][bank].stateChangeCountdown = tRP;
				bankStates[rank][bank].nextActivate = max(currentClockCycle + tRP, bankStates[rank][bank].nextActivate);

				break;
			case REFRESH:
				//add energy to account for total
				if (DEBUG_POWER)
				{
					PRINT(" ++ Adding Refresh energy to total energy");
				}
				refreshEnergy[rank] += (IDD5 - IDD3N) * tRFC * NUM_DEVICES;

				for (size_t i=0;i<NUM_BANKS;i++)
				{
					bankStates[rank][i].nextActivate = currentClockCycle + tRFC;
					bankStates[rank][i].currentBankState = Refreshing;
					bankStates[rank][i].lastCommand = REFRESH;
					bankStates[rank][i].stateChangeCountdown = tRFC;
				}

				break;
			default:
				ERROR("== Error - Popped a command we shouldn't have of type : " << poppedBusPacket->busPacketType);
				exit(0);
		}

		//issue on bus and print debug
		if (DEBUG_BUS)
		{
			PRINTN(" -- MC Issuing On Command Bus : ");
			poppedBusPacket->print();
		}

		//check for collision on bus
		if (outgoingCmdPacket != NULL)
		{
			ERROR("== Error - Command Bus Collision");
			exit(-1);
		}
		outgoingCmdPacket = poppedBusPacket;
		cmdCyclesLeft = tCMD;

	}

	if (protection == Regular || protection == FixedService_Channel || protection == FixedRate) {
		for (size_t i=0;i<transactionQueue.size();i++)
		{
			//pop off top transaction from queue
			//
			//	assuming simple scheduling at the moment
			//	will eventually add policies here
			Transaction *transaction = transactionQueue[i];

			//map address to rank,bank,row,col
			unsigned newTransactionChan, newTransactionRank, newTransactionBank, newTransactionRow, newTransactionColumn;

			// pass these in as references so they get set by the addressMapping function
			addressMapping(transaction->address, newTransactionChan, newTransactionRank, newTransactionBank, newTransactionRow, newTransactionColumn);

            if (SINGLE_BANK) newTransactionBank = 0;

			//if we have room, break up the transaction into the appropriate commands
			//and add them to the command queue
			if (commandQueue.hasRoomFor(2, newTransactionRank, newTransactionBank))
			{
				if (DEBUG_ADDR_MAP) 
				{
					PRINTN("== New Transaction - Mapping Address [0x" << hex << transaction->address << dec << "]");
					if (transaction->transactionType == DATA_READ) 
					{
						PRINT(" (Read)");
					}
					else
					{
						PRINT(" (Write)");
					}
					PRINT("  Rank : " << newTransactionRank);
					PRINT("  Bank : " << newTransactionBank);
					PRINT("  Row  : " << newTransactionRow);
					PRINT("  Col  : " << newTransactionColumn);
					PRINT("  Domain  : " << transaction->securityDomain);
					PRINT("  Time  : " << currentClockCycle);
				}



				//now that we know there is room in the command queue, we can remove from the transaction queue
				transactionQueue.erase(transactionQueue.begin()+i);

				//create activate command to the row we just translated
				BusPacket *ACTcommand = new BusPacket(ACTIVATE, transaction->address,
						newTransactionColumn, newTransactionRow, newTransactionRank,
						newTransactionBank, 0, transaction->isFake, transaction->securityDomain, dramsim_log);

				//create read or write command and enqueue it
				BusPacketType bpType = transaction->getBusPacketType();
				BusPacket *command = new BusPacket(bpType, transaction->address,
						newTransactionColumn, newTransactionRow, newTransactionRank,
						newTransactionBank, transaction->data, transaction->isFake, transaction->securityDomain, dramsim_log);



				commandQueue.enqueue(ACTcommand);
				commandQueue.enqueue(command);

				// If we have a read, save the transaction so when the data comes back
				// in a bus packet, we can staple it back into a transaction and return it
				if (transaction->transactionType == DATA_READ)
				{
					pendingReadTransactions.push_back(transaction);
				}
				else
				{
					// just delete the transaction now that it's a buspacket
					delete transaction; 
				}
				/* only allow one transaction to be scheduled per cycle -- this should
				* be a reasonable assumption considering how much logic would be
				* required to schedule multiple entries per cycle (parallel data
				* lines, switching logic, decision logic)
				*/
				break;
			}
			else // no room, do nothing this cycle
			{
				//PRINT( "== Warning - No room in command queue" << endl;
			}
		}
	} else if (protection == DAG) {

		// First, check if we have anything scheduled
		int scheduledBank = -1;
		int scheduledNode, scheduledDomain;

		if (scheduleNode.count(currentClockCycle)) {
			if (DEBUG_DEFENCE) PRINT("Executing scheduled node\n");
			// Determine the scheduled defence node's information
			scheduledNode = scheduleNode[currentClockCycle];
			scheduledDomain = scheduleDomain[currentClockCycle];

			int dataID = dataIDArr[scheduledDomain];
			int instID = instIDArr[scheduledDomain];

			int oldDataID = -100;
			int oldInstID = -100;

			if(oldDataIDArr.size() > scheduledDomain) {
				oldDataID = oldDataIDArr[scheduledDomain];
				oldInstID = oldInstIDArr[scheduledDomain];
			}
			if (DEBUG_DEFENCE) PRINT("currloop" << to_string(currentLoop[scheduledDomain]) << " curcycle " << currentClockCycle << " transqueue " << transactionQueue.size()) ;
			scheduledBank = this->dag[scheduledDomain][to_string(currentLoop[scheduledDomain])]["node"][scheduledNode]["bankID"];
			Transaction *transaction;

			Transaction *readTransaction;
			int readID = -1;

			Transaction *writeTransaction;
			int writeID = -1;

			int writeRequested = this->dag[scheduledDomain][to_string(currentLoop[scheduledDomain])]["node"][scheduledNode]["combinedWB"];
			int writeBank = this->dag[scheduledDomain][to_string(currentLoop[scheduledDomain])]["node"][scheduledNode]["combinedWBBankID"];
			
			unsigned newTransactionChan, newTransactionRank, newTransactionBank, newTransactionRow, newTransactionColumn;

			// Search the defence queue for a match...
			for (size_t i=0; i<defenceQueue.size(); i++) {
				transaction = defenceQueue[i];

				if (transaction->securityDomain != dataID && transaction->securityDomain != instID && transaction->securityDomain != oldDataID && transaction->securityDomain != oldInstID) continue;

				addressMapping(transaction->address, newTransactionChan, newTransactionRank, newTransactionBank, newTransactionRow, newTransactionColumn);

				if (SINGLE_BANK) newTransactionBank = 0;

				// First we need to find a read
				if (transaction->transactionType == DATA_READ && readID == -1 && scheduledBank == newTransactionBank) {
					readTransaction = transaction;
					readID = i;
                    defenceQueue.erase(defenceQueue.begin()+readID);
                    i--;
				} 
				else if (transaction->transactionType == DATA_WRITE && writeID == -1 && writeRequested && writeBank == newTransactionBank) {
					writeTransaction = transaction;
					writeID = i;
                	defenceQueue.erase(defenceQueue.begin()+writeID);
                    i--;
				}
				else continue;

				transaction->nodeID = scheduledNode;

				if ((readID != -1) && (writeID != -1 || !writeRequested)) break;

			}

			if (readID == -1) {
				if(DEBUG_DEFENCE) PRINT("No matching read transaction, enqueuing fake request")

				totalFakeReadRequests[scheduledDomain]++;
				readTransaction = new Transaction(DATA_READ, 0, nullptr, dataID, scheduledNode, true, scheduledBank);
				readTransaction->timeAdded = currentClockCycle;
			} 
			transactionQueue.push_back(readTransaction);

			if(writeRequested) {
				if (writeID == -1) {
					if(DEBUG_DEFENCE) PRINT("No matching write transaction, enqueuing fake request")

					totalFakeWriteRequests[scheduledDomain]++;                    
					writeTransaction = new Transaction(DATA_WRITE, 0, nullptr, dataID, scheduledNode, true, writeBank);
					writeTransaction->timeAdded = currentClockCycle;
				}

				transactionQueue.push_back(writeTransaction);
			}


		}

		for (size_t i=0;i<transactionQueue.size();i++)
		{
			Transaction *transaction;
			unsigned newTransactionChan, newTransactionRank, newTransactionBank, newTransactionRow, newTransactionColumn;
			//pop off top transaction from queue
			//
			//	assuming simple scheduling at the moment
			//	will eventually add policies here
			transaction = transactionQueue[i];

			//map address to rank,bank,row,col

			// pass these in as references so they get set by the addressMapping function
			addressMapping(transaction->address, newTransactionChan, newTransactionRank, newTransactionBank, newTransactionRow, newTransactionColumn);

			if (SINGLE_BANK) newTransactionBank = 0;
			else if (transaction->isFake) newTransactionBank = transaction->fakeBank;
                        //PRINT("index " << i << " contains address" << transaction->address);

			// If we have a request scheduled, try to match the bank with a transaction in the queue

			//if we have room, break up the transaction into the appropriate commands
			//and add them to the command queue
			if (commandQueue.hasRoomFor(2, newTransactionRank, newTransactionBank))
			{
				if (DEBUG_ADDR_MAP) 
				{
					PRINTN("== New Transaction - Mapping Address [0x" << hex << transaction->address << dec << "]");
					if (transaction->transactionType == DATA_READ) 
					{
						PRINT(" (Read)");
					}
					else
					{
						PRINT(" (Write)");
					}
					PRINT("  Rank : " << newTransactionRank);
					PRINT("  Bank : " << newTransactionBank);
					PRINT("  Row  : " << newTransactionRow);
					PRINT("  Col  : " << newTransactionColumn);
					PRINT("  Domain  : " << transaction->securityDomain);
					PRINT("  Time  : " << currentClockCycle);
					PRINT("  Fake? : " << transaction->isFake);
				}

				//now that we know there is room in the command queue, we can remove from the transaction queue
				transactionQueue.erase(transactionQueue.begin()+i);

				//create activate command to the row we just translated
				BusPacket *ACTcommand = new BusPacket(ACTIVATE, transaction->address,
						newTransactionColumn, newTransactionRow, newTransactionRank,
						newTransactionBank, 0, transaction->isFake, transaction->securityDomain, dramsim_log);

				//create read or write command and enqueue it
				BusPacketType bpType = transaction->getBusPacketType();
				BusPacket *command = new BusPacket(bpType, transaction->address,
						newTransactionColumn, newTransactionRow, newTransactionRank,
						newTransactionBank, transaction->data, transaction->isFake, transaction->securityDomain, dramsim_log);

				commandQueue.enqueue(ACTcommand);
				commandQueue.enqueue(command);

				// If we have a read, save the transaction so when the data comes back
				// in a bus packet, we can staple it back into a transaction and return it
				if (transaction->transactionType == DATA_READ)
				{
					pendingReadTransactions.push_back(transaction);
				}
				else
				{
					// just delete the transaction now that it's a buspacket
					delete transaction; 
				}
				/* only allow one transaction to be scheduled per cycle -- this should
				* be a reasonable assumption considering how much logic would be
				* required to schedule multiple entries per cycle (parallel data
				* lines, switching logic, decision logic)
				*/
				break;
			}
			else // no room, do nothing this cycle
			{
				//PRINT( "== Warning - No room in command queue" << endl;
			}
		}
	}
	else {

		int skip = 1;
		if (protection == FixedService_Rank && currentClockCycle % 7 == 0) {
			skip = 0;
		} else if (protection == FixedService_Bank && currentClockCycle % 15 == 0) {
			skip = 0;
		} else if (protection == FixedService_BTA && currentClockCycle % 43 == 0 && SINGLE_BANK) {
			skip = 0;
		} else if (protection == FixedService_BTA && currentClockCycle % 15 == 0 && !SINGLE_BANK) {
			skip = 0;
		}
		
		if (!skip) {
			// Search for transaction we can issue
			currentDomain = (currentDomain + 1) % NUM_DOMAINS;
			BTAPhase = (BTAPhase + 1) % 3;
                        
                        // Speculatively increment the fake FS counter (decrement it later if we were wrong) 
                        numFakeFS++;
			
                        //if (!SINGLE_BANK) PRINT("BTA PHASE: " << BTAPhase);
                        //PRINT("CURRENT DOMAIN: " << currentDomain);

			for (size_t i=0;i<transactionQueue.size();i++)
			{
				//pop off top transaction from queue
				//
				//	assuming simple scheduling at the moment
				//	will eventually add policies here
				Transaction *transaction = transactionQueue[i];

				//map address to rank,bank,row,col
				unsigned newTransactionChan, newTransactionRank, newTransactionBank, newTransactionRow, newTransactionColumn;

				// pass these in as references so they get set by the addressMapping function
				addressMapping(transaction->address, newTransactionChan, newTransactionRank, newTransactionBank, newTransactionRow, newTransactionColumn);

				if (SINGLE_BANK) newTransactionBank = 0;
				//Technically NUM_DOMAINS must be a power, but that's too hard to check.
				assert(NUM_DOMAINS % 2 == 0);

				if (!SINGLE_BANK) {
					if (newTransactionBank % 3 != BTAPhase) {
						continue;
					}
				}

				bool isSecure0 = !(dataIDArr.size() < 1) && (transaction->securityDomain == dataIDArr[0] || transaction->securityDomain == instIDArr[0]);
				bool isSecure1 = !(dataIDArr.size() < 2) && (transaction->securityDomain == dataIDArr[1] || transaction->securityDomain == instIDArr[1]);
				bool isSecure2 = !(dataIDArr.size() < 3) && (transaction->securityDomain == dataIDArr[2] || transaction->securityDomain == instIDArr[2]);
				bool isSecure3 = !(dataIDArr.size() < 4) && (transaction->securityDomain == dataIDArr[3] || transaction->securityDomain == instIDArr[3]);

				if (NUM_DOMAINS == 2) {
					if (currentDomain == 0 && !isSecure0) {
						continue;
					} else if (currentDomain == 1 && isSecure0) {
						continue;
					}
				} else if (NUM_DOMAINS == 4) {
					if (currentDomain == 0 && !isSecure0) {
						continue;
					} else if (currentDomain == 1 && !isSecure1) {
						continue;
					} else if (currentDomain > 1 && (isSecure0 || isSecure1)) {
						continue;
					}
				} else if (NUM_DOMAINS == 8) {
                                       if (currentDomain == 0 && !isSecure0) continue;
                                       else if (currentDomain == 1 && !isSecure1) continue;
                                       else if (currentDomain == 2 && !isSecure2) continue;
                                       else if (currentDomain == 3 && !isSecure3) continue;
                                       else if (currentDomain > 3 && (isSecure0 || isSecure1 || isSecure2 || isSecure3)) continue;
                                } else {
					assert(false);
				}

                                // If we've gotten this far, we'll issue. Thus, we're issuing a real request.
                                numFakeFS--;
				
				//if we have room, break up the transaction into the appropriate commands
				//and add them to the command queue
				if (commandQueue.hasRoomFor(2, newTransactionRank, newTransactionBank))
				{
					if (DEBUG_ADDR_MAP) 
					{
						PRINTN("== New Transaction - Mapping Address [0x" << hex << transaction->address << dec << "]");
						if (transaction->transactionType == DATA_READ) 
						{
							PRINT(" (Read)");
						}
						else
						{
							PRINT(" (Write)");
						}
						PRINT("  Protection Domain  : " << currentDomain);
						PRINT("  Time  : " << currentClockCycle);
						PRINT("  Rank : " << newTransactionRank);
						PRINT("  Bank : " << newTransactionBank);
						PRINT("  Row  : " << newTransactionRow);
						PRINT("  Col  : " << newTransactionColumn);
					}


					//now that we know there is room in the command queue, we can remove from the transaction queue
					transactionQueue.erase(transactionQueue.begin()+i);

					//create activate command to the row we just translated
					BusPacket *ACTcommand = new BusPacket(ACTIVATE, transaction->address,
							newTransactionColumn, newTransactionRow, newTransactionRank,
							newTransactionBank, 0, transaction->isFake, transaction->securityDomain, dramsim_log);

					//create read or write command and enqueue it
					BusPacketType bpType = transaction->getBusPacketType();
					BusPacket *command = new BusPacket(bpType, transaction->address,
							newTransactionColumn, newTransactionRow, newTransactionRank,
							newTransactionBank, transaction->data, transaction->isFake, transaction->securityDomain, dramsim_log);



					commandQueue.enqueue(ACTcommand);
					commandQueue.enqueue(command);

					// If we have a read, save the transaction so when the data comes back
					// in a bus packet, we can staple it back into a transaction and return it
					if (transaction->transactionType == DATA_READ)
					{
						pendingReadTransactions.push_back(transaction);
					}
					else
					{
						// just delete the transaction now that it's a buspacket
						delete transaction; 
					}
					/* only allow one transaction to be scheduled per cycle -- this should
					* be a reasonable assumption considering how much logic would be
					* required to schedule multiple entries per cycle (parallel data
					* lines, switching logic, decision logic)
					*/
					break;
				}
				else // no room, do nothing this cycle
				{
					PRINT( "== Warning - No room in command queue" << endl );
				}
			}

		}
	}


	//calculate power
	//  this is done on a per-rank basis, since power characterization is done per device (not per bank)
	for (size_t i=0;i<NUM_RANKS;i++)
	{
		if (USE_LOW_POWER)
		{
			//if there are no commands in the queue and that particular rank is not waiting for a refresh...
			if (commandQueue.isEmpty(i) && !(*ranks)[i]->refreshWaiting)
			{
				//check to make sure all banks are idle
				bool allIdle = true;
				for (size_t j=0;j<NUM_BANKS;j++)
				{
					if (bankStates[i][j].currentBankState != Idle)
					{
						allIdle = false;
						break;
					}
				}

				//if they ARE all idle, put in power down mode and set appropriate fields
				if (allIdle)
				{
					powerDown[i] = true;
					(*ranks)[i]->powerDown();
					for (size_t j=0;j<NUM_BANKS;j++)
					{
						bankStates[i][j].currentBankState = PowerDown;
						bankStates[i][j].nextPowerUp = currentClockCycle + tCKE;
					}
				}
			}
			//if there IS something in the queue or there IS a refresh waiting (and we can power up), do it
			else if (currentClockCycle >= bankStates[i][0].nextPowerUp && powerDown[i]) //use 0 since theyre all the same
			{
				powerDown[i] = false;
				(*ranks)[i]->powerUp();
				for (size_t j=0;j<NUM_BANKS;j++)
				{
					bankStates[i][j].currentBankState = Idle;
					bankStates[i][j].nextActivate = currentClockCycle + tXP;
				}
			}
		}

		//check for open bank
		bool bankOpen = false;
		for (size_t j=0;j<NUM_BANKS;j++)
		{
			if (bankStates[i][j].currentBankState == Refreshing ||
			        bankStates[i][j].currentBankState == RowActive)
			{
				bankOpen = true;
				break;
			}
		}

		//background power is dependent on whether or not a bank is open or not
		if (bankOpen)
		{
			if (DEBUG_POWER)
			{
				PRINT(" ++ Adding IDD3N to total energy [from rank "<< i <<"]");
			}
			backgroundEnergy[i] += IDD3N * NUM_DEVICES;
		}
		else
		{
			//if we're in power-down mode, use the correct current
			if (powerDown[i])
			{
				if (DEBUG_POWER)
				{
					PRINT(" ++ Adding IDD2P to total energy [from rank " << i << "]");
				}
				backgroundEnergy[i] += IDD2P * NUM_DEVICES;
			}
			else
			{
				if (DEBUG_POWER)
				{
					PRINT(" ++ Adding IDD2N to total energy [from rank " << i << "]");
				}
				backgroundEnergy[i] += IDD2N * NUM_DEVICES;
			}
		}
	}

	//check for outstanding data to return to the CPU
	if (returnTransaction.size()>0)
	{
		if (DEBUG_BUS)
		{
			PRINTN(" -- MC Issuing to CPU bus : " << *returnTransaction[0]);
		}
		totalTransactions++;

		bool foundMatch=false;
		//find the pending read transaction to calculate latency
		for (size_t i=0;i<pendingReadTransactions.size();i++)
		{
			if (pendingReadTransactions[i]->address == returnTransaction[0]->address)
			{
				//if(currentClockCycle - pendingReadTransactions[i]->timeAdded > 2000)
				//	{
				//		pendingReadTransactions[i]->print();
				//		exit(0);
				//	}
				unsigned chan,rank,bank,row,col;
				addressMapping(returnTransaction[0]->address,chan,rank,bank,row,col);
				if(!pendingReadTransactions[i]->isFake) {
					insertHistogram(currentClockCycle-pendingReadTransactions[i]->timeAdded,rank,bank);
					//return latency
					returnReadData(pendingReadTransactions[i]);
				}

				int currDomain = -1;

				if (revData.count(pendingReadTransactions[i]->securityDomain)) currDomain = revData[pendingReadTransactions[i]->securityDomain];
				else if (revInst.count(pendingReadTransactions[i]->securityDomain)) currDomain = revInst[pendingReadTransactions[i]->securityDomain];
				else if (revOldData.count(pendingReadTransactions[i]->securityDomain)) currDomain = revOldData[pendingReadTransactions[i]->securityDomain];
				else if (revOldInst.count(pendingReadTransactions[i]->securityDomain)) currDomain = revOldInst[pendingReadTransactions[i]->securityDomain];

				if (protection == DAG && currDomain != -1) {
					if (DEBUG_DEFENCE) PRINT("Finished Transaction " << hex << pendingReadTransactions[i]->address << "(node " << pendingReadTransactions[i]->nodeID << " at time " << dec << currentClockCycle << " in domain " << currDomain);

					// Update phase information
					int loopID = currentLoop[currDomain];
					finishTimes[currDomain][loopID][pendingReadTransactions[i]->nodeID] = currentClockCycle;

					totalNodes[currDomain]++;
					if (pendingReadTransactions[i]->nodeID == this->dag[currDomain][to_string(loopID)]["node"].size() - 1) {
						// We've reached the end of the loop! 
						// Check if we're repeating the section, or starting a new one.
						if (currentLoopIteration[currDomain]+1 == this->dag[currDomain][to_string(loopID)]["loop"]) {
							// We're done here, move to next block
							if (DEBUG_DEFENCE) PRINT("Finished loop body, moving to loop " << (currentLoop[currDomain] + 1) % this->dag[currDomain].size());
							currentLoop[currDomain] = (currentLoop[currDomain] + 1) % this->dag[currDomain].size();
							currentLoopIteration[currDomain] = 0;
						} else {
							// We're looping!
							if (DEBUG_DEFENCE) PRINT("Looping!");
							currentLoopIteration[currDomain]++;
							// If we haven't looped before, we'll have to set our target.
							if (childrenList[currDomain][loopID][pendingReadTransactions[i]->nodeID].size() == 0) {
								childrenList[currDomain][loopID][pendingReadTransactions[i]->nodeID].push_back(0);
							}

						}

						for (auto& node : this->dag[currDomain][to_string(loopID)]["node"].items()) {
							finishTimes[currDomain][loopID][node.value()["nodeID"]] = std::numeric_limits<uint64_t>::max();
						}

					} 
					// Check all children
					for (auto& child : childrenList[currDomain][loopID][pendingReadTransactions[i]->nodeID]) {
						// If all parents of the child are complete, we can issue it!
						bool ready = true;
						for (auto& parent : parentList[currDomain][loopID][child]) {
							if (DEBUG_DEFENCE) PRINT("Parent: " << parent << " Child: " << child);

							if (finishTimes[currDomain][loopID][parent] > currentClockCycle) {
								if (DEBUG_DEFENCE) PRINT("NOT READY!");
								ready = false;
								break;
							}							
						}

						if (ready) {
							int edgeWeight = weightList[currDomain][loopID][pendingReadTransactions[i]->nodeID][child]/DEF_CLK_DIV;
							int scheduledTime = edgeWeight + currentClockCycle;

                            if (scheduledTime == currentClockCycle) scheduledTime++;
							while (scheduleNode.count(scheduledTime) > 0) scheduledTime++;
							scheduleNode[scheduledTime] = child;
							scheduleDomain[scheduledTime] = currDomain;
							if (DEBUG_DEFENCE) PRINT("Issuing new node " << child << " at time" << scheduledTime);

						}
					}

				}

				delete pendingReadTransactions[i];
				pendingReadTransactions.erase(pendingReadTransactions.begin()+i);
				foundMatch=true; 


				break;
			}
		}
		if (!foundMatch)
		{
			ERROR("Can't find a matching transaction for 0x"<<hex<<returnTransaction[0]->address<<dec);
			abort(); 
		}
		delete returnTransaction[0];
		returnTransaction.erase(returnTransaction.begin());
	}

	//decrement refresh counters
	for (size_t i=0;i<NUM_RANKS;i++)
	{
		refreshCountdown[i]--;
	}

	//
	//print debug
	//
	if (DEBUG_TRANS_Q)
	{
		PRINT("== Printing transaction queue");
		for (size_t i=0;i<transactionQueue.size();i++)
		{
			PRINTN("  " << i << "] "<< *transactionQueue[i]);
		}
	}

	if (DEBUG_BANKSTATE)
	{
		//TODO: move this to BankState.cpp
		PRINT("== Printing bank states (According to MC)");
		for (size_t i=0;i<NUM_RANKS;i++)
		{
			for (size_t j=0;j<NUM_BANKS;j++)
			{
				if (bankStates[i][j].currentBankState == RowActive)
				{
					PRINTN("[" << bankStates[i][j].openRowAddress << "] ");
				}
				else if (bankStates[i][j].currentBankState == Idle)
				{
					PRINTN("[idle] ");
				}
				else if (bankStates[i][j].currentBankState == Precharging)
				{
					PRINTN("[pre] ");
				}
				else if (bankStates[i][j].currentBankState == Refreshing)
				{
					PRINTN("[ref] ");
				}
				else if (bankStates[i][j].currentBankState == PowerDown)
				{
					PRINTN("[lowp] ");
				}
			}
			PRINT(""); // effectively just cout<<endl;
		}
	}

	if (DEBUG_CMD_Q)
	{
		commandQueue.print();
	}

	commandQueue.step();

}

bool MemoryController::WillAcceptTransaction()
{
	return transactionQueue.size() < TRANS_QUEUE_DEPTH;
}

bool MemoryController::WillAcceptDefenceTransaction()
{
	return defenceQueue.size() < DEFENCE_QUEUE_DEPTH;
}

//allows outside source to make request of memory system
bool MemoryController::addTransaction(Transaction *trans)
{
	if (DEBUG_DEFENCE) PRINT("NEWTRANS: Addr: " << std::hex << trans->address << " Clk: " << std::dec << currentClockCycle << " Domain: " << trans->securityDomain << " isWrite? " << (trans->transactionType == DATA_WRITE) << " Current Cycle: " << currentClockCycle);

	if (protection == DAG && (revData.count(trans->securityDomain) || revInst.count(trans->securityDomain))) {
    	        if (DEBUG_DEFENCE) PRINT("PUSHED!")
		defenceQueue.push_back(trans);
		return true;
	}

	if (WillAcceptTransaction())
	{
		trans->timeAdded = currentClockCycle;
		transactionQueue.push_back(trans);
		return true;
	}
	else 
	{
		return false;
	}
}

void MemoryController::resetStats()
{
	for (size_t i=0; i<NUM_RANKS; i++)
	{
		for (size_t j=0; j<NUM_BANKS; j++)
		{
			//XXX: this means the bank list won't be printed for partial epochs
			grandTotalBankAccesses[SEQUENTIAL(i,j)] += totalReadsPerBank[SEQUENTIAL(i,j)] + totalWritesPerBank[SEQUENTIAL(i,j)];
			totalReadsPerBank[SEQUENTIAL(i,j)] = 0;
			totalWritesPerBank[SEQUENTIAL(i,j)] = 0;
			totalEpochLatency[SEQUENTIAL(i,j)] = 0;
		}

		burstEnergy[i] = 0;
		actpreEnergy[i] = 0;
		refreshEnergy[i] = 0;
		backgroundEnergy[i] = 0;
		totalReadsPerRank[i] = 0;
		totalWritesPerRank[i] = 0;
	}
}
//prints statistics at the end of an epoch or  simulation
void MemoryController::printStats(bool finalStats)
{
	//unsigned myChannel = parentMemorySystem->systemID;

	//if we are not at the end of the epoch, make sure to adjust for the actual number of cycles elapsed

	uint64_t cyclesElapsed = (currentClockCycle % EPOCH_LENGTH == 0) ? EPOCH_LENGTH : currentClockCycle % EPOCH_LENGTH;
	unsigned bytesPerTransaction = (JEDEC_DATA_BUS_BITS*BL)/8;
	uint64_t totalBytesTransferred = totalTransactions * bytesPerTransaction;
	double secondsThisEpoch = (double)cyclesElapsed * tCK * 1E-9;

	// only per rank
	vector<double> backgroundPower = vector<double>(NUM_RANKS,0.0);
	vector<double> burstPower = vector<double>(NUM_RANKS,0.0);
	vector<double> refreshPower = vector<double>(NUM_RANKS,0.0);
	vector<double> actprePower = vector<double>(NUM_RANKS,0.0);
	vector<double> averagePower = vector<double>(NUM_RANKS,0.0);

	// per bank variables
	vector<double> averageLatency = vector<double>(NUM_RANKS*NUM_BANKS,0.0);
	vector<double> bandwidth = vector<double>(NUM_RANKS*NUM_BANKS,0.0);

	double totalBandwidth=0.0;
	for (size_t i=0;i<NUM_RANKS;i++)
	{
		for (size_t j=0; j<NUM_BANKS; j++)
		{
			bandwidth[SEQUENTIAL(i,j)] = (((double)(totalReadsPerBank[SEQUENTIAL(i,j)]+totalWritesPerBank[SEQUENTIAL(i,j)]) * (double)bytesPerTransaction)/(1024.0*1024.0*1024.0)) / secondsThisEpoch;
			averageLatency[SEQUENTIAL(i,j)] = ((float)totalEpochLatency[SEQUENTIAL(i,j)] / (float)(totalReadsPerBank[SEQUENTIAL(i,j)])) * tCK;
			totalBandwidth+=bandwidth[SEQUENTIAL(i,j)];
			totalReadsPerRank[i] += totalReadsPerBank[SEQUENTIAL(i,j)];
			totalWritesPerRank[i] += totalWritesPerBank[SEQUENTIAL(i,j)];
		}
	}
#ifdef LOG_OUTPUT
	dramsim_log.precision(3);
	dramsim_log.setf(ios::fixed,ios::floatfield);
#else
	cout.precision(3);
	cout.setf(ios::fixed,ios::floatfield);
#endif

	PRINT( " =======================================================" );
	PRINT( " ============== Printing Statistics [id:"<<parentMemorySystem->systemID<<"]==============" );
	PRINTN( "   Total Return Transactions : " << totalTransactions );
	PRINT( " ("<<totalBytesTransferred <<" bytes) aggregate average bandwidth "<<totalBandwidth<<"GB/s");

	//PRINT(" ========== Defence DAG Statistics ========== ");
	//PRINT("\nFinal Defence Nodes Executed: " << std::dec << totalNodes << ",\nNumber of Fake Read Requests: " << totalFakeReadRequests << ",\nNumber of Fake Write Requests: " << totalFakeWriteRequests);

	if (finalStats && protection == DAG && VIS_FILE_OUTPUT) {
                csvOut.getOutputStream() << "Total Bytes Transferred: " << totalBytesTransferred << "\n";
                csvOut.getOutputStream() << "Aggregate Average Bandwidth (GB/s): " << totalBandwidth << "\n";
		for (int i = 0; i < dataIDArr.size(); i++) {
			csvOut.getOutputStream() << "\nDefence Group: " << i << std::dec << ",\nFinal Defence Nodes Executed: " << totalNodes[i] << ",\nNumber of Fake Read Requests: " << totalFakeReadRequests[i] << ",\nNumber of Fake Write Requests: " << totalFakeWriteRequests[i];
		}
	} else if (finalStats && protection == FixedService_BTA && VIS_FILE_OUTPUT) {
          csvOut.getOutputStream() << "\n Fake FS requests: " << numFakeFS;
        }

#ifdef LOG_OUTPUT
	dramsim_log.flush();
#endif

	resetStats();
}
MemoryController::~MemoryController()
{
	//ERROR("MEMORY CONTROLLER DESTRUCTOR");
	//abort();
	for (size_t i=0; i<pendingReadTransactions.size(); i++)
	{
		delete pendingReadTransactions[i];
	}
	for (size_t i=0; i<returnTransaction.size(); i++)
	{
		delete returnTransaction[i];
	}

}
//inserts a latency into the latency histogram
void MemoryController::insertHistogram(unsigned latencyValue, unsigned rank, unsigned bank)
{
	totalEpochLatency[SEQUENTIAL(rank,bank)] += latencyValue;
	//poor man's way to bin things.
	latencies[(latencyValue/HISTOGRAM_BIN_SIZE)*HISTOGRAM_BIN_SIZE]++;
}
