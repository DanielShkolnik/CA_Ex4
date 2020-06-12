/* 046267 Computer Architecture - Spring 2020 - HW #4 */

#include "core_api.h"
#include "sim_api.h"

#include <stdio.h>
#include <iostream>

class ThreadStatus{
public:
    bool isFinished;
    int idleUntilCycle;
    uint32_t nextLine;

    ThreadStatus():isFinished(false),idleUntilCycle(-1),nextLine(0){};
    //ThreadStatus(const ThreadStatus &threadStatus);
    //ThreadStatus &operator=(const ThreadStatus &threadStatus);
    //~ThreadStatus() = default;
};


class CoreMT{
public:
    bool isBlockedMT;
    int numOfThreads;
    int currentThread;
    int numOfCycles;
    int loadLatency;
    int storeLatency;
    int switchOverhead;
    int numOfInts;
    ThreadStatus* threads;
    tcontext* regFiles;


    CoreMT(bool isBlockedMT):isBlockedMT(isBlockedMT),numOfThreads(SIM_GetThreadsNum()),
                                            currentThread(0),numOfCycles(0),loadLatency(SIM_GetLoadLat()),
                                            storeLatency(SIM_GetStoreLat()),switchOverhead(SIM_GetSwitchCycles()),numOfInts(0)
                                            ,threads(NULL) ,regFiles(NULL){
        this->threads = new ThreadStatus[this->numOfThreads];
        this->regFiles = new tcontext[this->numOfThreads];
        for(int i=0; i<this->numOfThreads; i++){
            for(int j=0; j<REGS_COUNT; j++){
                this->regFiles[i].reg[j]=0;
            }
        }

    };
    //CoreMT(const CoreMT &coreMT) = delete;
    //CoreMT &operator=(const CoreMT &coreMT) = delete;
    ~CoreMT(){
        delete[] this->threads;
        delete[] this->regFiles;
    };

    int findNextThreadBlockedMT(int currThread){
        bool allThreadFinished = true;
        int nextThread = currThread;
        int counter = 0;
        while(counter < this->numOfThreads){
            if(!this->threads[nextThread].isFinished && this->numOfCycles > this->threads[nextThread].idleUntilCycle){
                return nextThread;
            }
            if(!this->threads[nextThread].isFinished) allThreadFinished = false;
            nextThread = (nextThread + 1) % (this->numOfThreads);
            counter++;
        }
        if(allThreadFinished) return -2;
        else return -1; //all threads are idle

    }


    int findNextThreadFinegrainedMT(int currThread){
        bool allThreadFinished = true;
        int nextThread =  (currThread + 1) % (this->numOfThreads);
        int counter = 0;
        while(counter < this->numOfThreads){
            if(!this->threads[nextThread].isFinished && this->numOfCycles > this->threads[nextThread].idleUntilCycle){
                return nextThread;
            }
            if(!this->threads[nextThread].isFinished) allThreadFinished = false;
            nextThread = (nextThread + 1) % (this->numOfThreads);
            counter++;
        }
        if(allThreadFinished) return -2;
        else return -1; //all threads are idle

    }


    void runBlockedMT(){
        int currThread = 0;
        bool isEvent = false;
        bool isIdle = false;
        bool allThreadsFinishd = false;
        while(!allThreadsFinishd){
            //std::cout << this->numOfCycles << std::endl;
            if(!isIdle) {
                isEvent = false;
                Instruction currInst;
                SIM_MemInstRead(this->threads[currThread].nextLine, &currInst, currThread);
                int opc = currInst.opcode;
                switch (opc) {
                    case CMD_NOP: // NOP
                        break;
                    case CMD_ADDI:
                        this->regFiles[currThread].reg[currInst.dst_index] =
                                this->regFiles[currThread].reg[currInst.src1_index] + currInst.src2_index_imm;
                        break;
                    case CMD_SUBI:
                        this->regFiles[currThread].reg[currInst.dst_index] =
                                this->regFiles[currThread].reg[currInst.src1_index] - currInst.src2_index_imm;
                        break;
                    case CMD_ADD:
                        this->regFiles[currThread].reg[currInst.dst_index] =
                                this->regFiles[currThread].reg[currInst.src1_index] +
                                this->regFiles[currThread].reg[currInst.src2_index_imm];
                        break;
                    case CMD_SUB:
                        this->regFiles[currThread].reg[currInst.dst_index] =
                                this->regFiles[currThread].reg[currInst.src1_index] -
                                this->regFiles[currThread].reg[currInst.src2_index_imm];
                        break;
                    case CMD_LOAD:
                        if (currInst.isSrc2Imm) {
                            //std::cout << "Load addr " << (uint32_t) this->regFiles[currThread].reg[currInst.src1_index] + currInst.src2_index_imm << std::endl;
                            SIM_MemDataRead(
                                    (uint32_t) this->regFiles[currThread].reg[currInst.src1_index] + currInst.src2_index_imm,
                                    &(this->regFiles[currThread].reg[currInst.dst_index]));
                        } else {
                            //std::cout << "Load addr " << (uint32_t) this->regFiles[currThread].reg[currInst.src1_index] + this->regFiles[currThread].reg[currInst.src2_index_imm] << std::endl;
                            SIM_MemDataRead((uint32_t) this->regFiles[currThread].reg[currInst.src1_index] +
                                            this->regFiles[currThread].reg[currInst.src2_index_imm],
                                            &(this->regFiles[currThread].reg[currInst.dst_index]));
                        }
                        this->threads[currThread].idleUntilCycle = this->numOfCycles + this->loadLatency;
                        isEvent = true;
                        break;
                    case CMD_STORE:
                        if (currInst.isSrc2Imm) {
                            //std::cout << "Store addr " << (uint32_t) this->regFiles[currThread].reg[currInst.dst_index] + currInst.src2_index_imm<< "  Store value " << this->regFiles[currThread].reg[currInst.src1_index] << std::endl;
                            SIM_MemDataWrite(
                                    (uint32_t) this->regFiles[currThread].reg[currInst.dst_index] +
                                    currInst.src2_index_imm,
                                    this->regFiles[currThread].reg[currInst.src1_index]);
                        } else {
                            //std::cout << "Store addr " << (uint32_t) this->regFiles[currThread].reg[currInst.dst_index] + this->regFiles[currThread].reg[currInst.src2_index_imm]<< "  Store value " << this->regFiles[currThread].reg[currInst.src1_index] << std::endl;
                            SIM_MemDataWrite((uint32_t) this->regFiles[currThread].reg[currInst.dst_index] +
                                            this->regFiles[currThread].reg[currInst.src2_index_imm],
                                            this->regFiles[currThread].reg[currInst.src1_index]);
                        }
                        this->threads[currThread].idleUntilCycle = this->numOfCycles + this->storeLatency;
                        isEvent = true;
                        break;
                    case CMD_HALT:
                        this->threads[currThread].isFinished = true;
                        isEvent = true;
                        break;
                }
            }
            if(!isIdle) this->threads[currThread].nextLine++;
            this->numOfCycles ++;
            if(!isIdle) this->numOfInts++;

            if(isEvent || isIdle) {
                int nextThread = this->findNextThreadBlockedMT(currThread);
                //std::cout << "nextThread " << nextThread << std::endl;
                if(nextThread == -2) allThreadsFinishd = true;
                else {
                    if (nextThread == -1)isIdle = true;
                    else isIdle = false;
                    if (!isIdle && currThread != nextThread)this->numOfCycles += this->switchOverhead;
                    if (!isIdle) currThread = nextThread;
                }
            }
            /*
            printf("\n---- Blocked MT Simulation ----\n");
            for(int k=0; k<this->numOfThreads; k++){
                printf("\nRegister file thread id %d:\n", k);
                for (int i=0; i<REGS_COUNT; ++i)
                    printf("\tR%d = 0x%X", i, this->regFiles[k].reg[i]);
                 }*/
        }
    }



    void runFinegrainedMT(){
        int currThread = 0;
        bool isIdle = false;
        bool allThreadsFinishd = false;
        while(!allThreadsFinishd){
            if(!isIdle) {
                Instruction currInst;
                SIM_MemInstRead(this->threads[currThread].nextLine, &currInst, currThread);
                int opc = currInst.opcode;
                switch (opc) {
                    case CMD_NOP: // NOP
                        break;
                    case CMD_ADDI:
                        this->regFiles[currThread].reg[currInst.dst_index] =
                                this->regFiles[currThread].reg[currInst.src1_index] + currInst.src2_index_imm;
                        break;
                    case CMD_SUBI:
                        this->regFiles[currThread].reg[currInst.dst_index] =
                                this->regFiles[currThread].reg[currInst.src1_index] - currInst.src2_index_imm;
                        break;
                    case CMD_ADD:
                        this->regFiles[currThread].reg[currInst.dst_index] =
                                this->regFiles[currThread].reg[currInst.src1_index] +
                                this->regFiles[currThread].reg[currInst.src2_index_imm];
                        break;
                    case CMD_SUB:
                        this->regFiles[currThread].reg[currInst.dst_index] =
                                this->regFiles[currThread].reg[currInst.src1_index] -
                                this->regFiles[currThread].reg[currInst.src2_index_imm];
                        break;
                    case CMD_LOAD:
                        if (currInst.isSrc2Imm) {
                            //std::cout << "Load addr " << (uint32_t) this->regFiles[currThread].reg[currInst.src1_index] + currInst.src2_index_imm << std::endl;
                            SIM_MemDataRead(
                                    (uint32_t) this->regFiles[currThread].reg[currInst.src1_index] +
                                    currInst.src2_index_imm,
                                    &(this->regFiles[currThread].reg[currInst.dst_index]));
                        } else {
                            //std::cout << "Load addr " << (uint32_t) this->regFiles[currThread].reg[currInst.src1_index] + this->regFiles[currThread].reg[currInst.src2_index_imm] << std::endl;
                            SIM_MemDataRead((uint32_t) this->regFiles[currThread].reg[currInst.src1_index] +
                                            this->regFiles[currThread].reg[currInst.src2_index_imm],
                                            &(this->regFiles[currThread].reg[currInst.dst_index]));
                        }
                        this->threads[currThread].idleUntilCycle = this->numOfCycles + this->loadLatency;
                        break;
                    case CMD_STORE:
                        if (currInst.isSrc2Imm) {
                            //std::cout << "Store addr " << (uint32_t) this->regFiles[currThread].reg[currInst.dst_index] + currInst.src2_index_imm << std::endl;
                            SIM_MemDataWrite(
                                    (uint32_t) this->regFiles[currThread].reg[currInst.dst_index] +
                                    currInst.src2_index_imm,
                                    this->regFiles[currThread].reg[currInst.src1_index]);
                        } else {
                            //std::cout << "Store addr " << (uint32_t) this->regFiles[currThread].reg[currInst.dst_index] + this->regFiles[currThread].reg[currInst.src2_index_imm] << std::endl;
                            SIM_MemDataWrite((uint32_t) this->regFiles[currThread].reg[currInst.dst_index] +
                                            this->regFiles[currThread].reg[currInst.src2_index_imm],
                                            this->regFiles[currThread].reg[currInst.src1_index]);
                        }
                        this->threads[currThread].idleUntilCycle = this->numOfCycles + this->storeLatency;
                        break;
                    case CMD_HALT:
                        this->threads[currThread].isFinished = true;
                        break;
                }
            }
            if(!isIdle) this->threads[currThread].nextLine++;
            this->numOfCycles ++;
            if(!isIdle) this->numOfInts++;
            int nextThread = this->findNextThreadFinegrainedMT(currThread);
            if(nextThread == -2){
                allThreadsFinishd = true;
            }else {
                if (nextThread == -1)isIdle = true;
                else isIdle = false;
                if (!isIdle) currThread = nextThread;
            }
        }
    }
};


CoreMT* BlockedMT= NULL;
CoreMT* FinegrainedMT= NULL;

void CORE_BlockedMT() {
    BlockedMT = new CoreMT(true);
    BlockedMT->runBlockedMT();
}

void CORE_FinegrainedMT() {
    FinegrainedMT = new CoreMT(false);
    FinegrainedMT->runFinegrainedMT();
}

double CORE_BlockedMT_CPI(){
    if(BlockedMT == NULL) return 0;
    //std::cout << "numOfCycles " << BlockedMT->numOfCycles <<  " numOfInts " << BlockedMT->numOfInts << std::endl;
    double CPI = ((double)(BlockedMT->numOfCycles)/(BlockedMT->numOfInts));
    delete BlockedMT;
    return CPI;
}

double CORE_FinegrainedMT_CPI(){
    if(FinegrainedMT == NULL) return 0;
    double CPI = ((double)(FinegrainedMT->numOfCycles)/(BlockedMT->numOfInts));
    //std::cout << "numOfCycles " << FinegrainedMT->numOfCycles  << " numOfInts "  << BlockedMT->numOfInts << std::endl;
    delete BlockedMT;
    return CPI;
}

void CORE_BlockedMT_CTX(tcontext* context, int threadid) {
    if(BlockedMT == NULL) return;
    for(int i=0; i<REGS_COUNT; i++){
        context[threadid].reg[i] = BlockedMT->regFiles[threadid].reg[i];
    }
}

void CORE_FinegrainedMT_CTX(tcontext* context, int threadid) {
    if(FinegrainedMT == NULL) return;
    for(int i=0; i<REGS_COUNT; i++){
        context[threadid].reg[i] = FinegrainedMT->regFiles[threadid].reg[i];
    }
}
