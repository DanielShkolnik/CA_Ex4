/* 046267 Computer Architecture - Spring 2020 - HW #4 */

#include "core_api.h"
#include "sim_api.h"

#include <stdio.h>

class ThreadStatus{
public:
    bool isFinished;
    int idleUntilCycle;
    uint32_t nextLine;

    ThreadStatus():isFinished(false),idleUntilCycle(-1),nextLine(0){};
    ThreadStatus(const ThreadStatus &threadStatus) = delete;
    ThreadStatus &operator=(const ThreadStatus &threadStatus) = delete;
    ~ThreadStatus() = default;
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
    tcontext* regFile;


    CoreMT(bool isBlockedMT):isBlockedMT(isBlockedMT),numOfThreads(SIM_GetThreadsNum()),
                                            currentThread(0),numOfCycles(0),loadLatency(SIM_GetLoadLat()),
                                            storeLatency(SIM_GetStoreLat()),switchOverhead(SIM_GetSwitchCycles()),numOfInts(0)
                                            ,threads(nullptr) ,regFile(nullptr){
        this->threads = new ThreadStatus[this->numOfThreads];
        this->regFile = new tcontext;
        for(int i=0; i<REGS_COUNT; i++){
            this->regFile->reg[i]=0;
        }
    };
    CoreMT(const CoreMT &coreMT) = delete;
    CoreMT &operator=(const CoreMT &coreMT) = delete;
    ~CoreMT(){
        delete[] this->threads;
        delete[] this->regFile;
    };

    int findNextThread(int currThread){
        bool allThreadFinished = true;
        int nextThread = (currThread + 1) % (this->numOfThreads);
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
            if(!isIdle) {
                isEvent = false;
                Instruction currInst;
                SIM_MemInstRead(this->threads[currThread].nextLine, &currInst, currThread);
                int opc = currInst.opcode;
                switch (opc) {
                    case CMD_NOP: // NOP
                        break;
                    case CMD_ADDI:
                        this->regFile[currThread].reg[currInst.dst_index] =
                                this->regFile[currThread].reg[currInst.src1_index] + currInst.src2_index_imm;
                        break;
                    case CMD_SUBI:
                        this->regFile[currThread].reg[currInst.dst_index] =
                                this->regFile[currThread].reg[currInst.src1_index] - currInst.src2_index_imm;
                        break;
                    case CMD_ADD:
                        this->regFile[currThread].reg[currInst.dst_index] =
                                this->regFile[currThread].reg[currInst.src1_index] +
                                this->regFile[currThread].reg[currInst.src2_index_imm];
                        break;
                    case CMD_SUB:
                        this->regFile[currThread].reg[currInst.dst_index] =
                                this->regFile[currThread].reg[currInst.src1_index] -
                                this->regFile[currThread].reg[currInst.src2_index_imm];
                        break;
                    case CMD_LOAD:
                        if (currInst.isSrc2Imm) {
                            SIM_MemDataRead(
                                    (uint32_t) this->regFile[currThread].reg[currInst.src1_index] +
                                    currInst.src2_index_imm,
                                    &(this->regFile[currThread].reg[currInst.dst_index]));
                        } else {
                            SIM_MemDataRead((uint32_t) this->regFile[currThread].reg[currInst.src1_index] +
                                            this->regFile[currThread].reg[currInst.src2_index_imm],
                                            &(this->regFile[currThread].reg[currInst.dst_index]));
                        }
                        this->threads[currThread].idleUntilCycle = this->numOfCycles + this->loadLatency;
                        isEvent = true;
                        break;
                    case CMD_STORE:
                        if (currInst.isSrc2Imm) {
                            SIM_MemDataWrite(
                                    (uint32_t) this->regFile[currThread].reg[currInst.dst_index] +
                                    currInst.src2_index_imm,
                                    this->regFile[currThread].reg[currInst.src1_index]);
                        } else {
                            SIM_MemDataWrite((uint32_t) this->regFile[currThread].reg[currInst.dst_index] +
                                            this->regFile[currThread].reg[currInst.src2_index_imm],
                                            this->regFile[currThread].reg[currInst.src1_index]);
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
            this->threads[currThread].nextLine += 4;
            this->numOfCycles ++;
            if(!isIdle) this->numOfInts++;

            if(isEvent || isIdle) {
                int nextThread = this->findNextThread(currThread);
                if(nextThread == -2){
                    allThreadsFinishd = true;
                }else {
                    if (nextThread == -1)isIdle = true;
                    else isIdle = false;
                    if (!isIdle && currThread != nextThread)this->numOfCycles += this->switchOverhead;
                    if (!isIdle) currThread = nextThread;
                }
            }
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
                        this->regFile[currThread].reg[currInst.dst_index] =
                                this->regFile[currThread].reg[currInst.src1_index] + currInst.src2_index_imm;
                        break;
                    case CMD_SUBI:
                        this->regFile[currThread].reg[currInst.dst_index] =
                                this->regFile[currThread].reg[currInst.src1_index] - currInst.src2_index_imm;
                        break;
                    case CMD_ADD:
                        this->regFile[currThread].reg[currInst.dst_index] =
                                this->regFile[currThread].reg[currInst.src1_index] +
                                this->regFile[currThread].reg[currInst.src2_index_imm];
                        break;
                    case CMD_SUB:
                        this->regFile[currThread].reg[currInst.dst_index] =
                                this->regFile[currThread].reg[currInst.src1_index] -
                                this->regFile[currThread].reg[currInst.src2_index_imm];
                        break;
                    case CMD_LOAD:
                        if (currInst.isSrc2Imm) {
                            SIM_MemDataRead(
                                    (uint32_t) this->regFile[currThread].reg[currInst.src1_index] +
                                    currInst.src2_index_imm,
                                    &(this->regFile[currThread].reg[currInst.dst_index]));
                        } else {
                            SIM_MemDataRead((uint32_t) this->regFile[currThread].reg[currInst.src1_index] +
                                            this->regFile[currThread].reg[currInst.src2_index_imm],
                                            &(this->regFile[currThread].reg[currInst.dst_index]));
                        }
                        this->threads[currThread].idleUntilCycle = this->numOfCycles + this->loadLatency;
                        break;
                    case CMD_STORE:
                        if (currInst.isSrc2Imm) {
                            SIM_MemDataWrite(
                                    (uint32_t) this->regFile[currThread].reg[currInst.dst_index] +
                                    currInst.src2_index_imm,
                                    this->regFile[currThread].reg[currInst.src1_index]);
                        } else {
                            SIM_MemDataWrite((uint32_t) this->regFile[currThread].reg[currInst.dst_index] +
                                            this->regFile[currThread].reg[currInst.src2_index_imm],
                                            this->regFile[currThread].reg[currInst.src1_index]);
                        }
                        this->threads[currThread].idleUntilCycle = this->numOfCycles + this->storeLatency;
                        break;
                    case CMD_HALT:
                        this->threads[currThread].isFinished = true;
                        break;
                }
            }
            this->threads[currThread].nextLine += 4;
            this->numOfCycles ++;
            if(!isIdle) this->numOfInts++;
            int nextThread = this->findNextThread(currThread);
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


CoreMT* BlockedMT= nullptr;
CoreMT* FinegrainedMT= nullptr;

void CORE_BlockedMT() {
    BlockedMT = new CoreMT(true);
    BlockedMT->runBlockedMT();
}

void CORE_FinegrainedMT() {
    FinegrainedMT = new CoreMT(false);
    FinegrainedMT->runFinegrainedMT();
}

double CORE_BlockedMT_CPI(){
    if(BlockedMT == nullptr) return 0;
    double CPI = ((double)(BlockedMT->numOfCycles)/(BlockedMT->numOfInts));
    delete BlockedMT;
    return CPI;
}

double CORE_FinegrainedMT_CPI(){
    if(FinegrainedMT == nullptr) return 0;
    double CPI = ((double)(FinegrainedMT->numOfCycles)/(BlockedMT->numOfInts));
    delete BlockedMT;
    return CPI;
}

void CORE_BlockedMT_CTX(tcontext* context, int threadid) {
    *context = BlockedMT->regFile[threadid];
}

void CORE_FinegrainedMT_CTX(tcontext* context, int threadid) {
    *context = FinegrainedMT->regFile[threadid];
}
