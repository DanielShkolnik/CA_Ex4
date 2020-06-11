/* 046267 Computer Architecture - Spring 2020 - HW #4 */

#include "core_api.h"
#include "sim_api.h"

#include <stdio.h>

class ThreadStatus{
public:
    bool isFinished;
    int idleUntilCycle;


    ThreadStatus():isFinished(false),idleUntilCycle(-1){};
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
                                            storeLatency(SIM_GetStoreLat()),switchOverhead(SIM_GetSwitchCycles()),numOfInts(0){
        this->threads = new ThreadStatus[this->numOfThreads];
        this->regFile = new tcontext;
        for(int i=0; i<REGS_COUNT; i++){
            this->regFile->reg[i]=0;
        }
    };
    CoreMT(const CoreMT &coreMT) = delete;
    CoreMT &operator=(const CoreMT &coreMT) = delete;
    ~CoreMT(){};

    void runBlockedMT(){

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
	return 0;
}

double CORE_FinegrainedMT_CPI(){
	return 0;
}

void CORE_BlockedMT_CTX(tcontext* context, int threadid) {
}

void CORE_FinegrainedMT_CTX(tcontext* context, int threadid) {
}
