#ifndef DEBUG_H_
# define DEBUG_H_


#include <iostream>
#include <random>


namespace fleetdeployment
{
    class  Setting;
}

class  fleetdeployment::Setting {
protected:
    // whether show Debug information
    static const bool DebugEnable = true;

    // whether show Debug information in GenerateParameter
    static const bool GenerateParamEnable = false;
    //const bool GenerateParamEnable = true;

    // whether show Debug information in subProblem
    static const bool SubEnable = true;
    //const bool SubEnable = false;

    // whether show Debug information in DualProblem
    static const bool DualEnable = false;
    //const bool DualEnable = true;

    // whether show Debug information in DualProblem
    static const bool DualSubEnable = false;
    //const bool DualSubEnable = true;

    // whether show Debug information in MasterProblem
    static const bool MasterEnable = true;
    //const bool MasterEnable = true;

    static const bool CCG_PAP_Use_Sp = true;

    static const int randomSeed = 0;

    static const bool wetherExportModel = true;
    static const bool WhetherOutputLog = false;

    static const bool WhetherPrintFileLog = true;
    static const bool WhetherPrintDataStatus = false;
    static const bool WhetherPrintVesselDecision = false;
    static const bool WhetherPrintIteration = true;
    static const bool WhetherPrintSolveTime = false;
    static const bool WhetherPrintProcess = true;

    const double MIPGapLimit = 1e-3;
    const double MIPTimeLimit = 3600;
    static const int MaxThreads = 16;
    static const int MaxWorkMem = 30720;
};


#endif // !DEBUG_H_