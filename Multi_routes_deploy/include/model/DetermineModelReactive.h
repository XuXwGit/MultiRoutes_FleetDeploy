#ifndef DETERMINE_REACTIVE_MODEL_H
#define DETERMINE_REACTIVE_MODEL_H

#include <iostream>
#include <vector>

#include <ilcplex/ilocplex.h>

#include "InputData.h"
#include "Parameter.h"
#include "Setting.h"


namespace fleetdeployment
{


    class  DetermineModelReactive : public Setting {
    public:
        DetermineModelReactive(const InputData& in, const Parameter& p);

        void SolveModel();
        void PrintSolution();

    private:
        InputData in_;
        Parameter p_;
        IloEnv env_;
        IloCplex cplex_;
        IloModel model_;

        // Decision variable arrays
        std::vector<std::vector<IloIntVar>> vVar_;
        std::vector<std::vector<IloIntVar>> vVar2_;
        std::vector<std::vector<IloNumVar>> xVar_;
        std::vector<std::vector<IloNumVar>> yVar_;
        std::vector<std::vector<IloNumVar>> zVar_;
        std::vector<IloNumVar> gVar_;

        // Objective and operational cost variables
        double objVal_;
        double operationCost_;

        // Decision variable values
        std::vector<std::vector<int>> vVarValue_;
        std::vector<std::vector<int>> vVarValue2_;

        // Additional members
        double solveTime_;
        double mipGap_;

    public:
        inline void SetMipGap(double mipGap) { mipGap_ = mipGap; }
        inline double GetMipGap() const { return mipGap_; }
        inline void SetSolveTime(double solveTime) { solveTime_ = solveTime; }
        inline double GetSolveTime() const { return solveTime_; }
        inline void SetObjVal(double objVal) { objVal_ = objVal; }
        inline double GetObjVal() const { return objVal_; }
        //inline void SetOperationCost(double operationCost) { operationCost_ = operationCost; }
        //inline double GetOperationCost() const { return operationCost_; }
        inline void SetVVarValue(std::vector<std::vector<int>> vVarValue) { vVarValue_ = vVarValue; }
        inline std::vector<std::vector<int>> GetVVarValue() const { return vVarValue_; }
        inline void SetVVarValue2(std::vector<std::vector<int>> vVarValue2) { vVarValue2_ = vVarValue2; }
        inline std::vector<std::vector<int>> GetVVarValue2() const { return vVarValue2_; }

    private:
        void SetDecisionVars();
        void SetObjective();
        void SetConstraints();

        void SetConstraint1();
        void SetConstraint2();
        void SetConstraint3_1();
        void SetConstraint3_2();
        void SetConstraint4();
    };


}

#endif //!DETERMINE_REACTIVE_MODEL_H