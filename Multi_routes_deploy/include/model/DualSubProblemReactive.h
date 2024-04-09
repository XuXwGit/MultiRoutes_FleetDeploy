#ifndef DUALSUBPROBLEMREACTIVE_H
#define DUALSUBPROBLEMREACTIVE_H

#include <vector>
#include <ilcplex/ilocplex.h>

#include "InputData.h"
#include "Parameter.h"
#include "Setting.h"
#include "Scenario.h"



namespace fleetdeployment
{



    class  DualSubProblemReactive : public Setting {
    public:
        DualSubProblemReactive(const InputData& inputData, const Parameter& parameter, int tau);

        Scenario GetScene() const {
            return Scene_;
        }

        double GetObjVal() const {
            return objVal_;
        }
        void SetObjVal(double objective) {
            objVal_ = objective;
        }

        void end() {
            cplex_.end();
            model_.end();
            env_.end();
        }

        IloCplex::IloAlgorithm::Status GetSolveStatus() const {
            return cplex_.getStatus();
        }

        double GetMipGap() const {
            return mipGap_;
        }

    private:
        const InputData& in_;
        const Parameter& p_;
        int tau_;

        IloEnv env_;
        IloCplex cplex_;
        IloModel model_;

        std::vector<std::vector<int>> vVarValue1_;
        std::vector<std::vector<int>> vVarValue2_;
        std::vector<int> uVarValue_;

        Scenario Scene_;

        std::vector<IloNumVar> alphaVar_;
        std::vector<IloNumVar> betaVar1_;
        std::vector<IloNumVar> betaVar2_;
        std::vector<IloNumVar> lambdaVar_;
        std::vector<IloIntVar> miuVar_;
        std::vector<std::vector<IloNumVar>> gammaVar_;

        IloObjective obj_;

        double objVal_;
        double mipGap_;

        void SetDecisionVars();
        void SetObjective();

        void SetConstraints();
        void SetConstraint1();
        void SetConstraint2();
        void SetConstraint3();
        void SetConstraint4();
        void SetConstraint5();
        void SetConstraint6();
        void SetConstraint7();
        void SetConstraint8();
        void SetConstraint9();

    public:
        void changeObjectiveVCoefficients(const std::vector<std::vector<int>>& vValue1, const std::vector<std::vector<int>>& vValue2);
        void SolveModel();
        void PrintSolution();
        double GetConstantItem();
        std::vector<double> GetBeta1Value();
        std::vector<double> GetBeta2Value();
        std::string GetSolveStatusString();
        inline void SetMipGap(double gap) {
            this->mipGap_ = gap;
        }
        Scenario& GetScene() {
            return Scene_;
        }
    };



}
#endif // DUALSUBPROBLEMREACTIVE_H
