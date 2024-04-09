#ifndef DUALPROBLEM_H_
#define DUALPROBLEM_H_

#include <iostream>
#include <fstream>
#include <vector>
#include <string>
#include <stdexcept>
#include <memory>

#include <ilcplex/ilocplex.h>

#include "InputData.h"
#include "Parameter.h"
#include "Setting.h"



namespace fleetdeployment
{


    class  DualProblem : public Setting {
    private:
        const InputData& in_;
        const Parameter& p_;
        std::vector<std::vector<int>> vVarValue_;
        std::vector<int> uVarValue_;

        double objVal_;

        IloEnv env_;
        IloModel model_;
        IloCplex cplex_;

        std::vector<IloNumVar> alphaVar_;
        std::vector<IloNumVar> betaVar_;
        std::vector<std::vector<IloNumVar>> gammaVar_;

        IloObjective objective_;

        void SetDecisionVars();
        void SetObjective();

        void SetConstraints();
        void SetConstraint1();
        void SetConstraint2();
        void SetConstraint3();
        void SetConstraint4();

    public:
        DualProblem(const InputData& input, const Parameter& params);

        void SolveModel();
        void exportModel(const std::string& filename);

        void WriteSolution();

        inline ~DualProblem() {
            model_.end();
            cplex_.end();
            env_.end();
        }

    public:
        void changeObjectiveCoefficients(const std::vector<std::vector<int>>& vValue, const std::vector<int>& uValue);
        std::vector<double> GetBetaValue();
        inline double getObjVal() const {
            return objVal_;
        }
        inline IloCplex::IloAlgorithm::Status getSolveStatus() const {
            return cplex_.getStatus();
        }
        double GetConstantItem();
    private:
        inline void SetObjVal(double obj) {
            objVal_ = obj;
        }
    };

}

#endif // !DUALPROBLEM_H_