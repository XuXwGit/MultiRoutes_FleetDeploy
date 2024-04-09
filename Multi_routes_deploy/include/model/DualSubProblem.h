#ifndef DUALSUBPROBLEM_H
#define DUALSUBPROBLEM_H

#include <vector>
#include <string>

#include <ilcplex/ilocplex.h>

#include "Setting.h"
#include "InputData.h"
#include "Parameter.h"
#include "Request.h"
#include "Scenario.h"



namespace fleetdeployment
{


    class  DualSubProblem : public Setting {
    public:
        DualSubProblem(const InputData& inputData, const Parameter& parameter, int tau);
        ~DualSubProblem();

        std::vector<double> GetBetaValue();
        double GetConstantItem();

        void SolveModel();

        void PrintSolution();

        void changeObjectiveVCoefficients(const std::vector<std::vector<int>>& vValue);
        void changeVVarCoefficients(const std::vector<std::vector<int>>& vValue);

    private:
        void SetDecisionVars();

        void SetObjective();


        void SetConstraints();
        void SetConstraint1();
        void SetConstraint2();
        void SetConstraint3();
        void SetConstraint4();
        // uncertain Set
        void SetConstraint5();
        // linearize constraints
        // λ<=α
        void SetConstraint6();
        // λ>= α-(1-u)M
        void SetConstraint7();
        // λ<= M*u
        void SetConstraint8();
        // λ>=- M*u
        void SetConstraint9();

    private:
        InputData in_;
        Parameter p_;
        int tau_;

        Scenario Scene_;

        IloCplex cplex_;
        IloEnv env_;
        IloModel model_;

        std::vector<IloNumVar> alphaVar_;
        std::vector<IloNumVar> betaVar_;
        std::vector<std::vector<IloNumVar>> gammaVar_;
        std::vector<IloIntVar> miuVar_;
        std::vector<IloNumVar> lambdaVar_;

        IloObjective obj_;

        std::vector<int> uVarValue_;
        std::vector<std::vector<int>> vVarValue_;
        double objVal_;
        double mipGap_;


    public:
        inline std::vector<int> GetUValue() {
            return uVarValue_;
        }
        inline std::vector<double> GetUValueDouble() {
            return std::vector<double>(uVarValue_.begin(), uVarValue_.end());
        }
        inline Scenario GetScene() {
            return Scene_;
        }
        inline double GetObjVal() {
            return objVal_;
        }
        inline IloCplex::IloAlgorithm::Status GetSolveStatus() {
            return cplex_.getStatus();
        }
        inline std::string GetSolveStatusString() {
            if (cplex_.getStatus() == IloCplex::IloAlgorithm::Status::Optimal) {
                return "Optimal";
            }
            else if (cplex_.getStatus() == IloCplex::IloAlgorithm::Status::Infeasible) {
                return "Infeasible";
            }
            else if (cplex_.getStatus() == IloCplex::IloAlgorithm::Status::Unbounded) {
                return "Unbounded";
            }
            else if (cplex_.getStatus() == IloCplex::IloAlgorithm::Status::InfeasibleOrUnbounded) {
                return "InfeasibleOrUnbounded";
            }
            else if (cplex_.getStatus() == IloCplex::IloAlgorithm::Status::Error) {
                return "Error";
            }
            else if (cplex_.getStatus() == IloCplex::IloAlgorithm::Status::Unknown) {
                return "Unknown";
            }
            else {
                return "Error to get status";
            }
        }

        inline double GetMipGap() {
            return mipGap_;
        }

        // setter
        inline void SetUVarValue(const std::vector<int>& uValue) {
            uVarValue_ = uValue;
        }
        inline void SetVVarValue(const std::vector<std::vector<int>>& vValue) {
            vVarValue_ = vValue;
        }
        inline void SetObjVal(double objective) {
            objVal_ = objective;
        }
        inline void SetMipGap(double mipGap) {
            mipGap_ = mipGap;
        }

        inline void exportModel() {
            cplex_.exportModel("DualSubProblem.lp");
        }

        inline void end() {
            cplex_.end();
            model_.end();
            env_.end();
        }
    };



}
#endif //DUALSUBPROBLEM_H