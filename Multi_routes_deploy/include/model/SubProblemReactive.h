#ifndef SUBPROBLEM_REACTIVE_H
#define SUBPROBLEM_REACTIVE_H

#include <vector>
#include <string>
#include <iostream>
#include <memory>

#include <ilcplex/ilocplex.h>

#include "InputData.h" 
#include "Parameter.h" 
#include "Setting.h"



namespace fleetdeployment
{


    class SubProblemReactive : public Setting
    {
    public:
        SubProblemReactive(const InputData& in, const Parameter& p);
        ~SubProblemReactive() {
            model_.end();
            cplex_.end();
            env_.end();
        }

    private:

        void SetDecisionVars();
        void SetObjective();
        void SetConstraints();

        void SetConstraint1();
        void SetConstraint2_1();
        void SetConstraint2_2();
        void SetConstraint3();

    public:

        void solveModel();

        void changeConstraintCoefficients(const std::vector<std::vector<int>>& vVarValue1, const std::vector<std::vector<int>>& vVarValue2, const std::vector<double>& uValue);

        inline void ExportModel() {
            try {
                std::string filename = "SP_Reactive.lp";
                cplex_.exportModel(filename.c_str());
            }
            catch (const IloException& e) {
                std::cerr << e.getMessage() << std::endl;
            }
        }

    private:
        InputData in_;
        Parameter p_;
        std::vector<std::vector<int>> vVarValue1_;
        std::vector<std::vector<int>> vVarValue2_;
        std::vector<double> uValue_;
        std::vector<double> uu_;

        IloEnv env_;
        IloCplex cplex_;
        IloModel model_;

        double objVal_;
        double ladenCost_;
        double emptyCost_;
        double penaltyCost_;
        double rentalCost_;

        std::vector<std::vector<IloNumVar>> xVar_;
        std::vector<std::vector<IloNumVar>> yVar_;
        std::vector<std::vector<IloNumVar>> zVar_;
        std::vector<IloNumVar> gVar_;

        std::vector<IloRange> C1_;
        std::vector<IloRange> C2_1_;
        std::vector<IloRange> C2_2_;
        std::vector<std::vector<IloRange>> C3_;

    public:
        inline void end() {
            env_.end();
            cplex_.end();
            model_.end();
        }

        double GetObjVal() const {
            return objVal_;
        }
        void SetObjVal(double obj) {
            objVal_ = obj;
        }

        void exportModel(const std::string& filename) {
            cplex_.exportModel(filename.c_str());
        }

        double GetLadenCost() const {
            return ladenCost_;
        }

        void SetLadenCost(double ladenCost) {
            ladenCost_ = ladenCost;
        }

        double GetEmptyCost() const {
            return emptyCost_;
        }

        void SetEmptyCost(double emptyCost) {
            emptyCost_ = emptyCost;
        }

        double GetPenaltyCost() const {
            return penaltyCost_;
        }

        void SetPenaltyCost(double penaltyCost) {
            penaltyCost_ = penaltyCost;
        }

        double GetRentalCost() const {
            return rentalCost_;
        }

        void SetRentalCost(double rentalCost) {
            rentalCost_ = rentalCost;
        }

    };


}

#endif // SUBPROBLEM_REACTIVE_H