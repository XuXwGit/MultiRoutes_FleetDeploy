#ifndef SUBPROBLEM_H
#define SUBPROBLEM_H

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


    class  SubProblem : public Setting
    {
    public:
        SubProblem(const InputData& in, const Parameter& p, const std::vector<double>& uValue);
        SubProblem(const InputData& in, const Parameter& p, const std::vector<int>& uValue);
        ~SubProblem() {
            model_.end();
            cplex_.end();
            env_.end();
        }

    private:

        void SetDecisionVars();
        void SetObjective();
        void SetConstraints();

        void SetConstraint1();
        void SetConstraint2();
        void SetConstraint3();

    public:

        void solveModel();

        double GetDualObjective();
        double GetTotalCost();


        void printSolutions();

        void changeObjSense(int flag);
        void changeConstraintCoefficients(const std::vector<std::vector<int>>& VValue, const std::vector<double>& uValue);
        void changeConstraintCoefficients(const std::vector<std::vector<int>>& VValue, const std::vector<int>& uValue);

        void writeSolution();
        void writeDualSolution();
        void writePortContainers();

        inline void ExportModel() {
            try {
                std::string filename = "SP.lp";
                cplex_.exportModel(filename.c_str());
            }
            catch (const IloException& e) {
                std::cerr << e.getMessage() << std::endl;
            }
        }

    private:
        InputData in_;
        Parameter p_;
        IloCplex cplex_;
        IloEnv env_;
        IloModel model_;

        double objVal_;
        double ladenCost_;
        double emptyCost_;
        double penaltyCost_;
        double rentalCost_;

        std::vector<std::vector<int>> vVarValue_;
        std::vector<double> uValue_;

        std::vector<std::vector<IloNumVar>> xVar_;
        std::vector<std::vector<IloNumVar>> yVar_;
        std::vector<std::vector<IloNumVar>> zVar_;
        std::vector<IloNumVar> gVar_;

        std::vector<IloRange> C1_;
        std::vector<IloRange> C2_;
        std::vector<std::vector<IloRange>> C3_;


    public:
        inline double GetObjVal() const { return objVal_; }
        inline void SetObjVal(double obj) { objVal_ = obj; }

        inline double GetLadenCost() const { return ladenCost_; }
        inline void SetLadenCost(double cost) { ladenCost_ = cost; }
        inline double GetEmptyCost() const { return emptyCost_; }
        inline void SetEmptyCost(double cost) { emptyCost_ = cost; }
        inline double GetPenaltyCost() const { return penaltyCost_; }
        inline void SetPenaltyCost(double cost) { penaltyCost_ = cost; }
        inline double GetRentalCost() const { return rentalCost_; }
        inline void SetRentalCost(double cost) { rentalCost_ = cost; }

        inline void end() {
            cplex_.end();
            model_.end();
            env_.end();
        }
    };


}

#endif // SUBPROBLEM_H
