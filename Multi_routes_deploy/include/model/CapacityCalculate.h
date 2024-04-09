#ifndef CAPACITY_CALCULATE_H
#define CAPACITY_CALCULATE_H

#include <vector>
#include <string>
#include <iostream>

#include <ilconcert/iloenv.h>
#include <ilcplex/ilocplex.h>

#include "InputData.h"
#include "Parameter.h"
#include "Request.h"
#include "Setting.h"


namespace fleetdeployment
{

    class  CapacityCalculate : public Setting {
    public:
  
        CapacityCalculate(const InputData& in, const Parameter& p);

        void SolveModel();

        inline void End() {
            cplex_.end();
            model_.end();
            env_.end();
        }


    private:
        void SetDecisionVars();
        void SetObjectives();
        void SetConstraints();

        void SetConstraint1();
        void SetConstraint2();
        void SetConstraint3();

        void SetMinV();
        void SetMaxV();

        inline void SetObjVal(double objVal) {
            objVal_ = objVal;
        }

    public:
        void ChangeDemandVariation();
        void PrintSolution();
        void PrintDetail();
        inline double GetObjVal(double& objVal) {
            return objVal_;
        }

        InputData in_;
        Parameter p_;
        IloEnv env_;
        IloCplex cplex_;
        IloModel model_;
        std::vector<std::vector<IloNumVar>> v_var_;
        std::vector<std::vector<IloNumVar>> x_var_;
        std::vector<std::vector<IloNumVar>> y_var_;
        std::vector<std::vector<IloNumVar>> z_var_;
        std::vector<IloNumVar> g_var_;
        std::vector<IloRange> c1_;
        std::vector<std::vector<int>> min_v_;
        std::vector<std::vector<int>> max_v_;
        IloObjective obj_;

        double objVal_;

    };



}
#endif // CAPACITY_CALCULATE_H
