#include "SubProblemReactive.h"

namespace fleetdeployment {

    SubProblemReactive::SubProblemReactive(const InputData& in, const Parameter& p) 
        : in_(in), p_(p)
    {
        try {
            env_ = IloEnv();
            model_ = IloModel(env_);
            cplex_ = IloCplex(model_);

            cplex_.setOut(env_.getNullStream());
            cplex_.setParam(IloCplex::Param::WorkMem, MaxWorkMem);
            cplex_.setParam(IloCplex::Param::TimeLimit, MIPTimeLimit);
            cplex_.setParam(IloCplex::Param::MIP::Tolerances::MIPGap, MIPGapLimit);
            cplex_.setParam(IloCplex::Param::Threads, MaxThreads);

            SetDecisionVars();
            SetObjective();
            SetConstraints();
        }
        catch (const IloException& e) {
            std::cerr << "Error: " << e.getMessage() << std::endl;
        }

        // 使用 std::vector 初始化 vVarValue1, vVarValue2 和 uValue
        vVarValue1_.resize(p_.GetVesselSet().size(), std::vector<int>(p_.GetVesselRouteSet().size()));
        vVarValue2_.resize(p_.GetVesselSet().size(), std::vector<int>(p_.GetVesselPathSet().size()));
        uValue_.resize(p_.GetDemand().size());
    }



    void SubProblemReactive::SetDecisionVars() {
        // Assuming Request, InputData, Parameter classes are defined appropriately
        // Resize vectors according to the number of demands
        xVar_.resize(p_.GetDemand().size());
        yVar_.resize(p_.GetDemand().size());
        zVar_.resize(p_.GetDemand().size());
        gVar_.resize(p_.GetDemand().size());

        std::string varName;

        for (int i = 0; i < p_.GetDemand().size(); ++i) {
            Request od = in_.GetRequests()[i];

            xVar_[i].resize(od.GetNumberOfLadenPath());
            yVar_[i].resize(od.GetNumberOfLadenPath());
            zVar_[i].resize(od.GetNumberOfEmptyPath());

            for (int k = 0; k < od.GetNumberOfLadenPath(); ++k) {
                varName = "x(" + std::to_string(i + 1) + ")";
                xVar_[i][k] = IloNumVar(env_, 0, IloInfinity, varName.c_str());
                varName = "y(" + std::to_string(i + 1) + ")";
                yVar_[i][k] = IloNumVar(env_, 0, IloInfinity, varName.c_str());
            }
            for (int k = 0; k < od.GetNumberOfEmptyPath(); ++k) {
                varName = "z(" + std::to_string(i + 1) + ")";
                zVar_[i][k] = IloNumVar(env_, 0, IloInfinity, varName.c_str());
            }

            varName = "g(" + std::to_string(i + 1) + ")";
            gVar_[i] = IloNumVar(env_, 0, IloInfinity, varName.c_str());
        }
    }


    // Minimize total cost about containers
    void SubProblemReactive::SetObjective() {
        IloExpr obj(env_);

        // Iterate over demands
        for (int i = 0; i < p_.GetDemand().size(); ++i) {
            // Penalty cost for unsatisfied demand
            obj += p_.GetPenaltyCostForDemand()[i] * gVar_[i];

            Request od = in_.GetRequests()[i];
            // Iterate over laden paths
            for (int k = 0; k < od.GetNumberOfLadenPath(); ++k) {
                int j = od.GetLadenPathIndexes()[k];
                // Demurrage of self-owned and leased containers, and rental cost on laden paths
                obj += p_.GetLadenPathCost()[j] * xVar_[i][k];
                obj += p_.GetLadenPathCost()[j] * yVar_[i][k];
                obj += p_.GetRentalCost() * p_.GetTravelTimeOnPath()[j] * yVar_[i][k];
            }

            // Iterate over empty paths
            for (int k = 0; k < od.GetNumberOfEmptyPath(); ++k) {
                int j = od.GetEmptyPathIndexes()[k];
                // Demurrage of self-owned containers for empty path
                obj += p_.GetEmptyPathCost()[j] * zVar_[i][k];
            }
        }

        model_.add(IloMinimize(env_, obj));
    }


    void SubProblemReactive::SetConstraints() {
        SetConstraint1();
        SetConstraint2_1();
        SetConstraint2_2();
        SetConstraint3();
    }

    // Demand equation :
    // C-5------α
    // C-5 = 0
#include <ilcplex/ilocplex.h>
#include <string>

    void SubProblemReactive::SetConstraint1() {
        // Assuming C1_ is a std::vector<IloRange> member of SubProblemReactive
        C1_.resize(p_.GetDemand().size());

        // ∀i∈I
        for (int i = 0; i < p_.GetDemand().size(); ++i) {
            IloEnv env = cplex_.getEnv();
            IloExpr left(env_);

            Request od = in_.GetRequests()[i];
            // φ
            for (int k = 0; k < od.GetNumberOfLadenPath(); ++k) {
                left += xVar_[i][k];
                left += yVar_[i][k];
            }

            left += gVar_[i];

            std::string constr_name = "C1(" + std::to_string(i + 1) + ")";
            C1_[i] = IloRange(env_, p_.GetDemand()[i] + p_.GetMaximumDemandVariation()[i] * uValue_[i], left, p_.GetDemand()[i] + p_.GetMaximumDemandVariation()[i] * uValue_[i], constr_name.c_str());
            // Add the constraint to the model
            cplex_.getModel().add(C1_[i]);
        }
    }


    // Vessel Capacity Constraint :
    // C-6------β
    // C-6<= 0
    void SubProblemReactive::SetConstraint2_1() {
        C2_1_.resize(p_.GetTravelArcsSet().size());

        for (int nn = 0; nn < p_.GetTravelArcsSet().size(); ++nn) {
            IloEnv env = cplex_.getEnv();
            IloExpr left(env_);

            for (int i = 0; i < p_.GetDemand().size(); ++i) {
                Request od = in_.GetRequests()[i];
                for (int k = 0; k < od.GetNumberOfLadenPath(); ++k) {
                    int j = od.GetLadenPathIndexes()[k];
                    left += p_.GetArcAndPath()[nn][j] * xVar_[i][k];
                    left += p_.GetArcAndPath()[nn][j] * yVar_[i][k];
                }
            }

            double capacity1 = 0;
            for (int w = 0; w < p_.GetVesselPathSet().size(); ++w) {
                int r = in_.GetVesselPathSet()[w].GetRouteID() - 1;
                for (int h = 0; h < p_.GetVesselSet().size(); ++h) {
                    capacity1 += p_.GetArcAndVesselPath()[nn][w]
                        * p_.GetShipRouteAndVesselPath()[r][w]
                        * p_.GetVesselTypeAndShipRoute()[h][r]
                        * p_.GetVesselCapacity()[h]
                        * vVarValue1_[h][r];
                }
            }

            std::string constr_name = "C6-_" + std::to_string(nn + 1);
            C2_1_[nn] = IloRange(env, left, capacity1, constr_name.c_str());
            cplex_.getModel().add(C2_1_[nn]);
        }
    }

    void SubProblemReactive::SetConstraint2_2() {
        C2_2_.resize(p_.GetTravelArcsSet().size());

        for (int nn = 0; nn < p_.GetTravelArcsSet().size(); ++nn) {
            IloEnv env = cplex_.getEnv();
            IloExpr left(env_);

            for (int i = 0; i < p_.GetDemand().size(); ++i) {
                Request od = in_.GetRequests()[i];
                for (int k = 0; k < od.GetNumberOfEmptyPath(); ++k) {
                    int j = od.GetEmptyPathIndexes()[k];
                    left += p_.GetArcAndPath()[nn][j] * zVar_[i][k];
                }
            }

            double capacity2 = 0;
            for (int w = 0; w < p_.GetVesselPathSet().size(); ++w) {
                int r = in_.GetVesselPathSet()[w].GetRouteID() - 1;
                for (int h = 0; h < p_.GetVesselSet().size(); ++h) {
                    capacity2 += p_.GetArcAndVesselPath()[nn][w]
                        * p_.GetShipRouteAndVesselPath()[r][w]
                        * p_.GetVesselTypeAndShipRoute()[h][r]
                        * p_.GetVesselCapacity()[h]
                        * vVarValue2_[h][w];
                }
            }

            std::string constr_name = "C6-_" + std::to_string(nn + 1);
            C2_2_[nn] = IloRange(env, left, capacity2, constr_name.c_str());
            cplex_.getModel().add(C2_2_[nn]);
        }
    }



    // Container Flow Conservation Constraint :
    // calculate the number of available empty self-owned containers at port p at time t
    // or
    // L[p][t] > 0 means all empty self-owned containers that repositioning to other ports (sumZ(out))
    // plus all laden self-owned containers that transport to other ports (sumX(out))
    // should no more than (L[p][t-1] + sumX(in) + sumZ(in))
    // C7------γ
    // C7>=0  =>  -C7 <= 0
    // ( item4 - item1 ) * Z + (item3 - item2) * X <= Lp0
    // Output-X + OutputZ - Input-X - Input-Z <= lp0
#include <ilcplex/ilocplex.h>
#include <string>
#include <vector>

    void SubProblemReactive::SetConstraint3() {
        C3_.resize(p_.GetPortSet().size());
        for (auto& row : C3_) {
            row.resize(p_.GetTimePointSet().size());
        }

        for (int pp = 0; pp < p_.GetPortSet().size(); ++pp) {
            for (int t = 1; t < p_.GetTimePointSet().size(); ++t) {
                IloEnv env = cplex_.getEnv();
                IloExpr left(env_);

                for (int i = 0; i < p_.GetDemand().size(); ++i) {
                    Request od = in_.GetRequests()[i];

                    // Input Z flow
                    if (p_.GetPortSet()[pp] == p_.GetOriginOfDemand()[i]) {
                        for (int k = 0; k < od.GetNumberOfEmptyPath(); ++k) {
                            int j = od.GetEmptyPathIndexes()[k];
                            for (int nn = 0; nn < p_.GetTravelArcsSet().size(); ++nn) {
                                if (in_.GetTravelArcs()[nn].GetDestinationPort() == p_.GetPortSet()[pp]
                                    && in_.GetTravelArcs()[nn].GetDestinationTime() <= t
                                    && in_.GetTravelArcs()[nn].GetDestinationTime() >= 1) {
                                    left += p_.GetArcAndPath()[nn][j] * zVar_[i][k];
                                }
                            }
                        }
                    }

                    // Input flow X
                    if (p_.GetPortSet()[pp] == p_.GetDestinationOfDemand()[i]) {
                        for (int k = 0; k < od.GetNumberOfLadenPath(); ++k) {
                            int j = od.GetLadenPathIndexes()[k];
                            for (int nn = 0; nn < p_.GetTravelArcsSet().size(); ++nn) {
                                if (in_.GetTravelArcs()[nn].GetDestinationPort() == p_.GetPortSet()[pp]
                                    && in_.GetTravelArcs()[nn].GetDestinationTime() <= t - p_.GetTurnOverTimeSet()[pp]
                                    && in_.GetTravelArcs()[nn].GetDestinationTime() >= 1) {
                                    left += p_.GetArcAndPath()[nn][j] * xVar_[i][k];
                                }
                            }
                        }
                    }

                    // Output flow X
                    if (p_.GetPortSet()[pp] == p_.GetOriginOfDemand()[i]) {
                        for (int k = 0; k < od.GetNumberOfLadenPath(); ++k) {
                            int j = od.GetLadenPathIndexes()[k];
                            for (int nn = 0; nn < p_.GetTravelArcsSet().size(); ++nn) {
                                if (in_.GetTravelArcs()[nn].GetOriginPort() == p_.GetPortSet()[pp]
                                    && in_.GetTravelArcs()[nn].GetOriginTime() <= t
                                    && in_.GetTravelArcs()[nn].GetOriginTime() >= 1) {
                                    left -= p_.GetArcAndPath()[nn][j] * xVar_[i][k];
                                }
                            }
                        }
                    }

                    // Output Flow Z
                    for (int k = 0; k < od.GetNumberOfEmptyPath(); ++k) {
                        int j = od.GetEmptyPathIndexes()[k];
                        for (int nn = 0; nn < p_.GetTravelArcsSet().size(); ++nn) {
                            if (in_.GetTravelArcs()[nn].GetOriginPort() == p_.GetPortSet()[pp]
                                && in_.GetTravelArcs()[nn].GetOriginTime() <= t
                                && in_.GetTravelArcs()[nn].GetOriginTime() >= 1) {
                                left -= p_.GetArcAndPath()[nn][j] * zVar_[i][k];
                            }
                        }
                    }
                }

                std::string constr_name = "C7_" + std::to_string(pp + 1) + "_" + std::to_string(t);
                C3_[pp][t] = IloRange(env, -p_.GetInitialEmptyContainer()[pp], left, IloInfinity, constr_name.c_str());
                cplex_.getModel().add(C3_[pp][t]);
            }
        }
    }


#include <ilcplex/ilocplex.h>
#include <iostream>

    void SubProblemReactive::solveModel() {
        try {
            if (cplex_.solve()) {
                SetObjVal(cplex_.getObjValue());
            }
            else {
                std::cout << "SubProblem No solution" << std::endl;
            }
        }
        catch (const IloException& ex) {
            std::cout << "Concert Error: " << ex.getMessage() << std::endl;
        }
    }



    // change the coefficients of capacity constraints
#include <vector>

    void SubProblemReactive::changeConstraintCoefficients(const std::vector<std::vector<int>>& vVarValue1, const std::vector<std::vector<int>>& vVarValue2, const std::vector<double>& uValue) {
        vVarValue1_ = vVarValue1;
        vVarValue2_ = vVarValue2;
        uValue_ = uValue;

        for (int i = 0; i < p_.GetDemand().size(); ++i) {
            double newBound = p_.GetDemand()[i] + p_.GetMaximumDemandVariation()[i] * uValue_[i];
            C1_[i].setBounds(newBound, newBound);
        }

        for (int nn = 0; nn < p_.GetTravelArcsSet().size(); ++nn) {
            double capacity1 = 0;
            double capacity2 = 0;
            for (int w = 0; w < p_.GetVesselPathSet().size(); ++w) {
                int r = in_.GetVesselPathSet()[w].GetRouteID() - 1;
                for (int h = 0; h < p_.GetVesselSet().size(); ++h) {
                    capacity1 += p_.GetArcAndVesselPath()[nn][w] * p_.GetShipRouteAndVesselPath()[r][w] * p_.GetVesselTypeAndShipRoute()[h][r] * p_.GetVesselCapacity()[h] * vVarValue1_[h][r];
                    capacity2 += p_.GetArcAndVesselPath()[nn][w] * p_.GetShipRouteAndVesselPath()[r][w] * p_.GetVesselTypeAndShipRoute()[h][r] * p_.GetVesselCapacity()[h] * vVarValue2_[h][w];
                }
            }
            C2_1_[nn].setBounds(0, capacity1);
            C2_2_[nn].setBounds(0, capacity2);
        }
    }


}