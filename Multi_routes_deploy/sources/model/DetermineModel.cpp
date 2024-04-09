# include "DetermineModel.h"

#include <iostream>
#include <ctime>

#include <ilcplex/ilocplex.h>


namespace fleetdeployment {


    DetermineModel::DetermineModel(const InputData& in, const Parameter& p)
        : in_(in), p_(p)
    {
        try {
            if (Setting::WhetherPrintProcess || Setting::WhetherPrintIteration) {
                std::cout << "=========DetermineModel==========" << std::endl;
            }

            env_ = IloEnv();
            model_ = IloModel(env_);
            cplex_ = IloCplex(model_);

            cplex_.setOut(env_.getNullStream());
            cplex_.setParam(IloCplex::Param::WorkMem, Setting::MaxWorkMem);
            cplex_.setParam(IloCplex::Param::TimeLimit, Setting::MIPTimeLimit);
            cplex_.setParam(IloCplex::Param::MIP::Tolerances::MIPGap, Setting::MIPGapLimit);
            cplex_.setParam(IloCplex::Param::Threads, Setting::MaxThreads);

            time_t begin = time(0);
            SetDecisionVars();
            SetObjective();
            SetConstraints();

            time_t start = time(0);
            SolveModel();
            time_t end = time(0);

            SetSolveTime(difftime(end, start));

            if (WhetherPrintProcess) {
                double buildTime = difftime(start, begin);
                double solveTime = difftime(end, start);
                std::cout << "BuildTime = " << buildTime << "\t\tSolveTime = " << solveTime << std::endl;
                std::cout << "Determine Objective = " << std::fixed << std::setprecision(2) << GetObjVal() << std::endl;
                PrintSolution();
                std::cout << "================================" << std::endl;
            }
        }
        catch (const IloException& e) {
            std::cerr << "Error: " << e.getMessage() << std::endl;
        }
    }

    void DetermineModel::SetDecisionVars() {

        vVar_.resize(p_.GetVesselSet().size());
        for (size_t h = 0; h < p_.GetVesselSet().size(); ++h) {
            vVar_[h].resize(p_.GetVesselRouteSet().size());
            for (size_t r = 0; r < p_.GetVesselRouteSet().size(); ++r) {
                std::string varName = "V(" + std::to_string(p_.GetVesselSet()[h]) + ")(" + std::to_string(p_.GetVesselRouteSet()[r]) + ")";
                vVar_[h][r] = IloIntVar(env_, 0, 1, varName.c_str());  // Assuming these are binary variables
            }
        }

        xVar_.resize(p_.GetDemand().size());
        yVar_.resize(p_.GetDemand().size());
        zVar_.resize(p_.GetDemand().size());
        gVar_.resize(p_.GetDemand().size());

        for (size_t i = 0; i < p_.GetDemand().size(); ++i) {
            const Request& od = in_.GetRequests()[i];

            xVar_[i].resize(od.GetNumberOfLadenPath());
            yVar_[i].resize(od.GetNumberOfLadenPath());
            zVar_[i].resize(od.GetNumberOfEmptyPath());

            for (size_t k = 0; k < od.GetNumberOfLadenPath(); ++k) {
                std::string varNameX = "x(" + std::to_string(i + 1) + ")";
                xVar_[i][k] = IloNumVar(env_, 0, IloInfinity, varNameX.c_str());

                std::string varNameY = "y(" + std::to_string(i + 1) + ")";
                yVar_[i][k] = IloNumVar(env_, 0, IloInfinity, varNameY.c_str());
            }

            for (size_t k = 0; k < od.GetNumberOfEmptyPath(); ++k) {
                std::string varNameZ = "z(" + std::to_string(i + 1) + ")";
                zVar_[i][k] = IloNumVar(env_, 0, IloInfinity, varNameZ.c_str());
            }

            std::string varNameG = "g(" + std::to_string(i + 1) + ")";
            gVar_[i] = IloNumVar(env_, 0, IloInfinity, varNameG.c_str());
        }
    }

    void DetermineModel::SetObjective() {
        try {

            IloExpr expr(env_);

            // Loop over vessel Set and vessel path Set
            for (size_t h = 0; h < p_.GetVesselSet().size(); ++h) {
                for (size_t w = 0; w < p_.GetVesselPathSet().size(); ++w) {
                    int r = in_.GetVesselPathSet()[w].GetRouteID() - 1;
                    expr += p_.GetVesselTypeAndShipRoute()[h][r] *
                        p_.GetShipRouteAndVesselPath()[r][w] *
                        p_.GetVesselOperationCost()[h] * vVar_[h][r];
                }
            }

            // Loop over demands
            for (size_t i = 0; i < p_.GetDemand().size(); ++i) {
                const Request& od = in_.GetRequests()[i];
                expr += p_.GetPenaltyCostForDemand()[i] * gVar_[i];

                // Adding terms for laden and empty paths
                for (size_t k = 0; k < od.GetNumberOfLadenPath(); ++k) {
                    int j = od.GetLadenPathIndexes()[k];
                    expr += p_.GetRentalCost() * p_.GetTravelTimeOnPath()[j] * yVar_[i][k];
                    expr += p_.GetLadenPathCost()[j] * yVar_[i][k];
                    expr += p_.GetLadenPathCost()[j] * xVar_[i][k];
                }
                for (size_t k = 0; k < od.GetNumberOfEmptyPath(); ++k) {
                    int j = od.GetEmptyPathIndexes()[k];
                    expr += p_.GetEmptyPathCost()[j] * zVar_[i][k];
                }
            }

            IloObjective obj = IloMinimize(env_, expr);
            model_.add(obj);
        }
        catch (const IloException& e) {
            std::cerr << "Error in SetObjectives: " << e.getMessage() << std::endl;
        }
    }


    void DetermineModel::SetConstraints()
    {
        // each ship route assigned to one vessel
        SetConstraint1();

        // Demand Equation Constraints
        SetConstraint2();

        // Transport Capacity Constraints
        SetConstraint3();

        // Containers Flow Conservation Constraints
        SetConstraint4();
    }

    void DetermineModel::SetConstraint1() {
        try {

            for (size_t r = 0; r < p_.GetVesselRouteSet().size(); ++r) {
                IloExpr left(env_);
                for (size_t h = 0; h < p_.GetVesselSet().size(); ++h) {
                    left += p_.GetVesselTypeAndShipRoute()[h][r] * vVar_[h][r];
                }

                // Create a named constraint
                std::string constr_name = "VesselAssignment" + std::to_string(r + 1);
                IloRange c1(env_, 1, left, 1);
                c1.setName(constr_name.c_str());

                // Adding the constraint to the model
                model_.add(c1);
            }
        }
        catch (const IloException& e) {
            std::cerr << "Error in SetConstraint1: " << e.getMessage() << std::endl;
        }
    }


    void DetermineModel::SetConstraint2() {
        try {
            for (size_t i = 0; i < p_.GetDemand().size(); ++i) {
                IloExpr left(env_);
                const Request& OD = in_.GetRequests()[i];

                // phi
                for (size_t k = 0; k < OD.GetNumberOfLadenPath(); ++k) {
                    left += xVar_[i][k];
                    left += yVar_[i][k];
                }

                left += gVar_[i];

                std::string constr_name = "Demand" + std::to_string(i + 1);
                IloRange c2(env_, p_.GetDemand()[i], left, p_.GetDemand()[i]);
                c2.setName(constr_name.c_str());
                model_.add(c2);
            }
        }
        catch (const IloException& e) {
            std::cerr << "Error in SetConstraint2: " << e.getMessage() << std::endl;
        }
    }


    // (4)
    // Capacity Constraint on each travel arc
    void DetermineModel::SetConstraint3() {
        try {
            for (size_t l = 0; l < p_.GetTravelArcsSet().size(); ++l) {
                IloExpr left(env_);

                for (size_t i = 0; i < p_.GetDemand().size(); ++i) {
                    const Request& od = in_.GetRequests()[i];

                    // phi
                    for (size_t k = 0; k < od.GetNumberOfLadenPath(); ++k) {
                        int j = od.GetLadenPathIndexes()[k];
                        left += p_.GetArcAndPath()[l][j] * xVar_[i][k];
                        left += p_.GetArcAndPath()[l][j] * yVar_[i][k];
                    }

                    // theta
                    for (size_t k = 0; k < od.GetNumberOfEmptyPath(); ++k) {
                        int j = od.GetEmptyPathIndexes()[k];
                        left += p_.GetArcAndPath()[l][j] * zVar_[i][k];
                    }
                }

                // r and w
                for (size_t w = 0; w < p_.GetVesselPathSet().size(); ++w) {
                    int r = in_.GetVesselPathSet()[w].GetRouteID() - 1;
                    for (size_t h = 0; h < p_.GetVesselSet().size(); ++h) {
                        left += -p_.GetArcAndVesselPath()[l][w] *
                            p_.GetVesselCapacity()[h] *
                            p_.GetShipRouteAndVesselPath()[r][w] *
                            p_.GetVesselTypeAndShipRoute()[h][r] *
                            vVar_[h][r];
                    }
                }

                std::string constr_name = "Capacity" + std::to_string(l + 1);
                IloRange c3(env_, -IloInfinity, left, 0);
                c3.setName(constr_name.c_str());
                model_.add(c3);
            }
        }
        catch (const IloException& e) {
            std::cerr << "Error in SetConstraint3: " << e.getMessage() << std::endl;
        }
    }

    // (29)
    // Containers flow conservation
    // Containers of each port p at each time t
    void DetermineModel::SetConstraint4() {
        try {
            for (size_t pp = 0; pp < p_.GetPortSet().size(); ++pp) {
                std::string port = p_.GetPortSet()[pp];
                for (size_t t = 1; t < p_.GetTimePointSet().size(); ++t) {
                    IloExpr left(env_);

                    for (size_t i = 0; i < p_.GetDemand().size(); ++i) {
                        const Request& od = in_.GetRequests()[i];

                        for (size_t nn = 0; nn < p_.GetTravelArcsSet().size(); ++nn) {
                            const TravelArc& arc = in_.GetTravelArcs()[nn];

                            // In-Z
                            if (od.GetOriginPort() == port) {
                                for (size_t k = 0; k < od.GetNumberOfEmptyPath(); ++k) {
                                    if (arc.GetDestinationPort() == port &&
                                        arc.GetDestinationTime() <= t &&
                                        arc.GetDestinationTime() >= 1) {
                                        int j = od.GetEmptyPathIndexes()[k];
                                        left += p_.GetArcAndPath()[nn][j] * zVar_[i][k];
                                    }
                                }
                            }

                            // Out-X
                            if (od.GetOriginPort() == port) {
                                for (size_t k = 0; k < od.GetNumberOfLadenPath(); ++k) {
                                    if (arc.GetOriginPort() == port &&
                                        arc.GetOriginTime() <= t &&
                                        arc.GetOriginTime() >= 1) {
                                        int j = od.GetLadenPathIndexes()[k];
                                        left -= p_.GetArcAndPath()[nn][j] * xVar_[i][k];
                                    }
                                }
                            }

                            // In-X
                            if (od.GetDestinationPort() == port) {
                                for (size_t k = 0; k < od.GetNumberOfLadenPath(); ++k) {
                                    if (arc.GetDestinationPort() == port &&
                                        arc.GetDestinationTime() <= t - p_.GetTurnOverTimeSet()[pp] &&
                                        arc.GetDestinationTime() >= 1) {
                                        int j = od.GetLadenPathIndexes()[k];
                                        left += p_.GetArcAndPath()[nn][j] * xVar_[i][k];
                                    }
                                }
                            }

                            // Out-Z
                            if (od.GetDestinationPort() == port) {
                                for (size_t k = 0; k < od.GetNumberOfEmptyPath(); ++k) {
                                    if (arc.GetOriginPort() == port &&
                                        arc.GetOriginTime() <= t &&
                                        arc.GetOriginTime() >= 1) {
                                        int j = od.GetEmptyPathIndexes()[k];
                                        left -= p_.GetArcAndPath()[nn][j] * zVar_[i][k];
                                    }
                                }
                            }
                        }
                    }
                    model_.add(IloRange(env_, left, -p_.GetInitialEmptyContainer()[pp]));
                }
            }
        }
        catch (const IloException& e) {
            std::cerr << "Error in SetConstraint4: " << e.getMessage() << std::endl;
        }
    }


    void DetermineModel::SolveModel() {
        try {
            if (cplex_.solve()) {
                SetObjVal(cplex_.getObjValue());

                std::vector<std::vector<int>> vvv(p_.GetVesselSet().size(), std::vector<int>(p_.GetVesselRouteSet().size()));
                for (size_t i = 0; i < p_.GetVesselRouteSet().size(); ++i) {
                    for (size_t j = 0; j < p_.GetVesselSet().size(); ++j) {
                        vvv[j][i] = static_cast<int>(cplex_.getValue(vVar_[j][i]) + 0.5);
                    }
                }
                SetVVarValue(vvv);

                mipGap_ = cplex_.getMIPRelativeGap();

                if (WhetherPrintProcess) {
                    PrintSolution();
                }
            }
            else {
                std::cout << "No solution" << std::endl;
            }
            cplex_.end();
        }
        catch (const IloException& e) {
            std::cerr << "Concert Error: " << e.getMessage() << std::endl;
        }
    }


    void DetermineModel::PrintSolution() {
        std::cout << "Objective = " << std::fixed << std::setprecision(2) << GetObjVal() << std::endl;
        std::cout << "Vessel Decision: ";
        for (size_t r = 0; r < p_.GetVesselRouteSet().size(); ++r) {
            std::cout << p_.GetVesselRouteSet()[r] << "(";
            for (size_t h = 0; h < p_.GetVesselSet().size(); ++h) {
                if (vVarValue_[h][r] != 0) {
                    std::cout << p_.GetVesselSet()[h] << ")\t";
                }
            }
        }
        std::cout << std::endl;
    }

}
