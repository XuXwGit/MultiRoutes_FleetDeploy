#include "DetermineModelReactive.h"


namespace fleetdeployment {


    DetermineModelReactive::DetermineModelReactive(const InputData& in, const Parameter& p)
        : in_(in), p_(p)
    {
        try {
            if (Setting::WhetherPrintProcess || Setting::WhetherPrintIteration) {
                std::cout << "=========DetermineModel(Reactive)==========" << std::endl;
            }

            env_ = IloEnv();
            model_ = IloModel(env_);
            cplex_ = IloCplex(env_);

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

            std::cout << "================================" << std::endl;
        }
        catch (const IloException& e) {
            std::cerr << "Error: " << e.getMessage() << std::endl;
        }
    }


    void DetermineModelReactive::SetDecisionVars() {
        try {
            // Initialize vVar and vVar2
            vVar_.resize(p_.GetVesselSet().size(), std::vector<IloIntVar>(p_.GetVesselRouteSet().size()));
            vVar2_.resize(p_.GetVesselSet().size(), std::vector<IloIntVar>(p_.GetVesselPathSet().size()));

            // Create binary variables for vVar and vVar2
            for (size_t h = 0; h < p_.GetVesselSet().size(); ++h) {
                for (size_t r = 0; r < p_.GetVesselRouteSet().size(); ++r) {
                    std::string varName = "v(" + std::to_string(h + 1) + ")(" + std::to_string(r + 1) + ")";
                    vVar_[h][r] = IloIntVar(env_, 0, 1, varName.c_str());
                }
                for (size_t w = 0; w < p_.GetVesselPathSet().size(); ++w) {
                    std::string varName = "v(" + std::to_string(h + 1) + ")(" + std::to_string(w + 1) + ")";
                    vVar2_[h][w] = IloIntVar(env_, 0, 1, varName.c_str());
                }
            }

            // Initialize and create numeric variables for xVar, yVar, zVar
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
                    std::string varNameY = "y(" + std::to_string(i + 1) + ")";
                    xVar_[i][k] = IloNumVar(env_, 0, IloInfinity, varNameX.c_str());
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
        catch (const IloException& e) {
            std::cerr << "Error in SetDecisionVars: " << e.getMessage() << std::endl;
        }
    }


    void DetermineModelReactive::SetObjective() {
        try {
            IloExpr expr(env_);

            // Loop over vessel route and vessel path Set for vVar
            for (size_t r = 0; r < p_.GetVesselRouteSet().size(); ++r) {
                for (size_t h = 0; h < p_.GetVesselSet().size(); ++h) {
                    for (size_t w = 0; w < p_.GetVesselPathSet().size(); ++w) {
                        expr += p_.GetShipRouteAndVesselPath()[r][w] *
                            p_.GetVesselTypeAndShipRoute()[h][r] *
                            p_.GetVesselOperationCost()[h] * vVar_[h][r];
                    }
                }
            }

            // Loop over vessel path Set for vVar2
            for (size_t h = 0; h < p_.GetVesselSet().size(); ++h) {
                for (size_t w = 0; w < p_.GetVesselPathSet().size(); ++w) {
                    int r = in_.GetVesselPathSet()[w].GetRouteID() - 1;
                    expr += p_.GetShipRouteAndVesselPath()[r][w] *
                        p_.GetVesselTypeAndShipRoute()[h][r] *
                        p_.GetVesselOperationCost()[h] * vVar2_[h][w];
                }
            }

            // Loop over demands
            for (size_t i = 0; i < p_.GetDemand().size(); ++i) {
                const Request& od = in_.GetRequests()[i];
                expr += p_.GetPenaltyCostForDemand()[i] * gVar_[i];

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



    void DetermineModelReactive::SetConstraints() {
        try {
            SetConstraint1();  // Each ship route assigned to one vessel
            SetConstraint2();  // Demand Equation Constraints
            SetConstraint3_1();  // Transport Capacity Constraints - Part 1
            SetConstraint3_2();  // Transport Capacity Constraints - Part 2
            SetConstraint4();  // Containers Flow Conservation Constraints
        }
        catch (const IloException& e) {
            std::cerr << "Error in SetConstraints: " << e.getMessage() << std::endl;
        }
    }



    // (2)
    // Each Route should be assigned only one Vessel
    void DetermineModelReactive::SetConstraint1() {
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



    // (3)
    // Demand Equation Constraints
    void DetermineModelReactive::SetConstraint2() {
        try {
            for (size_t i = 0; i < p_.GetDemand().size(); ++i) {
                IloExpr left(env_);
                const Request& OD = in_.GetRequests()[i];

                for (size_t k = 0; k < OD.GetNumberOfLadenPath(); ++k) {
                    int j = OD.GetLadenPathIndexes()[k];
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
    void DetermineModelReactive::SetConstraint3_1() {
        try {
            for (size_t l = 0; l < p_.GetTravelArcsSet().size(); ++l) {
                IloExpr left(env_);

                // Loop over demands
                for (size_t i = 0; i < p_.GetDemand().size(); ++i) {
                    const Request& od = in_.GetRequests()[i];

                    // Loop over laden paths (phi)
                    for (size_t k = 0; k < od.GetNumberOfLadenPath(); ++k) {
                        int j = od.GetLadenPathIndexes()[k];
                        left += p_.GetArcAndPath()[l][j] * xVar_[i][k];
                        left += p_.GetArcAndPath()[l][j] * yVar_[i][k];
                    }
                }

                // Loop over vessel route and path sets
                for (size_t r = 0; r < p_.GetVesselRouteSet().size(); ++r) {
                    for (size_t w = 0; w < p_.GetVesselPathSet().size(); ++w) {
                        for (size_t h = 0; h < p_.GetVesselSet().size(); ++h) {
                            left += -p_.GetArcAndVesselPath()[l][w] *
                                p_.GetVesselCapacity()[h] *
                                p_.GetShipRouteAndVesselPath()[r][w] *
                                p_.GetVesselTypeAndShipRoute()[h][r] *
                                vVar_[h][r];
                        }
                    }
                }

                std::string constr_name = "Capacity1" + std::to_string(l + 1);
                IloRange c3(env_, -IloInfinity, left, 0);
                c3.setName(constr_name.c_str());
                model_.add(c3);  // Add the constraint to the model
            }
        }
        catch (const IloException& e) {
            std::cerr << "Error in SetConstraint3_1: " << e.getMessage() << std::endl;
        }
    }


    void DetermineModelReactive::SetConstraint3_2() {
        try {
            for (size_t l = 0; l < p_.GetTravelArcsSet().size(); ++l) {
                IloExpr left(env_);

                // Loop over demands for empty paths (theta)
                for (size_t i = 0; i < p_.GetDemand().size(); ++i) {
                    const Request& od = in_.GetRequests()[i];
                    for (size_t k = 0; k < od.GetNumberOfEmptyPath(); ++k) {
                        int j = od.GetEmptyPathIndexes()[k];
                        left += p_.GetArcAndPath()[l][j] * zVar_[i][k];
                    }
                }

                // Loop over vessel path set
                for (size_t w = 0; w < p_.GetVesselPathSet().size(); ++w) {
                    int r = in_.GetVesselPathSet()[w].GetRouteID() - 1;
                    for (size_t h = 0; h < p_.GetVesselSet().size(); ++h) {
                        left += -p_.GetArcAndVesselPath()[l][w] *
                            p_.GetVesselCapacity()[h] *
                            p_.GetShipRouteAndVesselPath()[r][w] *
                            p_.GetVesselTypeAndShipRoute()[h][r] *
                            vVar2_[h][w];
                    }
                }

                std::string constr_name = "Capacity2" + std::to_string(l + 1);
                IloRange c3(env_, -IloInfinity, left, 0);
                c3.setName(constr_name.c_str());
                model_.add(c3);  // Add the constraint to the model
            }
        }
        catch (const IloException& e) {
            std::cerr << "Error in SetConstraint3_2: " << e.getMessage() << std::endl;
        }
    }


    // (29)
    // Containers flow conservation
    // Containers of each port p at each time t
    void DetermineModelReactive::SetConstraint4() {
        try {
            TravelArc arc;
            // Loop over ports
            for (size_t pp = 0; pp < p_.GetPortSet().size(); ++pp) {
                const std::string& port = p_.GetPortSet()[pp];
                // Loop over time points
                for (int t = 1; t < p_.GetTimePointSet().size(); ++t) {
                    IloExpr left(env_);
                    // Loop over demands
                    for (size_t i = 0; i < p_.GetDemand().size(); ++i) {
                        const Request& od = in_.GetRequests()[i];
                        // Loop over travel arcs
                        for (size_t nn = 0; nn < p_.GetTravelArcsSet().size(); ++nn) {
                            arc = in_.GetTravelArcs()[nn];
                            // Origin and destination ports
                            const std::string& originPort = arc.GetOriginPort();
                            const std::string& destPort = arc.GetDestinationPort();

                            // Check if origin port matches the current port
                            if (od.GetOriginPort() == port) {
                                // In-Z
                                for (int k = 0; k < od.GetNumberOfEmptyPath(); ++k) {
                                    if (destPort == port && arc.GetDestinationTime() <= t && arc.GetDestinationTime() >= 1) {
                                        int j = od.GetEmptyPathIndexes()[k];
                                        left += p_.GetArcAndPath()[nn][j] * zVar_[i][k];
                                    }
                                }

                                // Out-X
                                for (int k = 0; k < od.GetNumberOfLadenPath(); ++k) {
                                    if (originPort == port && arc.GetOriginTime() <= t && arc.GetOriginTime() >= 1) {
                                        int j = od.GetLadenPathIndexes()[k];
                                        left += -p_.GetArcAndPath()[nn][j] * xVar_[i][k];
                                    }
                                }
                            }
                            else if (od.GetDestinationPort() == port) {
                                // In-X
                                for (int k = 0; k < od.GetNumberOfLadenPath(); ++k) {
                                    if (destPort == port && arc.GetDestinationTime() <= t - p_.GetTurnOverTimeSet()[pp] && arc.GetDestinationTime() >= 1) {
                                        int j = od.GetLadenPathIndexes()[k];
                                        left += p_.GetArcAndPath()[nn][j] * xVar_[i][k];
                                    }
                                }

                                // Out-Z
                                for (int k = 0; k < od.GetNumberOfEmptyPath(); ++k) {
                                    if (originPort == port && arc.GetOriginTime() <= t && arc.GetOriginTime() >= 1) {
                                        int j = od.GetEmptyPathIndexes()[k];
                                        left += -p_.GetArcAndPath()[nn][j] * zVar_[i][k];
                                    }
                                }
                            }
                        }
                    }
                    std::string constr_name = "Flow[" + std::to_string(pp + 1) + "][" + std::to_string(t) + "]";
                    IloRange c4(env_, -p_.GetInitialEmptyContainer()[pp], left, IloInfinity);
                    c4.setName(constr_name.c_str());
                    model_.add(c4);  // Add the constraint to the model
                }
            }
        }
        catch (const IloException& e) {
            std::cerr << "Error in SetConstraint4: " << e.getMessage() << std::endl;
        }
    }



    void DetermineModelReactive::SolveModel() {
        try {
            if (cplex_.solve()) {
                SetObjVal(cplex_.getObjValue());

                // Create and initialize the vvv array for vessel routes
                std::vector<std::vector<int>> vvv(p_.GetVesselSet().size(), std::vector<int>(p_.GetVesselRouteSet().size(), 0));
                for (size_t i = 0; i < p_.GetVesselRouteSet().size(); ++i) {
                    for (size_t j = 0; j < p_.GetVesselSet().size(); ++j) {
                        vvv[j][i] = static_cast<int>(cplex_.getValue(vVar_[j][i]) + 0.5);
                    }
                }

                // Create and initialize the vvv2 array for vessel paths
                std::vector<std::vector<int>> vvv2(p_.GetVesselSet().size(), std::vector<int>(p_.GetVesselPathSet().size(), 0));
                for (size_t w = 0; w < p_.GetVesselPathSet().size(); ++w) {
                    for (size_t j = 0; j < p_.GetVesselSet().size(); ++j) {
                        vvv2[j][w] = static_cast<int>(cplex_.getValue(vVar2_[j][w]) + 0.5);
                    }
                }

                SetVVarValue(vvv);
                SetVVarValue2(vvv2);

                SetMipGap(cplex_.getMIPRelativeGap());

                if (WhetherPrintProcess) {
                    PrintSolution();
                }
            }
            else {
                std::cout << "No solution" << std::endl;
            }
            cplex_.end();
        }
        catch (const IloException& ex) {
            std::cerr << "Concert Error: " << ex.getMessage() << std::endl;
        }
    }


    void DetermineModelReactive::PrintSolution() {
        std::cout << "Master Objective = " << std::fixed << std::setprecision(2) << GetObjVal() << std::endl;
        std::cout << "Vessel Decision vVar (MP) : ";
        for (size_t r = 0; r < p_.GetVesselRouteSet().size(); ++r) {
            std::cout << p_.GetVesselRouteSet()[r] << "(";
            for (size_t h = 0; h < p_.GetVesselSet().size(); ++h) {
                if (vVarValue_[h][r] != 0) {
                    std::cout << p_.GetVesselSet()[h] << ")\t";
                }
            }
        }
        std::cout << std::endl;
        std::cout << "Reactive Decision vVar2 (MP) : ";
        for (size_t w = 0; w < p_.GetVesselPathSet().size(); ++w) {
            for (size_t h = 0; h < p_.GetVesselSet().size(); ++h) {
                if (vVarValue2_[h][w] != 0) {
                    std::cout << p_.GetVesselPathSet()[w] << "(" << p_.GetVesselSet()[h] << ")\t";
                }
            }
        }
        std::cout << std::endl;
    }



}