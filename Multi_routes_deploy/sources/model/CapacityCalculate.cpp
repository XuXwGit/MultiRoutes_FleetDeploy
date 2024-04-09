#include "CapacityCalculate.h"

namespace fleetdeployment {


    CapacityCalculate::CapacityCalculate(const InputData& in, const Parameter& p)
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

            // create basic decision and add basic constraints
            SetDecisionVars();
            SetObjectives();
            SetConstraints();

            SolveModel();

        }
        catch (const IloException& e) {
            std::cerr << "Error: " << e.getMessage() << std::endl;
        }
    }

    void CapacityCalculate::SetDecisionVars()
    {
        v_var_.resize(p_.GetVesselRouteSet().size(), std::vector<IloNumVar>(p_.GetVesselPathSet().size()));
        x_var_.resize(p_.GetDemand().size());
        y_var_.resize(p_.GetDemand().size());
        z_var_.resize(p_.GetDemand().size());
        g_var_.resize(p_.GetDemand().size());


        for (size_t w = 0; w < p_.GetVesselPathSet().size(); ++w) {
            int r = in_.GetVesselPathSet()[w].GetRouteID() - 1;
            std::string var_name = "v(" + std::to_string(r + 1) + ")(" + std::to_string(w + 1) + ")";
            v_var_[r][w] = IloNumVar(env_, 0, IloInfinity, var_name.c_str());
        }

        for (size_t i = 0; i < p_.GetDemand().size(); ++i) {
            const Request& od = in_.GetRequests()[i];

            std::vector<IloNumVar> xxxVar_k(od.GetNumberOfLadenPath());
            std::vector<IloNumVar> yyyVar_k(od.GetNumberOfLadenPath());
            std::vector<IloNumVar> zzzVar_k(od.GetNumberOfEmptyPath());

            for (size_t j = 0; j < od.GetNumberOfLadenPath(); ++j) {
                std::string var_name_x = "x(" + std::to_string(i + 1) + ")";
                xxxVar_k[j] = IloNumVar(env_, 0, IloInfinity, var_name_x.c_str());
                std::string var_name_y = "y(" + std::to_string(i + 1) + ")";
                yyyVar_k[j] = IloNumVar(env_, 0, IloInfinity, var_name_y.c_str());
            }
            for (size_t j = 0; j < od.GetNumberOfEmptyPath(); ++j) {
                std::string var_name_z = "z(" + std::to_string(i + 1) + ")";
                zzzVar_k[j] = IloNumVar(env_, 0, IloInfinity, var_name_z.c_str());
            }

            x_var_[i] = std::move(xxxVar_k);
            y_var_[i] = std::move(yyyVar_k);
            z_var_[i] = std::move(zzzVar_k);

            std::string var_name_g = "g(" + std::to_string(i + 1) + ")";
            g_var_[i] = IloNumVar(env_, 0, IloInfinity, var_name_g.c_str());
        }
    }

    void CapacityCalculate::SetObjectives()
    {
        try {

            IloExpr  expr(env_);

            // Item 1: Operating Cost
            for (size_t w = 0; w < p_.GetVesselPathSet().size(); ++w) {
                int r = in_.GetVesselPathSet()[w].GetRouteID() - 1;
                expr += p_.GetShipRouteAndVesselPath()[r][w] * v_var_[r][w];
            }

            // Loop over demands
            for (size_t i = 0; i < p_.GetDemand().size(); ++i) {
                // Item 2: Penalty Cost of unsatisfied Demand
                expr += p_.GetPenaltyCostForDemand()[i] * g_var_[i];

                const Request& od = in_.GetRequests()[i];

                // Laden paths
                for (size_t k = 0; k < od.GetNumberOfLadenPath(); ++k) {
                    int j = od.GetLadenPathIndexes()[k];
                    // Item 3: Demurrage of self-owned and leased containers, and Rental cost on laden paths
                    expr += p_.GetLadenPathDemurrageCost()[j] * x_var_[i][k];
                    expr += p_.GetLadenPathDemurrageCost()[j] * y_var_[i][k];
                    expr += p_.GetRentalCost() * p_.GetTravelTimeOnPath()[j] * y_var_[i][k];
                }

                // Empty paths
                for (size_t k = 0; k < od.GetNumberOfEmptyPath(); ++k) {
                    int j = od.GetEmptyPathIndexes()[k];
                    // Item 4: Demurrage of self-owned containers for empty path
                    expr += p_.GetEmptyPathDemurrageCost()[j] * z_var_[i][k];
                }
            }

            obj_ = IloMinimize(env_, expr);
            model_.add(obj_);
        }
        catch (const IloException& e) {
            std::cerr << "Error in SetObjectives: " << e.getMessage() << std::endl;
        }
    }

    void CapacityCalculate::SetConstraints()
    {
        SetConstraint1();
        SetConstraint2();
        SetConstraint3();
    }

    void CapacityCalculate::SetConstraint1() {
        try {
            c1_.resize(p_.GetDemand().size());

            // 对于每个 i ∈ I
            for (size_t i = 0; i < p_.GetDemand().size(); ++i) {
                IloExpr left(env_);

                const Request& od = in_.GetRequests()[i];
                // φ
                for (size_t k = 0; k < od.GetNumberOfLadenPath(); ++k) {
                    left += x_var_[i][k];
                    left += y_var_[i][k];
                }

                left += g_var_[i];

                std::string constr_name = "C1(" + std::to_string(i + 1) + ")";
                c1_[i] = IloRange(p_.GetDemand()[i] <= left <= p_.GetDemand()[i]);
                c1_[i].setName(constr_name.c_str());
                model_.add(c1_[i]);
            }
        }
        catch (const IloException& e) {
            std::cerr << "Error in SetConstraint1: " << e.getMessage() << std::endl;
        }
    }

    void CapacityCalculate::SetConstraint2() {
        try {
            // 对于每个 <n,n'> ∈ A'
            for (size_t nn = 0; nn < p_.GetTravelArcsSet().size(); ++nn)
            {
                std::string constr_name = "C2(" + std::to_string(nn + 1) + ")";

                IloExpr left(env_);

                // 对于每个 i ∈ I
                for (size_t i = 0; i < p_.GetDemand().size(); ++i) {
                    Request od = in_.GetRequests()[i];

                    // φ
                    for (size_t k = 0; k < od.GetNumberOfLadenPath(); ++k) {
                        int j = od.GetLadenPathIndexes()[k];
                        left += p_.GetArcAndPath()[nn][j] * x_var_[i][k];
                        left += p_.GetArcAndPath()[nn][j] * y_var_[i][k];
                    }

                    //θ
                    for (size_t k = 0; k < od.GetNumberOfEmptyPath(); ++k) {
                        int j = od.GetEmptyPathIndexes()[k];
                        left += p_.GetArcAndPath()[nn][j] * z_var_[i][k];
                    }
                }

                //Vessel capacity
                for (size_t w = 0; w < p_.GetVesselPathSet().size(); ++w)
                {
                    int r = in_.GetVesselPathSet()[w].GetRouteID() - 1;
                    left -= p_.GetArcAndVesselPath()[nn][w] * p_.GetShipRouteAndVesselPath()[r][w] * v_var_[r][w];
                }

                IloRange c2 = IloRange(left <= 0);
                c2.setName(constr_name.c_str());
                model_.add(c2);
            }
        }
        catch (const IloException& e) {
            std::cerr << "Error in SetConstraint2: " << e.getMessage() << std::endl;
        }
    }


    void CapacityCalculate::SetConstraint3() {
        try {
            // 对于每个 p ∈ P
            for (size_t pp = 0; pp < p_.GetPortSet().size(); ++pp) {
                // 对于每个 t ∈ T
                for (size_t t = 1; t < p_.GetTimePointSet().size(); ++t) {
                    IloExpr left(env_);

                    // 对于每个 i ∈ I
                    for (size_t i = 0; i < p_.GetDemand().size(); ++i) {
                        const Request& od = in_.GetRequests()[i];

                        // 输入流 Z
                        if (p_.GetOriginOfDemand()[i] == p_.GetPortSet()[pp]) {
                            for (size_t k = 0; k < od.GetNumberOfEmptyPath(); ++k) {
                                int j = od.GetEmptyPathIndexes()[k];
                                for (size_t nn = 0; nn < p_.GetTravelArcsSet().size(); ++nn) {
                                    if (in_.GetTravelArcs()[nn].GetDestinationPort() == p_.GetPortSet()[pp] &&
                                        in_.GetTravelArcs()[nn].GetDestinationTime() <= t &&
                                        in_.GetTravelArcs()[nn].GetDestinationTime() >= 1) {
                                        left += p_.GetArcAndPath()[nn][j] * z_var_[i][k];
                                    }
                                }
                            }
                        }

                        // 输入流 X
                        if (p_.GetDestinationOfDemand()[i] == p_.GetPortSet()[pp]) {
                            for (size_t k = 0; k < od.GetNumberOfLadenPath(); ++k) {
                                int j = od.GetLadenPathIndexes()[k];
                                for (size_t nn = 0; nn < p_.GetTravelArcsSet().size(); ++nn) {
                                    if (in_.GetTravelArcs()[nn].GetDestinationPort() == p_.GetPortSet()[pp] &&
                                        in_.GetTravelArcs()[nn].GetDestinationTime() <= t - p_.GetTurnOverTimeSet()[pp] &&
                                        in_.GetTravelArcs()[nn].GetDestinationTime() >= 1) {
                                        left += p_.GetArcAndPath()[nn][j] * x_var_[i][k];
                                    }
                                }
                            }
                        }

                        // 输出流 X
                        if (p_.GetOriginOfDemand()[i] == p_.GetPortSet()[pp]) {
                            for (size_t k = 0; k < od.GetNumberOfLadenPath(); ++k) {
                                int j = od.GetLadenPathIndexes()[k];
                                for (size_t nn = 0; nn < p_.GetTravelArcsSet().size(); ++nn) {
                                    if (in_.GetTravelArcs()[nn].GetOriginPort() == p_.GetPortSet()[pp] &&
                                        in_.GetTravelArcs()[nn].GetOriginTime() <= t &&
                                        in_.GetTravelArcs()[nn].GetOriginTime() >= 1) {
                                        left -= p_.GetArcAndPath()[nn][j] * x_var_[i][k];
                                    }
                                }
                            }
                        }

                        // 输出流 Z
                        for (size_t k = 0; k < od.GetNumberOfEmptyPath(); ++k) {
                            int j = od.GetEmptyPathIndexes()[k];
                            for (size_t nn = 0; nn < p_.GetTravelArcsSet().size(); ++nn) {
                                if (in_.GetTravelArcs()[nn].GetOriginPort() == p_.GetPortSet()[pp] &&
                                    in_.GetTravelArcs()[nn].GetOriginTime() <= t &&
                                    in_.GetTravelArcs()[nn].GetOriginTime() >= 1) {
                                    left -= p_.GetArcAndPath()[nn][j] * z_var_[i][k];
                                }
                            }
                        }
                    }

                    std::string constr_name = "C3(" + std::to_string(pp + 1) + ")(" + std::to_string(t) + ")";
                    IloRange c3 = IloRange(left >= -p_.GetInitialEmptyContainer()[pp]);
                    c3.setName(constr_name.c_str());
                    model_.add(c3);
                }
            }
        }
        catch (const IloException& e) {
            std::cerr << "Error in SetConstraint3: " << e.getMessage() << std::endl;
        }
    }


    void CapacityCalculate::SolveModel() {
        try {
            if (cplex_.solve()) {
                // 可能的详细打印函数
                // PrintDetail();
                SetObjVal(cplex_.getObjValue());
                SetMinV();
                ChangeDemandVariation();
                if (cplex_.solve()) {
                    // 可能的详细打印函数
                    // PrintDetail();
                    SetMaxV();
                }
                else {
                    std::cout << "Exit with an error during re-solving" << std::endl;
                }
                PrintSolution();
            }
            else {
                std::cout << "No Solution" << std::endl;
            }
        }
        catch (const IloException& e) {
            std::cerr << "Concert Error: " << e.getMessage() << std::endl;
        }
    }


    void CapacityCalculate::ChangeDemandVariation() {
        try {
            for (size_t i = 0; i < p_.GetDemand().size(); ++i) {
                double newBound = p_.GetDemand()[i] + p_.GetMaximumDemandVariation()[i];
                c1_[i].setBounds(newBound, newBound);
            }
        }
        catch (const IloException& e) {
            std::cerr << "Error in ChangeDemandVariation: " << e.getMessage() << std::endl;
        }
    }


    void CapacityCalculate::SetMinV() {
        try {
            min_v_.resize(p_.GetVesselRouteSet().size(), std::vector<int>(p_.GetVesselPathSet().size()));

            for (size_t w = 0; w < p_.GetVesselPathSet().size(); ++w) {
                int r = in_.GetVesselPathSet()[w].GetRouteID() - 1;
                min_v_[r][w] = static_cast<int>(cplex_.getValue(v_var_[r][w]));
            }
        }
        catch (const IloException& e) {
            std::cerr << "Error in SetMinV: " << e.getMessage() << std::endl;
        }
    }


    void CapacityCalculate::SetMaxV() {
        try {
            max_v_.resize(p_.GetVesselRouteSet().size(), std::vector<int>(p_.GetVesselPathSet().size()));

            for (size_t w = 0; w < p_.GetVesselPathSet().size(); ++w) {
                int r = in_.GetVesselPathSet()[w].GetRouteID() - 1;
                max_v_[r][w] = static_cast<int>(cplex_.getValue(v_var_[r][w]));
            }
        }
        catch (const IloException& e) {
            std::cerr << "Error in SetMaxV: " << e.getMessage() << std::endl;
        }
    }

    void CapacityCalculate::PrintSolution() {
        std::cout << "Vessel Decision vVar : " << std::endl;
        for (size_t r = 0; r < p_.GetVesselRouteSet().size(); ++r) {
            std::cout << p_.GetVesselRouteSet()[r] << ":\t";
            for (size_t w = 0; w < p_.GetVesselPathSet().size(); ++w) {
                if (p_.GetShipRouteAndVesselPath()[r][w] != 0) {
                    std::cout << p_.GetVesselPathSet()[w];
                    std::cout << "(" << min_v_[r][w] << "~" << max_v_[r][w] << ")\t";
                }
            }
            std::cout << std::endl;
        }
        std::cout << std::endl;
    }

    void CapacityCalculate::PrintDetail() {
        try {
            for (size_t i = 0; i < p_.GetDemand().size(); ++i) {
                const Request& od = in_.GetRequests()[i];
                std::cout << "Demand" << (i + 1) << "("
                    << od.GetOriginPort() << "->" << od.GetDestinationPort() << ")"
                    << "(" << od.GetW_i_Earliest() << "->" << od.GetLatestDestinationTime() << ")"
                    << ":\t" << p_.GetDemand()[i] << " \t=\t";

                double totalX = 0;
                double totalY = 0;
                // φ
                for (size_t k = 0; k < od.GetNumberOfLadenPath(); ++k) {
                    totalX += cplex_.getValue(x_var_[i][k]);
                    totalY += cplex_.getValue(y_var_[i][k]);
                }

                std::cout << totalX << "\t\t" << totalY << "\t\t" << cplex_.getValue(g_var_[i]) << "\n";
            }
            std::cout << std::endl;
        }
        catch (const IloException& e) {
            std::cerr << "Error in PrintDetail: " << e.getMessage() << std::endl;
        }
    }


}