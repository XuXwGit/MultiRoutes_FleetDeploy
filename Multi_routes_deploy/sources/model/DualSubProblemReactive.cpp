#include "DualSubProblemReactive.h"

namespace fleetdeployment {


    DualSubProblemReactive::DualSubProblemReactive(const InputData& inputData, const Parameter& parameter, int tau)
        : in_(inputData), p_(parameter), tau_(tau), 
        vVarValue1_(p_.GetVesselSet().size(), std::vector<int>(parameter.GetVesselRouteSet().size(), 0)),
        vVarValue2_(parameter.GetVesselSet().size(), std::vector<int>(parameter.GetVesselPathSet().size(), 0)) {
        try {

            env_ = IloEnv();
            model_ = IloModel(env_);
            cplex_ = IloCplex(model_);

            cplex_.setOut(env_.getNullStream());
            cplex_.setParam(IloCplex::WorkMem, MaxWorkMem);
            cplex_.setParam(IloCplex::TiLim, MIPTimeLimit);
            cplex_.setParam(IloCplex::EpGap, MIPGapLimit);
            cplex_.setParam(IloCplex::Threads, MaxThreads);

            SetDecisionVars();
            SetObjective();
            SetConstraints();
        }
        catch (const IloException& e) {
            std::cerr << "Error: " << e.getMessage() << std::endl;
        }
    }


    void DualSubProblemReactive::SetDecisionVars() {
        // create dual variable
        // α[i]
        //β[nn']
        //γ[p][t]
        alphaVar_.resize(p_.GetDemand().size());
        betaVar1_.resize(p_.GetTravelArcsSet().size());
        betaVar2_.resize(p_.GetTravelArcsSet().size());
        gammaVar_.resize(p_.GetPortSet().size(), std::vector<IloNumVar>(p_.GetTimePointSet().size()));

        // u[i] ∈{0,1}
        // the uncertain request can be inferred by u
        // f[i] = nf[i] + vf[i]*u[i]
        miuVar_.resize(p_.GetDemand().size());

        // create auxiliary variable
        //λ[i]=α[i]*u[i]
        lambdaVar_.resize(p_.GetDemand().size());

        std::string varName;
        for (int i = 0; i < p_.GetDemand().size(); i++)
        {
            varName = "alpha(" + std::to_string(p_.GetDemand()[i]) + ")";
            alphaVar_[i] = IloNumVar(env_, -IloInfinity, p_.GetPenaltyCostForDemand()[i], varName.c_str());

            varName = "lambda(" + std::to_string(p_.GetDemand()[i]) + ")";
            lambdaVar_[i] = IloNumVar(env_, -IloInfinity, IloInfinity, varName.c_str());

            varName = "u(" + std::to_string(p_.GetDemand()[i]) + ")";
            miuVar_[i] = IloBoolVar(env_, varName.c_str());
        }

        for (int nn = 0; nn < p_.GetTravelArcsSet().size(); nn++)
        {
            // beta <= 0
            varName = "beta(" + std::to_string(p_.GetTravelArcsSet()[nn]) + ")";
            betaVar1_[nn] = IloNumVar(env_, -IloInfinity, 0, varName.c_str());
            varName = "beta2(" + std::to_string(p_.GetTravelArcsSet()[nn]) + ")";
            betaVar2_[nn] = IloNumVar(env_, -IloInfinity, 0, varName.c_str());
        }

        for (int pp = 0; pp < p_.GetPortSet().size(); pp++)
        {
            for (int t = 1; t < p_.GetTimePointSet().size(); t++)
            {
                // gamma >= 0
                varName = "gamma(" + p_.GetPortSet()[pp] + ")(" + std::to_string(p_.GetTimePointSet()[t]) + ")";
                gammaVar_[pp][t] = IloNumVar(env_, 0, IloInfinity, varName.c_str());
            }
        }
    }


    void DualSubProblemReactive::SetObjective() {
        IloExpr expr(env_);
        // I.part one : sum(normal_demand * alpha + max_var_demand*u*alpha) = sum(normal_demand * alpha + max_var_demand * lambda)
        // i ∈I
        for (int i = 0; i < p_.GetDemand().size(); i++)
        {
            expr += (p_.GetDemand()[i] * alphaVar_[i]);
            expr += (p_.GetMaximumDemandVariation()[i] * lambdaVar_[i]);
        }

        // II. sum (vessel capacity * V[h][r] * beta[arc])
        // V[h][r] : the solution come from the master problem (the only changeable input param in dual sub model)
        // <n,n'> ∈ A'
        for (int nn = 0; nn < p_.GetTravelArcsSet().size(); nn++)
        {
            double capacity1 = 0;
            double capacity2 = 0;
            // w∈Ω
            for (int w = 0; w < p_.GetVesselPathSet().size(); w++)
            {
                int r = in_.GetVesselPathSet()[w].GetRouteID() - 1;
                // r(w) = r
                // h \in Hr
                for (int h = 0; h < p_.GetVesselSet().size(); h++)
                {
                    // vValue[v][r] : come from solution of master problem
                    capacity1 += p_.GetArcAndVesselPath()[nn][w]
                        * p_.GetShipRouteAndVesselPath()[r][w]
                        * p_.GetVesselTypeAndShipRoute()[h][r]
                        * p_.GetVesselCapacity()[h]
                        * vVarValue1_[h][r];

                    // vValue[v][w] : come from solution of master problem
                    capacity2 += p_.GetArcAndVesselPath()[nn][w]
                        * p_.GetShipRouteAndVesselPath()[r][w]
                        * p_.GetVesselTypeAndShipRoute()[h][r]
                        * p_.GetVesselCapacity()[h]
                        * vVarValue2_[h][w];
                }
            }
            expr += (capacity1 * betaVar1_[nn]);
            expr += (capacity2 * betaVar2_[nn]);
        }

        // III. part three:
        // p∈P
        for (int pp = 0; pp < p_.GetPortSet().size(); pp++)
        {
            //t∈ T
            for (int t = 1; t < p_.GetTimePointSet().size(); t++)
            {
                expr += (-p_.GetInitialEmptyContainer()[pp] * gammaVar_[pp][t]);
            }
        }

        obj_ = IloMaximize(env_, expr);
        model_.add(obj_);
    }

    void DualSubProblemReactive::changeObjectiveVCoefficients(const std::vector<std::vector<int>>& vValue1, const std::vector<std::vector<int>>& vValue2) {
        this->vVarValue1_ = vValue1;
        this->vVarValue2_ = vValue2;
        // II. sum (vessel capacity * V[h][r] * beta[arc])
        // V[h][r] : the solution come from the master problem (the only changeable input param in dual sub model)
        // <n,n'> ∈ A'
        for (int n = 0; n < p_.GetTravelArcsSet().size(); n++) {
            double capacity1 = 0;
            double capacity2 = 0;
            // w∈Ω
            for (int w = 0; w < p_.GetVesselPathSet().size(); w++)
            {
                int r = in_.GetVesselPathSet()[w].GetRouteID() - 1;
                // r(w) = r
                // h \in Hr
                for (int h = 0; h < p_.GetVesselSet().size(); h++)
                {
                    // vValue[v][r] : come from solution of master problem
                    capacity1 += p_.GetArcAndVesselPath()[n][w]
                        * p_.GetShipRouteAndVesselPath()[r][w]
                        * p_.GetVesselTypeAndShipRoute()[h][r]
                        * p_.GetVesselCapacity()[h]
                        * vVarValue1_[h][r];
                    // vValue[v][w] : come from solution of master problem
                    capacity2 += p_.GetArcAndVesselPath()[n][w]
                        * p_.GetShipRouteAndVesselPath()[r][w]
                        * p_.GetVesselTypeAndShipRoute()[h][r]
                        * p_.GetVesselCapacity()[h]
                        * vVarValue2_[h][w];
                }
            }
            obj_.setLinearCoef(betaVar1_[n], capacity1);
            obj_.setLinearCoef(betaVar2_[n], capacity2);
        }
    }

    void DualSubProblemReactive::SetConstraints() {
        // dual constraints
        SetConstraint1();
        SetConstraint2();
        SetConstraint3();
        SetConstraint4();

        // uncertain Set
        SetConstraint5();

        // linearize constraints
        // λ<=α
        SetConstraint6();
        //λ>= α-(1-u)M
        SetConstraint7();
        // λ<= M*u
        SetConstraint8();
        // λ>=- M*u
        SetConstraint9();
    }

    // C1------X
    void DualSubProblemReactive::SetConstraint1() {
        //  ∀i∈I
        for (int i = 0; i < p_.GetDemand().size(); i++) {
            // ∀φ∈Φi
            Request OD = in_.GetRequests()[i];
            for (int k1 = 0; k1 < OD.GetNumberOfLadenPath(); k1++) {
                int j = OD.GetLadenPathIndexes()[k1];

                IloExpr left(env_);

                // first item :
                left += (1, alphaVar_[i]);

                // second item :
                // <n,n'> ∈A'
                for (int n = 0; n < p_.GetTravelArcsSet().size(); n++) {
                    left += (p_.GetArcAndPath()[n][j] * betaVar1_[n]);
                }

                // third item :
                // t∈T
                for (int t = 1; t < p_.GetTimePointSet().size(); t++) {
                    // p ∈P
                    for (int m = 0; m < p_.GetPortSet().size(); m++)
                    {
                        // p == d(i)
                        if (p_.GetPortSet()[m] == (p_.GetDestinationOfDemand()[i]))
                        {
                            // <n,n'>∈A'
                            for (int n = 0; n < p_.GetTravelArcsSet().size(); n++) {
                                // p(n') == p
                                // 1 <= t(n') <= t - sp
                                if (in_.GetTravelArcs()[n].GetDestinationPort() == (p_.GetPortSet()[m])
                                    && in_.GetTravelArcs()[n].GetDestinationTime() <= t - p_.GetTurnOverTimeSet()[m]
                                    && in_.GetTravelArcs()[n].GetDestinationTime() >= 1) {
                                    left += (p_.GetArcAndPath()[n][j] * gammaVar_[m][t]);
                                }
                            }
                        }
                        // p == o(i)
                        if (p_.GetPortSet()[m] == (p_.GetOriginOfDemand()[i]))
                        {
                            // <n,n'>∈A'
                            for (int n = 0; n < p_.GetTravelArcsSet().size(); n++)
                            {
                                // p(n) == p
                                // 1 <= t(n) <= t
                                if (in_.GetTravelArcs()[n].GetOriginPort() == (p_.GetPortSet()[m])
                                    && in_.GetTravelArcs()[n].GetOriginTime() <= t
                                    && in_.GetTravelArcs()[n].GetOriginTime() >= 1)
                                {
                                    left += (-p_.GetArcAndPath()[n][j], gammaVar_[m][t]);
                                }
                            }
                        }
                    }
                }

                std::string constr_name = "C1_" + std::to_string(j + 1);
                IloRange c1(env_, left, p_.GetLadenPathCost()[j], constr_name.c_str());
                model_.add(c1);
            }
        }
    }

    // C2------Y
    void DualSubProblemReactive::SetConstraint2() {
        // ∀i∈I
        for (int i = 0; i < p_.GetDemand().size(); i++)
        {
            // ∀φ∈Φi
            for (int k1 = 0; k1 < in_.GetRequests()[i].GetNumberOfLadenPath(); k1++)
            {
                int j = in_.GetRequests()[i].GetLadenPathIndexes()[k1];

                IloExpr left(env_);

                // item1:
                left += (1, alphaVar_[i]);

                // <n,n'>∈A'
                for (int n = 0; n < p_.GetTravelArcsSet().size(); n++)
                {
                    left += (p_.GetArcAndPath()[n][j] * betaVar1_[n]);
                }

                // left <= c3 * g(φ) + c4φ
                std::string constr_name = "C2_" + std::to_string(i + 1) + "_" + std::to_string(j + 1);
                IloRange c2(env_, left, p_.GetRentalCost() * p_.GetTravelTimeOnPath()[j] + p_.GetLadenPathCost()[j], constr_name.c_str());
                model_.add(c2);
            }
        }
    }

    // C3------Z
    void DualSubProblemReactive::SetConstraint3() {
        // i∈I
        for (int i = 0; i < p_.GetDemand().size(); i++)
        {
            //  θ∈Θi
            for (int k = 0; k < in_.GetRequests()[i].GetNumberOfEmptyPath(); k++) {
                int j = in_.GetRequests()[i].GetEmptyPathIndexes()[k];

                IloExpr left(env_);

                // <n,n'>∈A'
                for (int n = 0; n < p_.GetTravelArcsSet().size(); n++) {
                    // add item1:
                    left += (p_.GetArcAndPath()[n][j] * betaVar2_[n]);

                    // t∈T
                    for (int t = 1; t < p_.GetTimePointSet().size(); t++) {
                        // p∈P
                        for (int pp = 0; pp < p_.GetPortSet().size(); pp++)
                        {
                            // p == o(i)
                            if (p_.GetPortSet()[pp] == (p_.GetOriginOfDemand()[i]))
                            {
                                // add item1:
                                //p(n') == p
                                // 1<=t(n')<= t
                                if (in_.GetTravelArcs()[n].GetDestinationPort() == (p_.GetPortSet()[pp])
                                    && in_.GetTravelArcs()[n].GetDestinationTime() <= t
                                    && in_.GetTravelArcs()[n].GetDestinationTime() >= 1) {
                                    left += (p_.GetArcAndPath()[n][j] * gammaVar_[pp][t]);
                                }
                            }

                            // p
                            // add item4:
                            // p(n) == p
                            // 1<= t(n)<=t
                            if (in_.GetTravelArcs()[n].GetOriginPort() == (p_.GetPortSet()[pp])
                                && in_.GetTravelArcs()[n].GetOriginTime() <= t
                                && in_.GetTravelArcs()[n].GetOriginTime() >= 1)
                            {
                                left += (-p_.GetArcAndPath()[n][j] * gammaVar_[pp][t]);
                            }
                        }
                    }
                }

                // left <= c5θ
                std::string constr_name = "C3_" + std::to_string(j + 1);
                IloRange c3(env_, left, p_.GetEmptyPathCost()[j], constr_name.c_str());
                model_.add(c3);
            }
        }
    }

    // C4------G
    void DualSubProblemReactive::SetConstraint4() {
        for (int i = 0; i < p_.GetDemand().size(); i++) {
            std::string constr_name = "C4_" + std::to_string(i + 1);
            IloRange c4(env_, alphaVar_[i], p_.GetPenaltyCostForDemand()[i]);
            c4.setName(constr_name.c_str());
            model_.add(c4);
        }
    }

    // budGet uncertain Set
    void DualSubProblemReactive::SetConstraint5() {
        IloExpr left(env_);

        for (int i = 0; i < p_.GetDemand().size(); i++) {
            left += (1 * miuVar_[i]);
        }

        IloRange budget(env_, left, tau_);
        model_.add(budget);
    }

    // λ[i] <= α[i]
    void DualSubProblemReactive::SetConstraint6() {
        for (int i = 0; i < p_.GetDemand().size(); i++)
        {
            IloExpr left(env_);
            left += (1 * lambdaVar_[i]);
            left += (-1 * alphaVar_[i]);
            model_.add(IloRange(env_, left, 0));
        }
    }

    // λ[i] >= α[i] - M*(1-u[i])
    // λ[i] - M*u[i] - α[i] >= - M
    void DualSubProblemReactive::SetConstraint7() {
        double M = 1E6;

        for (int i = 0; i < p_.GetDemand().size(); i++)
        {
            IloExpr left(env_);

            left += (1 * lambdaVar_[i]);
            left += (-M * miuVar_[i]);
            left += (-1 * alphaVar_[i]);

            model_.add(IloRange(env_, -M, left));
        }
    }

    // λ[i] <= u[i]*M
    void DualSubProblemReactive::SetConstraint8() {
        double M = 1E6;

        for (int i = 0; i < p_.GetDemand().size(); i++)
        {
            IloExpr left(env_);

            left += (1 * lambdaVar_[i]);
            left += (-M * miuVar_[i]);

            model_.add(IloRange(env_, left, 0));
        }
    }

    // λ[i] >= - u[i]*M
    void DualSubProblemReactive::SetConstraint9() {
        double M = 1E6;

        for (int i = 0; i < p_.GetDemand().size(); i++)
        {
            IloExpr left(env_);

            left += (1 * lambdaVar_[i]);
            left += (M * miuVar_[i]);

            model_.add(IloRange(env_, 0, left, IloInfinity));
        }
    }

    void DualSubProblemReactive::SolveModel() {
        try
        {
            cplex_.exportModel("dsp2.lp");

            clock_t startTime = clock();
            if (cplex_.solve())
            {
                clock_t endTime = clock();

                SetObjVal(cplex_.getObjValue());

                uVarValue_ = std::vector<int>(p_.GetDemand().size());
                std::vector<double> request = std::vector<double>(p_.GetDemand().size());
                for (int i = 0; i < p_.GetDemand().size(); i++)
                {
                    if (cplex_.getValue(miuVar_[i]) != 0)
                    {
                        request[i] = (int)(cplex_.getValue(miuVar_[i]) + 0.5);
                        uVarValue_[i] = (int)(cplex_.getValue(miuVar_[i]) + 0.5);
                    }
                }

                //                PrintSolution();

                if (DebugEnable && DualSubEnable)
                {
                    std::cout << "------------------------------------------------------------------------" << std::endl;
                    std::cout << "SolveTime = " + (endTime - startTime) << std::endl;
                    PrintSolution();
                    std::cout << "------------------------------------------------------------------------" << std::endl;
                }

                Scene_ = Scenario();
                Scene_.SetRequest(request);
            }
            else
            {
                std::cout << "DualSubProblem No Solution" << std::endl;
            }
            /*cplex_.end();*/
        }
        catch (IloException& ex) {
            std::cerr << "Concert Error: " << ex << std::endl;
        }
    }

    void DualSubProblemReactive::PrintSolution() {
        std::cout << "The Worst Case(DSP) : " << " (tau = " + std::to_string(tau_) + ")";
        for (int i = 0; i < p_.GetDemand().size(); i++)
        {
            if (uVarValue_[i] != 0)
            {
                std::cout << (i + "(" + std::to_string(uVarValue_[i]) + ")\t");
            }
        }
        std::cout << std::endl;
    }


    double DualSubProblemReactive::GetConstantItem() {
        double constantItem = 0;
        for (int n = 0; n < p_.GetTravelArcsSet().size(); n++)
        {
            // r ∈R
            for (int w = 0; w < p_.GetVesselPathSet().size(); w++)
            {
                int r = in_.GetVesselPathSet()[w].GetRouteID() - 1;
                // r(w) = r
                for (int h = 0; h < p_.GetVesselSet().size(); h++)
                {
                    // vValue[v][r] : come from solution of master problem
                    constantItem += p_.GetArcAndVesselPath()[n][w]
                        * p_.GetShipRouteAndVesselPath()[r][w]
                        * p_.GetVesselTypeAndShipRoute()[h][r]
                        * p_.GetVesselCapacity()[h]
                        * vVarValue1_[h][r]
                        * cplex_.getValue(betaVar1_[n]);

                    // vValue[v][r] : come from solution of master problem
                    constantItem += p_.GetArcAndVesselPath()[n][w]
                        * p_.GetShipRouteAndVesselPath()[r][w]
                        * p_.GetVesselTypeAndShipRoute()[h][r]
                        * p_.GetVesselCapacity()[h]
                        * vVarValue2_[h][r]
                        * cplex_.getValue(betaVar2_[n]);
                }
            }
        }

        return this->GetObjVal() - constantItem;
    }

    std::vector<double> DualSubProblemReactive::GetBeta1Value() {
        std::vector<double> beta_value(p_.GetTravelArcsSet().size());
        if (cplex_.getStatus() == IloCplex::Status::Optimal)
        {
            for (int i = 0; i < p_.GetTravelArcsSet().size(); i++)
            {
                beta_value[i] = cplex_.getValue(betaVar1_[i]);
            }
        }
        return beta_value;
    }

    std::vector<double> DualSubProblemReactive::GetBeta2Value() {
        std::vector<double> beta_value(p_.GetTravelArcsSet().size());
        if (cplex_.getStatus() == IloCplex::Status::Optimal)
        {
            for (int i = 0; i < p_.GetTravelArcsSet().size(); i++)
            {
                beta_value[i] = cplex_.getValue(betaVar2_[i]);
            }
        }
        return beta_value;
    }

    std::string DualSubProblemReactive::GetSolveStatusString() {
        if (cplex_.getStatus() == IloCplex::Status::Optimal)
            return "Optimal";
        else if (cplex_.getStatus() == IloCplex::Status::Feasible) {
            return "Feasible";
        }
        else if (cplex_.getStatus() == IloCplex::Status::Infeasible) {
            return "Infeasible";
        }
        //else if (cplex_.getStatus() == IloCplex::Status::Bounded) {
        //    return "Bounded";
        //}
        return "Others";
    }


}