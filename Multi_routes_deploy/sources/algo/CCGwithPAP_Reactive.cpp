#include "CCGwithPAP_Reactive.h"

namespace fleetdeployment {

    void CCGwithPAP_Reactive::frame() {
        if (WhetherPrintProcess || WhetherPrintIteration) {
            std::cout << "=========C&CG with PAP (Reactive Strategy) : ===========" << std::endl;
        }

        std::string filename = "CCG-PAP-Reactive" + std::to_string(p_.GetTimePointSet().size()) + ".txt";
        std::ofstream fileWriter(filename, std::ios::app);

        if (!fileWriter.is_open()) {
            std::cerr << "Error: Unable to open file " << filename << std::endl;
        }
        time_t start = time(0);

        std::vector<double> maxDemandVar = p_.GetMaximumDemandVariation();
        // change  MaxVarDemand
        // beta = min(k, m/k)
        double beta = (double)p_.GetDemand().size() / (double)tau;
        if (beta > tau)
        {
            beta = tau;
        }
        p_.changeMaximunDemandVariation(beta);

        SetUpperBound(1e12);
        SetLowerBound(-1e12);

        // add initial scene to make model feasible
        std::vector<Scenario> sce;

        // vvv ¡ª¡ª the solution of master problem
        std::vector<std::vector<int>> vvv(p_.GetVesselSet().size(), std::vector<int>(p_.GetVesselRouteSet().size()));
        double masterObjective;
        int flag = 0;
        int max_iteration = 100;
        std::vector<double> upper(max_iteration + 1);
        std::vector<double> lower(max_iteration + 1);
        std::vector<double> masterObj(max_iteration + 1);
        std::vector<double> subObj(max_iteration + 1);
        upper[0] = upperBound;
        lower[0] = lowerBound;
        int iteration = 0;
        double gap = std::numeric_limits<double>::max();

        time_t time0 = time(0);
        MasterProblem mp(in_, p_, true);
        DualSubProblemReactive dsp(in_, p_, 1);
        Initialize(sce);

        if (WhetherPrintProcess) {
            std::cout << "=========C&CG with PAP-Reactive: ===========" << std::endl;
            std::cout << "BuildModelTime = "
                << std::fixed << std::setprecision(2) << time(0) - time0 << std::endl;
            std::cout << "k" << "\t\t"
                << "UB" << "\t\t"
                << "LB" << "\t\t"
                << "DSP-SolveTime" << "\t\t"
                << "MP-SolveTime" << "\t\t"
                << "Total Time" << "\t\t"
                << "DSP-Status" << "\t\t"
                << "MP-Status" << std::endl;
            std::cout << iteration << "\t\t"
                << std::fixed << std::setprecision(2) << upper[iteration] << "\t\t"
                << std::fixed << std::setprecision(2) << lower[iteration] << std::endl;
        }

        if (WhetherOutputLog) {
            fileWriter << "k" << "\t\t"
                << "UB" << "\t\t"
                << "LB" << "\t\t"
                << "DSP-SolveTime" << "\t\t"
                << "MP-SolveTime" << "\t\t"
                << "Total Time" << "\t\t"
                << "DSP-Status" << "\t\t"
                << "MP-Status" << std::endl;
            fileWriter << iteration << "\t\t"
                << std::fixed << std::setprecision(2) << upper[iteration] << "\t\t\t"
                << std::fixed << std::setprecision(2) << lower[iteration] << std::endl;
        }

        while (upperBound - lowerBound > MIPGapLimit
            && gap > MIPGapLimit
            && flag == 0
            && iteration < max_iteration)
        {
            // build and solve master model
            // add new scene to Master Problem
            mp.addReactiveScene(sce[iteration]);

            time_t start1 = time(0);
            mp.SolveReactiveModel();
            time_t end1 = time(0);

            // Get the solution
            masterObjective = mp.GetObjVal();
            this->SetOperationCost(mp.GetOperationCost());
            this->SetTotalCost(mp.GetObjVal());
            // check if the mp-solution changed
            double temp = 0;
            for (int r = 0; r < p_.GetVesselRouteSet().size(); r++)
            {
                for (int h = 0; h < p_.GetVesselSet().size(); h++)
                {
                    temp = temp + abs(mp.GetVVarValue()[h][r] - vvv[h][r]);
                    vvv[h][r] = mp.GetVVarValue()[h][r];
                }
            }
            // master solution has no change
            if (temp < 0.5)
            {
                flag = 1;
            }

            // update lower bound: LB = max {LB, Obj*}
            if (masterObjective > lowerBound) {
                SetLowerBound(masterObjective);
            }

            dsp.changeObjectiveVCoefficients(mp.GetVVarValue(), mp.GetVVarValue2());
            time_t start2 = time(0);
            dsp.SolveModel();
            time_t end2 = time(0);

            sce.push_back(dsp.GetScene());

            SubProblemReactive sp(in_, p_);
            sp.changeConstraintCoefficients(mp.GetVVarValue(), mp.GetVVarValue2(), dsp.GetScene().GetRequest());
            sp.solveModel();
            sp.end();
            // std::cout<<"SP-Obj = "+sp.GetObjective());

            if (sp.GetObjVal() + mp.GetOperationCost() < upperBound)
            {
                SetUpperBound(sp.GetObjVal() + mp.GetOperationCost());
            }

            iteration = iteration + 1;
            upper[iteration] = upperBound;
            lower[iteration] = lowerBound;
            masterObj[iteration] = masterObjective;
            subObj[iteration] = dsp.GetObjVal();
            gap = upper[iteration] - lower[iteration];

            if (WhetherPrintProcess || WhetherPrintIteration) {
                std::cout << iteration << "\t\t"
                    << std::fixed << std::setprecision(2) << upper[iteration] << "\t\t"
                    << std::fixed << std::setprecision(2) << lower[iteration] << "\t\t"
                    << std::fixed << std::setprecision(2) << difftime(end2, start2) << "\t\t"
                    << std::fixed << std::setprecision(2) << difftime(end1, start1) << "\t\t"
                    << dsp.GetSolveStatusString() << "(" << std::fixed << std::setprecision(4) << dsp.GetMipGap() << ")\t\t"
                    << mp.GetSolveStatusString() << "(" << std::fixed << std::setprecision(4) << mp.GetMipGap() << ")\n";
            }
            if (WhetherOutputLog) {
                fileWriter << iteration << "\t\t"
                    << std::fixed << std::setprecision(2) << upper[iteration] << "\t\t"
                    << std::fixed << std::setprecision(2) << lower[iteration] << "\t\t"
                    << std::fixed << std::setprecision(2) << difftime(end2, start2) << "\t\t"
                    << std::fixed << std::setprecision(2) << difftime(end1, start1) << "\t\t"
                    << dsp.GetSolveStatusString() << "(" << std::fixed << std::setprecision(4) << dsp.GetMipGap() << ")\t\t"
                    << mp.GetSolveStatusString() << "(" << std::fixed << std::setprecision(4) << mp.GetMipGap() << ")\n";
            }

        }

        SetObj(upperBound);
        SetTotalCost(upperBound);
        SetOperationCost(mp.GetObjVal() - mp.GetEtaValue());
        SetIter(iteration);
        SetVValue(mp.GetVVarValue());
        SetVValue2(mp.GetVVarValue2());

        Gap = (upperBound - lowerBound) / lowerBound;

        mp.end();
        dsp.end();

        time_t end = time(0);
        SetSolveTime(difftime(end, start));

        if (WhetherPrintProcess || WhetherPrintIteration) {
            std::cout << "CCG&PAP(Reactive Strategy) SolveTime = " << difftime(end, start) << std::endl;
            std::cout << "==================================" << std::endl;
        }

        if (WhetherOutputLog) {
            fileWriter << "CCG&PAP(Reactive Strategy) SolveTime = " << difftime(end, start) << std::endl;
        }
        p_.SetMaximumDemandVariation(maxDemandVar);
    }

    void CCGwithPAP_Reactive::Initialize(std::vector<Scenario>& sce)
    {
        std::vector<double> sss(in_.GetRequests().size());

        double b1 = (double)tau / (double)(p_.GetDemand().size());

        // beta = min{k , I/k}
        double beta = (double)p_.GetDemand().size() / (double)tau;
        if (beta > tau)
        {
            beta = tau;
        }

        for (int i = 0; i < p_.GetDemand().size(); i++)
        {
            sss[i] = b1 / beta;
        }

        Scenario sp;
        sp.SetRequest(sss);
        sce.push_back(sp);
    }

    void CCGwithPAP_Reactive::PrintSolution() {
        std::cout << "Master Objective =" << std::fixed << std::setprecision(2) << obj << std::endl;

        std::cout << "Vessel Decision vVar (MP) : " << std::endl;
        for (int r = 0; r < p_.GetVesselRouteSet().size(); r++)
        {
            std::cout << p_.GetVesselRouteSet()[r] << ":\t";
            for (int h = 0; h < p_.GetVesselSet().size(); h++)
            {
                if (vValue[h][r] != 0)
                {
                    std::cout << "(" << p_.GetVesselSet()[h] << ")\t";
                }
            }
        }
        std::cout << std::endl;

        std::cout << "Reactive Decision vVar2 (MP) : " << std::endl;
        for (int r = 0; r < p_.GetVesselRouteSet().size(); r++)
        {
            std::cout << p_.GetVesselRouteSet()[r] << ":\t";
            for (int h = 0; h < p_.GetVesselSet().size(); h++)
            {
                if (vValue2[h][r] != 0)
                {
                    std::cout << "(" << p_.GetVesselSet()[h] << ")\t";
                }
            }
        }
        std::cout << std::endl;
    }



}