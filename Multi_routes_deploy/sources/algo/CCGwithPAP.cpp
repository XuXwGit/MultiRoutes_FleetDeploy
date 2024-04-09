#include "CCGwithPAP.h"


namespace fleetdeployment
{

    void CCGwithPAP::frame() {
        std::string filename = "CCG-PAP" + std::to_string(p_.GetTimePointSet().size()) + ".txt";
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
        //        std::vector<int> worseCaseSet = new int[p_.GetDemand().size()];

        time_t time0 = time(0);

        MasterProblem mp(in_, p_);
        DualSubProblem dsp(in_, p_, 1);

        Initialize(sce);

        if (WhetherPrintProcess) {
            std::cout << "=========C&CG with PAP : ===========" << std::endl;
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


        time_t start0 = time(0);
        while (upperBound - lowerBound > MIPGapLimit
            && gap > MIPGapLimit
            && flag == 0
            && iteration < max_iteration)
        {
            // build and solve master model
            // add new scene to Master Problem
            mp.addScene(sce[iteration]);

            time_t start1 = time(0);
            mp.SolveModel();
            time_t end1 = time(0);

            // Get the solution
            masterObjective = mp.GetObjVal();
            this->SetOperationCost(mp.GetOperationCost());
            this->SetTotalCost(mp.GetObjVal());
            this->SetObj(mp.GetObjVal());
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

            dsp.changeObjectiveVCoefficients(mp.GetVVarValue());

            time_t start2 = time(0);
            dsp.SolveModel();
            time_t end2 = time(0);

            sce.push_back(dsp.GetScene());

            if (dsp.GetObjVal() + mp.GetOperationCost() < upperBound)
            {
                SetUpperBound(dsp.GetObjVal() + mp.GetOperationCost());
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
                    << std::fixed << std::setprecision(2) << difftime(time(0), start0) << "\t\t"
                    << dsp.GetSolveStatusString() << "(" << std::fixed << std::setprecision(4) << dsp.GetMipGap() << ")\t\t"
                    << mp.GetSolveStatusString() << "(" << std::fixed << std::setprecision(4) << mp.GetMipGap() << ")\n";
            }
            if (WhetherOutputLog) {
                fileWriter << iteration << "\t\t"
                    << std::fixed << std::setprecision(2) << upper[iteration] << "\t\t"
                    << std::fixed << std::setprecision(2) << lower[iteration] << "\t\t"
                    << std::fixed << std::setprecision(2) << difftime(end2, start2) << "\t\t"
                    << std::fixed << std::setprecision(2) << difftime(end1, start1) << "\t\t"
                    << std::fixed << std::setprecision(2) << difftime(time(0), start0) << "\t\t"
                    << dsp.GetSolveStatusString() << "(" << std::fixed << std::setprecision(4) << dsp.GetMipGap() << ")\t\t"
                    << mp.GetSolveStatusString() << "(" << std::fixed << std::setprecision(4) << mp.GetMipGap() << ")\n";
            }
        }

        SetObj(upperBound);
        SetTotalCost(upperBound);
        SetOperationCost(mp.GetObjVal() - mp.GetEtaValue());
        SetIter(iteration);
        SetVValue(mp.GetVVarValue());

        SetGap((upperBound - lowerBound) / lowerBound);

        if (WhetherOutputLog) {
            PrintSolution();
        }

        mp.end();
        dsp.end();

        time_t end = time(0);
        SetSolveTime(difftime(end, start));

        if (WhetherPrintProcess || WhetherPrintSolveTime) {
            std::cout << "CCG&PAP SolveTime = " << difftime(end, start) << std::endl;
            std::cout << "==================================" << std::endl;
        }

        if (WhetherOutputLog) {
            fileWriter << "CCG&PAP SolveTime = " << (difftime(end, start)) << std::endl;
        }

        p_.SetMaximumDemandVariation(maxDemandVar);

        if (CCG_PAP_Use_Sp) {
            auto start1 = time(0);
            SubProblem sp(in_, p_, dsp.GetUValueDouble());
            sp.changeConstraintCoefficients(mp.GetVVarValue(), dsp.GetUValueDouble());
            sp.solveModel();
            sp.end();

            if (WhetherPrintProcess || WhetherPrintSolveTime) {
                auto end1 = time(0);
                std::cout << "SubProblem Time = " << difftime(end1, start1) << std::endl;
            }

            SetLadenCost(sp.GetLadenCost());
            SetEmptyCost(sp.GetEmptyCost());
            SetRentalCost(sp.GetRentalCost());
            SetPenaltyCost(sp.GetPenaltyCost());
        }
    }

    void CCGwithPAP::Initialize(std::vector<Scenario>& sce)
    {
        std::vector<double> sss(in_.GetRequests().size());

        double b1 = (double)tau / (double)p_.GetDemand().size();

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

    void CCGwithPAP::PrintSolution() {
        std::cout << "Vessel Decision vVar : ";
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
    }



}