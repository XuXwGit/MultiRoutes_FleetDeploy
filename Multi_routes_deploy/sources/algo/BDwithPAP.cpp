#include "BDwithPAP.h"

namespace fleetdeployment
{

    void BDwithPAP::frame() {

        std::string filename = "BD&PAP" + std::to_string(p_.GetTimePointSet().size()) + ".txt";

        // 使用 std::ofstream 打开文件，ios::app 模式用于追加写入
        std::ofstream fileWriter(filename, std::ios::app);

        // 检查文件是否成功打开
        if (!fileWriter.is_open()) {
            std::cerr << "Error: Unable to open file " << filename << std::endl;
        }

        time_t start = time(0);

        // change  MaxVarDemand
        // beta = min(k, m/k)=tau

        SetUpperBound(1e12);
        SetLowerBound(-1e12);

        std::vector<Scenario> sce;

        std::vector<double> maxDemandVar = p_.GetMaximumDemandVariation();
        p_.changeMaximunDemandVariation(tau);

        int max_iteration = 100;
        std::vector<std::vector<int>> vvv(p_.GetVesselSet().size(), std::vector<int>(p_.GetVesselRouteSet().size()));
        std::vector<double> upper(max_iteration + 1);
        std::vector<double> lower(max_iteration + 1);
        upper[0] = upperBound;
        lower[0] = lowerBound;
        std::vector<int> worseCaseSet(p_.GetDemand().size());
        double gap = std::numeric_limits<double>::max();

        time_t time0 = time(0);
        MasterProblem mp(in_, p_);
        DualSubProblem dsp(in_, p_, 1);

        //        SubDerModel sdp = new SubDerModel(in, p);
        //        SubProblem sp = new SubProblem(in, p);

                // add the initial scene to make the MP feasible
        Initialize(sce);
        int iteration = 0;
        int flag = 0;

        if (WhetherPrintProcess) {
            std::cout << "==========BD with PAP : ===========" << std::endl;
            std::cout << "BuildModelTime = "
                << std::fixed << std::setprecision(2) << time(0) - time0 << std::endl;
            std::cout << "k" << "\t\t"
                << "UB" << "\t\t"
                << "LB" << "\t\t"
                << "DSP-SolveTime" << "\t\t"
                << "MP-SolveTime" << "\t\t"
                << "Total Time" << "\t\t"
                << "DSP-Status" << "\t\t"
                << "MP-Status";
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


        mp.addScene(sce[iteration]);
        time_t start0 = time(0);
        while (upperBound - lowerBound > MIPGapLimit
            && gap > MIPGapLimit
            && flag == 0
            && iteration < max_iteration)
        {
            time_t start1 = time(0);
            mp.SolveModel();
            time_t end1 = time(0);

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

            // LB = max{LB , MP.Objective}
            // LB = MP.Objective = MP.OperationCost + Eta
            if (mp.GetObjVal() > lowerBound) {
                SetLowerBound(mp.GetObjVal());
            }
            //            printSolution(mp.GetVVarValue());

            dsp.changeVVarCoefficients(mp.GetVVarValue());
            time_t start2 = time(0);
            dsp.SolveModel();
            time_t end2 = time(0);

            //  the SP is optimal :  add optimality cut
            if (dsp.GetSolveStatus() == IloCplex::Status::Optimal)
            {
                //  update UB : UB = min{UB, MP.OperationCost + SP.Objective}
                if (dsp.GetObjVal() + mp.GetOperationCost() < upperBound)
                {
                    SetUpperBound(dsp.GetObjVal() + mp.GetOperationCost());
                }

                // add optimality cut
                mp.addOptimalityCut(dsp.GetConstantItem(), dsp.GetBetaValue());

                // add the worst scene (extreme point) to scene Set
    //                Scenario scene =  dsp.GetScene();
    //                sce.add(scene);
    //                std::vector<double> request = scene.GetRequest();
    //                for(int i = 0; i < p_.GetDemand().size(); i++) {
    //                    request[i] = dsp.GetUValue()[i];
    //                    if (request[i] != 0)
    //                        worseCaseSet[iteration+1] = i;
    //                }
            }
            // the SP is unbounded : add feasibility cut
            else if (dsp.GetSolveStatus() == IloCplex::Status::Unbounded)
            {
                // ! here beta is extreme ray !
                mp.addFeasibilityCut(dsp.GetConstantItem(), dsp.GetBetaValue());
            }

            iteration = iteration + 1;
            upper[iteration] = upperBound;
            lower[iteration] = lowerBound;
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

        mp.end();
        dsp.end();

        SetVValue(mp.GetVVarValue());
        SetIter(iteration);
        SetObj(upperBound);

        Gap = (upperBound - lowerBound) / lowerBound;

        if (WhetherPrintProcess) {
            PrintSolution();
        }

        time_t end = time(0);
        SetSolveTime(difftime(end, start));

        if (WhetherPrintProcess || WhetherPrintSolveTime) {
            std::cout << "BD SolveTime = " << (difftime(end, start)) << std::endl;
            std::cout << "=================================" << std::endl;
        }
        if (WhetherOutputLog) {
            fileWriter << "BD SolveTime = " << difftime(end, start) << std::endl;
        }
        fileWriter.close();

        p_.SetMaximumDemandVariation(maxDemandVar);
    }

    void BDwithPAP::Initialize(std::vector<Scenario>& sce)
    {
        std::vector<double> sss(in_.GetRequests().size());

        // beta = min{k , I/k}
        double beta = (double)tau > p_.GetDemand().size() / (double)tau ?
            p_.GetDemand().size() / (double)tau : (double)tau;

        double v = (double)1 / (double)p_.GetDemand().size();
        for (int i = 0; i < p_.GetDemand().size(); i++)
        {
            sss[i] = beta * v * 1 / sqrt(p_.GetDemand().size());
        }

        Scenario sp;
        sp.SetRequest(sss);
        sce.push_back(sp);
    }


    void BDwithPAP::PrintSolution() {
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