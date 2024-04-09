#include "PerformanceExperiment.h"

namespace fleetdeployment {

    void PerformanceExperiment::experiment_test(const std::string& fileName) {

        std::string filename = fileName + "/";

        for (int T : timeHorizonSet) {
            std::cout << "TimeHorizon : " << T << "\n";
            InputData inputData;
            new ReadData(filename, inputData, T);

            inputData.ShowStatus();

            Parameter para;
            new GenerateParameter(para, inputData, T, uncertainDegree);


            std::cout << "After Cutting Paths : ";
            std::cout << "Requests = " + inputData.GetRequests().size() << "\t"
                << "Paths = " << inputData.GetPathSet().size() << "\t" << "\n";

            std::cout << "UncertainDegree : " << uncertainDegree << "\n";

            int tau = (int)sqrt(inputData.GetRequests().size());

            CCGwithPAP cp(inputData, para, tau);
            CCG ccg(inputData, para, tau);
            BDwithPAP bp(inputData, para, tau);
            BD bd(inputData, para, tau);
            std::cout << "=====================================================================\n";

            std::cout << "Algorithm :" << "\t"
                << "CCG&PAP" << "\t"
                << "CCG" << "\t"
                << "BD&PAP" << "\t"
                << "BD" << "\n";
            std::cout << "SolveTime :" << "\t"
                << cp.GetSolveTime() << "\t"
                << ccg.GetSolveTime() << "\t"
                << bp.GetSolveTime() << "\t"
                << bd.GetSolveTime() << "\n";
            std::cout << "Objective  :" << "\t"
                << std::fixed << std::setprecision(2) << cp.GetObj() << "\t"
                << std::fixed << std::setprecision(2) << ccg.GetObj() << "\t"
                << std::fixed << std::setprecision(2) << bp.GetObj() << "\t"
                << std::fixed << std::setprecision(2) << bd.GetObj() << "\n";
            std::cout << "Iteration    :" << "\t"
                + cp.GetIter() << "\t"
                + ccg.GetIter() << "\t"
                + bp.GetIter() << "\t"
                + bd.GetIter() << "\n";
            std::cout << "=====================================================================\n";
            if (WhetherOutputLog) {
                fileWriter << "Algorithm :" << "\t"
                    << "CCG&PAP" << "\t"
                    << "CCG" << "\t"
                    << "BD&PAP" << "\t"
                    << "BD" << "\n";
                fileWriter << "SolveTime :" << "\t"
                    << cp.GetSolveTime() << "\t"
                    << ccg.GetSolveTime() << "\t"
                    << bp.GetSolveTime() << "\t"
                    << bd.GetSolveTime() << "\n";
                fileWriter << "Objective  :" << "\t"
                    << std::fixed << std::setprecision(2) << cp.GetObj() << "\t"
                    << std::fixed << std::setprecision(2) << ccg.GetObj() << "\t"
                    << std::fixed << std::setprecision(2) << bp.GetObj() << "\t"
                    << std::fixed << std::setprecision(2) << bd.GetObj() << "\n";
                fileWriter << "Iteration    :" << "\t"
                    << cp.GetIter() << "\t"
                    << ccg.GetIter() << "\t"
                    << bp.GetIter() << "\t"
                    << bd.GetIter() << "\n";
            }
        }
    }

    void PerformanceExperiment::experiment_test2(const std::string& fileName) {

        std::string filename = fileName + "/";

        for (int T : timeHorizonSet) {
            std::cout << "TimeHorizon: " << T << "\n";
            std::cout << "UncertainDegree: " << uncertainDegree << "\n";

            InputData inputData;
            new ReadData(filename, inputData, T);

            Parameter para;
            new GenerateParameter(para, inputData, T, uncertainDegree);

            int tau = static_cast<int>(std::sqrt(inputData.GetRequests().size()));

            DetermineModel de(inputData, para);
            DetermineModelReactive der(inputData, para);
            CCGwithPAP cp(inputData, para, tau);
            CCGwithPAP_Reactive cpr(inputData, para, tau);

            std::cout << "=====================================================================\n";
            std::cout << "Algorithm :\t"
                << "Determine\t"
                << "Determine&Reactive\t"
                << "CCG&PAP\t"
                << "CCG&PAP&Reactive\t\n";

            std::cout << "SolveTime :\t"
                << de.GetSolveTime() << "\t"
                << der.GetSolveTime() << "\t"
                << cp.GetSolveTime() << "\t"
                << cpr.GetSolveTime() << "\n";

            std::cout << "Objective  :\t"
                << std::fixed << std::setprecision(2) << de.GetObjVal() << "\t"
                << std::fixed << std::setprecision(2) << der.GetObjVal() << "\t"
                << std::fixed << std::setprecision(2) << cp.GetObj() << "\t"
                << std::fixed << std::setprecision(2) << cpr.GetObj() << "\n";

            std::cout << "Objective  :\t"
                << std::fixed << std::setprecision(2) << de.GetObjVal() << "\t"
                << std::fixed << std::setprecision(2) << der.GetObjVal() << "\t"
                << std::fixed << std::setprecision(2) << cp.GetObj() << "\t"
                << std::fixed << std::setprecision(2) << cpr.GetObj() << "\n";

            std::cout << "=====================================================================\n\n";

            if (WhetherOutputLog) {
                fileWriter << "Algorithm :\t"
                    << "Determine\t"
                    << "Determine&Reactive\t"
                    << "CCG&PAP\t"
                    << "CCG&PAP&Reactive\t\n";

                fileWriter << "SolveTime :\t"
                    << de.GetSolveTime() << "\t"
                    << der.GetSolveTime() << "\t"
                    << cp.GetSolveTime() << "\t"
                    << cpr.GetSolveTime() << "\n";

                fileWriter << "Objective  :\t"
                    << std::fixed << std::setprecision(2) << de.GetObjVal() << "\t"
                    << std::fixed << std::setprecision(2) << der.GetObjVal() << "\t"
                    << std::fixed << std::setprecision(2) << cp.GetObj() << "\t"
                    << std::fixed << std::setprecision(2) << cpr.GetObj() << "\n";

                fileWriter << "Iteration    :" << "\t"
                    << 1 << "\t"
                    << 1 << "\t"
                    << cp.GetIter() << "\t"
                    << cpr.GetIter() << "\n";
            }

        }
    }


}