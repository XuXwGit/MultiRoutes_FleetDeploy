#include "CompareWithReactive.h"

namespace fleetdeployment {


    CompareWithReactive::CompareWithReactive(int instance, const std::string& type) {
        std::string fileName;

        std::string filePath = "Performance.txt";
        std::ifstream ifile(filePath);
        if (!ifile) {
            // File does not exist, create a new file
            fileWriter.open(filePath, std::ios::out | std::ios::app);
        }
        else {
            // File exists, append to it
            fileWriter.open(filePath, std::ios::out | std::ios::app);
        }

        // instance:
        // 1-> data1 : small scale with 2 ship routes
        // 2-> data2 : large scale with 8 ship routes
        fileName = "data/";
        if (instance == 1) {
            fileName += "data1";
            timeHorizonSet = { 60, 75, 90, 105, 120 };
        }
        else if (instance == 2) {
            fileName += "data2";
            timeHorizonSet = { 60, 75, 90, 105, 120 };
        }

        // compare the empty container reposition strategy with reactive strategy
        // experiment type
        // true : Use CCG&PAP
        // false: Use BD&PAP
        if (type == "CCG") {
            experiment_test1(fileName);
        }
        else if (type == "BD") {
            experiment_test2(fileName);
        }
    }


    void CompareWithReactive::experiment_test1(const std::string& fileName) {
        std::string filename = fileName + "/";

        for (int T : timeHorizonSet) {
            std::cout << "TimeHorizon: " << T << "\n";
            std::cout << "UncertainDegree: " << uncertainDegree << "\n";

            InputData inputData;
            ReadData read(filename, inputData, T);

            Parameter para;
            GenerateParameter gp(para, inputData, T, uncertainDegree);

            int tau = static_cast<int>(std::sqrt(inputData.GetRequests().size()));

            DetermineModel de(inputData, para);
            DetermineModelReactive der(inputData, para);
            CCGwithPAP cp(inputData, para, tau);
            CCGwithPAP_Reactive cpr(inputData, para, tau);

            std::cout << "=====================================================================\n";
            std::cout << "Algorithm: \tDetermine\tDetermine&Reactive\tCCG&PAP\tCCG&PAP&Reactive\n";
            std::cout << std::fixed << std::setprecision(2);  // Set fixed floating point and 2 decimal places
            std::cout << "SolveTime: \t" << de.GetSolveTime() << "\t" << der.GetSolveTime() << "\t"
                << cp.GetSolveTime() << "\t" << cpr.GetSolveTime() << "\n";
            std::cout << "Objective: \t" << de.GetObjVal() << "\t"
                << der.GetObjVal() << "\t"
                << cp.GetObj() << "\t"
                << cpr.GetObj() << "\n";
            std::cout << "Gap: \t" << de.GetMipGap() << "\t"
                << der.GetMipGap() << "\t"
                << cp.GetGap() << "\t"
                << cpr.GetGap() << "\n";
            std::cout << "=====================================================================\n";

            if (WhetherOutputLog) {
                fileWriter << std::fixed << std::setprecision(2);  // Apply formatting to file output
                fileWriter << "Algorithm: \tDetermine\tDetermine&Reactive\tCCG&PAP\tCCG&PAP&Reactive\n";
                fileWriter << "SolveTime: \t" << de.GetSolveTime() << "\t" << der.GetSolveTime() << "\t"
                    << cp.GetSolveTime() << "\t" << cpr.GetSolveTime() << "\n";
                fileWriter << "Objective: \t" << de.GetObjVal() << "\t"
                    << der.GetObjVal() << "\t"
                    << cp.GetObj() << "\t"
                    << cpr.GetObj() << "\n";
                fileWriter << "Iteration: \t1\t1\t" << cp.GetIter() << "\t"
                    << cpr.GetIter() << "\n";
            }
        }
    }

    void CompareWithReactive::experiment_test2(const std::string& fileName) {

        std::string filename = fileName + "/";

        for (int T : timeHorizonSet) {
            std::cout << "TimeHorizon : " << T << "\n";
            std::cout << "UncertainDegree : " << uncertainDegree << "\n";

            InputData inputData;
            new ReadData(filename, inputData, T);

            Parameter para;
            new GenerateParameter(para, inputData, T, uncertainDegree);

            int tau = (int)sqrt(inputData.GetRequests().size());

            DetermineModel de(inputData, para);
            DetermineModelReactive der(inputData, para);
            BDwithPAP  bp(inputData, para, tau);
            BDwithPAP_Reactive bpr(inputData, para, tau);

            std::cout << "=====================================================================\n";
            std::cout << "Algorithm :"
                << "\tDetermine"
                << "\tDetermine&Reactive"
                << "\tBD&PAP"
                << "\tBD&PAP&Reactive\n";
            std::cout << "SolveTime :"
                << "\t" << de.GetSolveTime()
                << "\t" << der.GetSolveTime()
                << "\t" << bp.GetSolveTime()
                << "\t" << bpr.GetSolveTime() << "\n";
            std::cout << "Objective  :"
                << "\t" << std::fixed << std::setprecision(2) << de.GetObjVal()
                << "\t" << std::fixed << std::setprecision(2) << der.GetObjVal()
                << "\t" << std::fixed << std::setprecision(2) << bp.GetObj()
                << "\t" << std::fixed << std::setprecision(2) << bpr.GetObj() << "\n";
            std::cout << "Gap :"
                << "\t" << std::fixed << std::setprecision(4) << de.GetMipGap()
                << "\t" << std::fixed << std::setprecision(4) << der.GetMipGap()
                << "\t" << std::fixed << std::setprecision(4) << bp.GetGap()
                << "\t" << std::fixed << std::setprecision(4) << bpr.GetGap() << "\n";
            std::cout << "=====================================================================\n";

            if (WhetherOutputLog) {
                fileWriter << "Algorithm :" << "\t"
                    << "Determine" << "\t"
                    << "Determine&Strategy" << "\t"
                    << "BD&PAP" << "\t"
                    << "BD&PAP&Reactive" << "\n";
                fileWriter << "SolveTime :" << "\t"
                    << de.GetSolveTime() << "\t"
                    << der.GetSolveTime() << "\t"
                    << bp.GetSolveTime() << "\t"
                    << bpr.GetSolveTime() << "\n";
                fileWriter << "Objective  :" << "\t"
                    << std::fixed << std::setprecision(2) << de.GetObjVal() << "\t"
                    << std::fixed << std::setprecision(2) << der.GetObjVal() << "\t"
                    << std::fixed << std::setprecision(2) << bp.GetObj() << "\t"
                    << std::fixed << std::setprecision(2) << bpr.GetObj() << "\n";
            }

        }
    }



}