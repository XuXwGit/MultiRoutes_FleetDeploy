#ifndef SENSITIVITYANALYSIS_H_
#define SENSITIVITYANALYSIS_H_

#include <fstream>
#include <vector>
#include <string>
#include <stdexcept>

#include "data_process.h"
#include "Setting.h"
#include "InputData.h"
#include "ReadData.h"
#include "CCGwithPAP.h"

namespace fleetdeployment {

class SensitivityAnalysis : public Setting {
private:
    std::string fileName = "data/data1";
    std::vector<double> uncertainDegreeSet = { 0.005, 0.015, 0.025, 0.035, 0.045, 0.055, 0.065, 0.075, 0.085, 0.095 };
    std::vector<double> containerPathCostSet = { 0.80, 0.825, 0.85, 0.875, 0.90, 0.925, 0.95, 0.975, 1.025, 1.05, 1.0725, 1.10, 1.125, 1.15, 1.175, 1.20 };
    std::vector<double> rentalContainerCostSet = { 0.50, 0.55, 0.60, 0.65, 0.70, 0.75, 0.80, 0.85, 0.90, 0.95, 1.05, 1.10, 1.15, 1.20, 1.25, 1.30, 1.35, 1.40, 1.45, 1.50 };
    std::vector<double> penaltyCostSet = { 0.50, 0.55, 0.60, 0.65, 0.70, 0.75, 0.80, 0.85, 0.90, 0.95, 1.05, 1.10, 1.15, 1.20, 1.25, 1.30, 1.35, 1.40, 1.45, 1.50 };
    std::vector<int> turnOverTimeSet = { 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20, 21, 22, 23, 24, 25, 26, 27, 28 };
    std::vector<int> initialContainerSet = { 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20, 21, 22, 23, 24, 25, 26, 27, 28, 29, 30, 31, 32, 33, 34, 35, 36, 37, 38, 39, 40, 41, 42 };
    std::vector<int> timeHorizonSet = { 60, 75, 90, 105, 120, 135, 150, 165, 180 };

    int T = 90;
    double uncertainDegree = 0.05;
    std::ofstream fileWriter;
    int Algo;

public:
    SensitivityAnalysis(const int algo) : Algo(algo) 
    {

        if (WhetherOutputLog) {
            fileWriter.open("Sensitivity Analysis.txt", std::ios::app);
            if (!fileWriter.is_open()) {
                throw std::runtime_error("Failed to open file 'Sensitivity Analysis.txt");
            }
        }

        std::string filename = fileName + "/";

        try {
            InputData in;
            // Assuming ReadData is a function or a class that needs to be implemented
            ReadData read(filename, in, T);

            // Assuming these methods are implemented elsewhere
            // VaryInitialContainers(in);
            VaryTurnOverTime(in);
            // VaryUncertainDegree(in);
            // VaryLoadAndDischargeCost(in);
            VaryRentalCost(in);
            // VaryPenaltyCost(in);
        }
        catch (const std::exception& e) {
            std::cerr << "Error: " << e.what() << std::endl;
        }
    }

private:

    void VaryUncertainDegree(InputData in);
    void VaryLoadAndDischargeCost(InputData in);
    void VaryRentalCost(InputData in);
    void VaryPenaltyCost(InputData in);
    void VaryTurnOverTime(InputData in);
    void VaryInitialContainers(InputData& in);
};

}
#endif // !SENSITIVITYANALYSIS_H_