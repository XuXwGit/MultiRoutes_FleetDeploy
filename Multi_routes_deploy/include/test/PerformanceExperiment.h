#ifndef PERFORMANCEEXPERMIENT_H_
#define PERFORMANCEEXPERMIENT_H_

#include <fstream>
#include <string>
#include <vector>

#include "data_process.h"
#include "Setting.h"
#include "GenerateParameter.h"
#include "InputData.h"
#include "Parameter.h"
#include "ReadData.h"
#include "DetermineModel.h"
#include "DetermineModelReactive.h"
#include "BD.h"
#include "BDwithPAP.h"
#include "BDwithPAP_Reactive.h"
#include "CCG.h"
#include "CCGwithPAP.h"
#include "CCGwithPAP_Reactive.h"

namespace fleetdeployment {


    class PerformanceExperiment : public Setting {
    private:
        std::string path = "Data/";
        std::ofstream fileWriter;
        std::vector<int> timeHorizonSet;
        double uncertainDegree = 0.05;

    public:
        PerformanceExperiment(int instance, bool type) {
            std::string fileName;

            // Open the file for writing. If it does not exist, it's created.
            fileWriter.open(path + "log/" + "Performance.txt", std::ios::app);
            if (!fileWriter.is_open()) {
                throw std::runtime_error("Failed to open file 'Performance.txt'");
            }

            // Setup based on instance type
            fileName = path + "data/";
            if (instance == 1) {
                fileName += "data1";
                timeHorizonSet = { 56, 63, 70, 77, 84, 91 };
            }
            else if (instance == 2) {
                fileName += "data2";
                timeHorizonSet = { 60, 75, 90, 105, 120 };
            }

            // Decide experiment type
            if (type) {
                experiment_test(fileName);
            }
            else {
                experiment_test2(fileName);
            }
        }

    private:
        void experiment_test(const std::string& fileName);

        void experiment_test2(const std::string& fileName);
    };

}


#endif //! PERFORMANCEEXPERMIENT_H_
