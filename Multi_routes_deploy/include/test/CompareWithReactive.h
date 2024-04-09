#ifndef COMPAREWITHREACTIVE_H_
#define COMPAREWITHREACTIVE_H_

#include <string>
#include <fstream>
#include <vector>

#include "data_process.h"
#include "InputData.h"
#include "Parameter.h"
#include "ReadData.h"
#include "GenerateParameter.h"
#include "DetermineModel.h"
#include "DetermineModelReactive.h"
#include "BDwithPAP.h"
#include "BDwithPAP_Reactive.h"
#include "CCGwithPAP.h"
#include "CCGwithPAP_Reactive.h"

namespace fleetdeployment {

    class CompareWithReactive : public Setting {
    private:
        std::ofstream fileWriter;
        std::vector<int> timeHorizonSet;
        static constexpr double uncertainDegree = 0.05;

    public:

        CompareWithReactive(int instance, const std::string& type);

        // Assume these methods are implemented elsewhere
        void experiment_test1(const std::string& fileName);
        void experiment_test2(const std::string& fileName);
    };

}
#endif // !COMPAREWITHREACTIVE_H_
