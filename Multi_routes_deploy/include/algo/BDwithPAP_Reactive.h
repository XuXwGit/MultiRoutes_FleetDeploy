#ifndef BDWITHPAP_REACTIVE_H
#define BDWITHPAP_REACTIVE_H

#include <vector>
#include <string>
#include <fstream>
#include <iostream>

#include "Setting.h"
#include "InputData.h"
#include "Parameter.h"
#include "Scenario.h"
#include "BDwithPAP.h"
#include "DualSubProblemReactive.h"


namespace fleetdeployment
{
    class BDwithPAP_Reactive;
}

class fleetdeployment::BDwithPAP_Reactive : public Setting {
private:
    InputData in_;
    Parameter p_;

    int tau;
    double mipGap;
    double upperBound;
    double lowerBound;
    double obj;
    double solveTime;
    int iter;
    double Gap;

    std::vector<std::vector<int>> vValue;

    // Additional methods declaration

public:
    BDwithPAP_Reactive(const InputData& in, const Parameter& p, int tau)
        : in_(in), p_(p), tau(tau)
    {
        frame();
    }

    // Inline getters and setters
    inline double GetUpperBound() const { return upperBound; }
    inline void SetUpperBound(double upperBound) { this->upperBound = upperBound; }

    inline double GetLowerBound() const { return lowerBound; }
    inline void SetLowerBound(double lowerBound) { this->lowerBound = lowerBound; }

    inline double GetObj() const { return obj; }
    inline void SetObj(double obj) { this->obj = obj; }

    inline double GetSolveTime() const { return solveTime; }
    inline void SetSolveTime(double solveTime) { this->solveTime = solveTime; }

    inline int GetIter() const { return iter; }
    inline void SetIter(int iter) { this->iter = iter; }

    inline const std::vector<std::vector<int>>& GetVValue() const { return vValue; }
    inline void SetVValue(const std::vector<std::vector<int>>& vValue) { this->vValue = vValue; }

    inline double GetGap() const { return Gap; }
    inline void SetGap(double gap) { Gap = gap; }

    void frame();

    void Initialize(std::vector<Scenario>& sce);

    void PrintSolution();

    // Additional methods definition
};

#endif // BDWITHPAP_REACTIVE_H