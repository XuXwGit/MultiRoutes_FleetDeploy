#ifndef CCGWITHPAP_H
#define CCGWITHPAP_H

#include <vector>
#include <string>
#include <fstream>
#include <iostream>

#include "Setting.h"
#include "InputData.h"
#include "Parameter.h"
#include "Scenario.h"
#include "MasterProblem.h"
#include "DualSubProblem.h"
#include "SubProblem.h"

namespace fleetdeployment
{
    class CCGwithPAP;

}

class fleetdeployment::CCGwithPAP : public Setting {
private:
    InputData in_;
    Parameter p_;

    const int tau;

    double upperBound;
    double lowerBound;
    double obj;
    double solveTime;
    int iter;
    double Gap;
    double operationCost;
    double rentalCost;
    double ladenCost;
    double emptyCost;
    double penaltyCost;
    double totalCost;

    std::vector<std::vector<int>> vValue;

public:
    CCGwithPAP(const InputData& in, const Parameter& p, int tau)
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

    inline double GetGap() const { return Gap; }
    inline void SetGap(double gap) { Gap = gap; }

    // More setters and getters for costs
    inline void SetOperationCost(double operationCost) { this->operationCost = operationCost; }
    inline double GetOperationCost() const { return operationCost; }

    inline void SetRentalCost(double rentalCost) { this->rentalCost = rentalCost; }
    inline double GetRentalCost() const { return rentalCost; }

    inline void SetLadenCost(double ladenCost) { this->ladenCost = ladenCost; }
    inline double GetLadenCost() const { return ladenCost; }

    inline void SetEmptyCost(double emptyCost) { this->emptyCost = emptyCost; }
    inline double GetEmptyCost() const { return emptyCost; }

    inline void SetPenaltyCost(double penaltyCost) { this->penaltyCost = penaltyCost; }
    inline double GetPenaltyCost() const { return penaltyCost; }

    inline double GetTotalCost() const { return obj; }
    inline void SetTotalCost(double obj) { this->obj = obj; }

    inline void SetVValue(std::vector<std::vector<int>> vValue) { this->vValue = vValue; }
    inline std::vector<std::vector<int>> GetVValue() const { return vValue; }


    void frame();

    void Initialize(std::vector<Scenario>& sce);

    void PrintSolution();

    // Implementation of frame() and other methods
};

#endif // CCGWITHPAP_H
