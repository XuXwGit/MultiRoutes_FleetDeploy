#ifndef BD_H_
#define BD_H_

#include <vector>
#include <string>
#include <fstream>
#include <iostream>

#include "Setting.h"
#include "InputData.h"
#include "Parameter.h"
#include "Scenario.h"


namespace fleetdeployment
{
    class BD;
}

class fleetdeployment::BD : public Setting {
private:
    InputData in_;
    Parameter p_;
    int tau;

    double upperBound;
    double lowerBound;
    double solveTime;
    double obj;
    int iter;
    double Gap;
    std::vector<std::vector<int>> vValue;

    // Function prototypes for any additional private methods
    void frame();

    void Initialize(std::vector<Scenario>& sce);

    void PrintSolution();

public:
    BD(const InputData& in, const Parameter& p, int tau) : in_(in), p_(p), tau(tau) {
        frame();
    }

public:
    // Inline methods
    inline void SetUpperBound(double upperBound) { this->upperBound = upperBound; }
    inline void SetLowerBound(double lowerBound) { this->lowerBound = lowerBound; }
    inline double GetUpperBound() const { return upperBound; }
    inline double GetLowerBound() const { return lowerBound; }
    inline double GetSolveTime() const { return solveTime; }
    inline void SetSolveTime(double solveTime) { this->solveTime = solveTime; }
    inline double GetObj() const { return obj; }
    inline void SetObj(double obj) { this->obj = obj; }
    inline int GetIter() const { return iter; }
    inline void SetIter(int iter) { this->iter = iter; }
    inline void SetVValue(const std::vector<std::vector<int>>& vValue) { this->vValue = vValue; }
    inline double GetGap() const { return Gap; }
    inline void SetGap(double Gap) { this->Gap = Gap; }
};

#endif // !BD_H_
