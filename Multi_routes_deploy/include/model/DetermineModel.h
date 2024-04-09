#ifndef DETERMINE_MODEL_H
#define DETERMINE_MODEL_H

#include <vector>
#include <iostream>
#include <string>

#include <ilcplex/ilocplex.h>

#include "InputData.h"
#include "Parameter.h"
#include "Setting.h"




namespace fleetdeployment
{


	class  DetermineModel : public Setting {
	public:
		DetermineModel(const InputData& in, const Parameter& p);  // Constructor

		void SolveModel();
		void PrintSolution();

	private:
		InputData in_;
		Parameter p_;
		IloEnv env_;
		IloCplex cplex_;
		IloModel model_;

		std::vector<std::vector<IloIntVar>> vVar_;
		std::vector<std::vector<IloNumVar>> xVar_;
		std::vector<std::vector<IloNumVar>> yVar_;
		std::vector<std::vector<IloNumVar>> zVar_;
		std::vector<IloNumVar> gVar_;

		// ... Other member variables ...
		double mipGap_;
		double solveTime_;
		double objVal_;
		//double operationCost_;

		std::vector<std::vector<int>> vVarValue_;

	public:
		inline void SetMipGap(double mipGap) { mipGap_ = mipGap; }
		inline double GetMipGap() const { return mipGap_; }
		inline void SetSolveTime(double solveTime) { solveTime_ = solveTime; }
		inline double GetSolveTime() const { return solveTime_; }
		inline void SetObjVal(double objVal) { objVal_ = objVal; }
		inline double GetObjVal() const { return objVal_; }
		//inline void SetOperationCost(double operationCost) { operationCost_ = operationCost; }
		//inline double GetOperationCost() const { return operationCost_; }
		inline void SetVVarValue(std::vector<std::vector<int>> vVarValue) { vVarValue_ = vVarValue; }
		inline std::vector<std::vector<int>> GetVVarValue() const { return vVarValue_; }

	private:
		void SetDecisionVars();
		void SetObjective();
		void SetConstraints();

		void SetConstraint1();
		void SetConstraint2();
		void SetConstraint3();
		void SetConstraint4();

		// ... Additional helper methods ...
	};

}
#endif //DETERMINE_MODEL_H
