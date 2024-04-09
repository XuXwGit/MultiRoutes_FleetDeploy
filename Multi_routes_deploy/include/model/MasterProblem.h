#ifndef MASTER_PROBLEM_H_
#define MASTER_PROBLEM_H_

#include <vector>
#include <string>
#include <iostream>
#include <memory>

#include <ilcplex/ilocplex.h>

#include "InputData.h" 
#include "Parameter.h" 
#include "Setting.h"
#include "Scenario.h"


namespace fleetdeployment
{


	class MasterProblem : public Setting
	{
	public:
		MasterProblem(const InputData& in, const Parameter& p);
		MasterProblem(const InputData& in, const Parameter& p, bool Reactive);


	private:
		void SetDecisionVars();

		void SetObjective();

		void SetReactiveDecisionVars();

		void SetReactiveObjective();

		void SetConstraint1();

		void SetConstraint0(const std::vector<std::vector<IloNumVar>>& xVar, const std::vector<std::vector<IloNumVar>>& yVar, const std::vector<std::vector<IloNumVar>>& zVar, const std::vector<IloNumVar>& gVar);

		void SetConstraint4(const std::vector<std::vector<IloNumVar>>& xVar, const std::vector<std::vector<IloNumVar>>& yVar, const std::vector<IloNumVar>& gVar, const std::vector<double>& uValue);

		void SetConstraint5(const std::vector<std::vector<IloNumVar>>& xVar, const std::vector<std::vector<IloNumVar>>& yVar, const std::vector<std::vector<IloNumVar>>& zVar);

		void SetConstraint5_Reactive1(const std::vector<std::vector<IloNumVar>>& xVar, const std::vector<std::vector<IloNumVar>>& yVar);

		void SetConstraint5_Reactive2(const std::vector<std::vector<IloNumVar>>& zVar);

		void SetConstraint6(const std::vector<std::vector<IloNumVar>>& xVar, const std::vector<std::vector<IloNumVar>>& zVar);

	public:

		void addScene(const Scenario& scene_k);

		void addReactiveScene(Scenario scene_k);

		void addOptimalityCut(double constantItem, const std::vector<double>& beta_value);

		void addFeasibilityCut(double constantItem, const std::vector<double>& beta_value);

		void addReactiveOptimalityCut(double constantItem, const std::vector<double>& beta1_value, const std::vector<double>& beta2_value);

		void addReactiveFeasibilityCut(double constantItem, const std::vector<double>& beta1_value, const std::vector<double>& beta2_value);

		void SolveModel();

		void SolveReactiveModel();

		void PrintSolution();

		void PrintReactiveSolution();

	private:
		const InputData& in_;
		const Parameter& p_;

		IloModel model_;
		IloCplex cplex_;
		IloEnv env_;

		std::vector<std::vector<IloIntVar>> vVar_;
		std::vector<std::vector<IloIntVar>> vVar2_;
		IloNumVar etaVar_;

		std::vector<std::vector<int>> vVarValue_;
		std::vector<std::vector<int>> vVarValue2_;

		double etaValue;
		double objective;
		double operationCost;
		double mipGap;


	private:
		inline void SetConstraints() {
			// each vessel route assigns one type of vessel
			SetConstraint1();
		}
		inline void SetEtaValue(double etaValue) {
			this->etaValue = etaValue;
		}
		inline void SetOperationCost(double operationCost) {
			this->operationCost = operationCost;
		}
		inline void SetObjVal(double objective) {
			this->objective = objective;
		}
		inline void SetVVarValue(std::vector<std::vector<int>> vVarValue) {
			this->vVarValue_ = vVarValue;
		}
		inline void SetVVarValue2(std::vector<std::vector<int>> vVarValue2) {
			this->vVarValue2_ = vVarValue2;
		}


	public:
		//		"set basic Constraint for MasterProblem start!"

		inline double GetOperationCost() {
			return operationCost;
		}

		inline double GetObjVal() {
			return objective;
		}

		inline const std::vector<std::vector<int>> GetVVarValue() const {
			return vVarValue_;
		}
		inline std::vector<std::vector<int>> GetVVarValue2() {
			return vVarValue2_;
		}

		inline IloCplex::IloAlgorithm::Status GetSolveStatus() {
			return cplex_.getStatus();
		}
		inline std::string GetSolveStatusString() {
			if (cplex_.getStatus() == IloCplex::Status::Optimal)
				return "Optimal";
			else if (cplex_.getStatus() == IloCplex::Status::Feasible) {
				return "Feasible";
			}
			else if (cplex_.getStatus() == IloCplex::Status::Infeasible) {
				return "Infeasible";
			}
			else if (cplex_.getStatus() == IloCplex::Status::Unbounded) {
				return "Bounded";
			}
			return "Others";
		}

		inline double GetEtaValue() {
			return etaValue;
		}

		inline void end()
		{
			model_.end();
			cplex_.end();
			env_.end();
		}

		inline double GetMipGap() {
			return mipGap;
		}

	};



}
#endif // !MASTER_PROBLEM_H_