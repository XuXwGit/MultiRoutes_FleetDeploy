#ifndef SUBDERMODEL_H_
# define SUBDERMODEL_H_

#include <vector>

#include <ilcplex/ilocplex.h>

#include "Setting.h"
#include "InputData.h"
#include "Parameter.h"
#include "Request.h"


namespace fleetdeployment
{

	class SubDerModel : public Setting {
	private:
		InputData in_;
		Parameter p_;
		std::vector<double> uValue_;
		IloEnv env_;
		IloCplex cplex_;
		IloModel model_;

		std::vector<std::vector<IloIntVar>> vVar_;
		std::vector<std::vector<IloNumVar>> xVar_;
		std::vector<std::vector<IloNumVar>> yVar_;
		std::vector<std::vector<IloNumVar>> zVar_;
		std::vector<IloNumVar>  gVar_;

		std::vector<IloRange> C1_;

		double objVal_;
		double operationCost;
		std::vector<std::vector<int>> vVarValue;

		inline void SetConstraints() {
			SetConstraint0();
			SetConstraint1();
			SetConstraint2();
			SetConstraint3();
		}


	public:
		void end()
		{
			model_.end();
			cplex_.end();
			env_.end();
		}

		double GetObjVal() {
			return objVal_;
		}

		double GetOperationCost() {
			return operationCost;
		}

		std::vector<std::vector<int>> GetVVarValue() {
			return vVarValue;
		}

		SubDerModel(const InputData& in, const Parameter& p);

	private:
		void SetDecisionVars();
		void SetObjective();
		void SetConstraint0();
		void SetConstraint1();
		void SetConstraint2();
		void SetConstraint3();
		void SolveModel();
		void changeConstraints(const std::vector<double>& uValue);

		inline void SetObjVal(double obj) {
			objVal_ = obj;
		}
		void SetVVarValue();
		void SetOperationCost();
	};



}
#endif // !SUBDERMODEL_H_
