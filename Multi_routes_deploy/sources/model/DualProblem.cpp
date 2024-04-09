# include "DualProblem.h"

namespace fleetdeployment {


	DualProblem::DualProblem(const InputData& input, const Parameter& params)
		:in_(input), p_(params)
	{
		vVarValue_.resize(p_.GetVesselSet().size());
		for (size_t h = 0; h < p_.GetVesselSet().size(); ++h) {
			vVarValue_[h].resize(p_.GetVesselRouteSet().size());
		}
		uVarValue_.resize(p_.GetDemand().size());
		try {

			env_ = IloEnv();
			model_ = IloModel(env_);
			cplex_ = IloCplex(model_);


			cplex_.setOut(env_.getNullStream());
			cplex_.setParam(IloCplex::Param::WorkMem, MaxWorkMem);
			cplex_.setParam(IloCplex::Param::TimeLimit, MIPTimeLimit);
			cplex_.setParam(IloCplex::Param::MIP::Tolerances::MIPGap, MIPGapLimit);
			cplex_.setParam(IloCplex::Param::Threads, MaxThreads);

			SetDecisionVars();
			SetConstraints();
			SetObjective();

		}
		catch (IloException e) {
			std::cerr << "Error: " << e.getMessage() << std::endl;
		}
	}


	void DualProblem::SetDecisionVars() {
		alphaVar_.resize(p_.GetDemand().size());
		betaVar_.resize(p_.GetTravelArcsSet().size());

		for (int i = 0; i < p_.GetDemand().size(); ++i) {
			std::string varName = "alpha(" + std::to_string(i + 1) + ")";
			alphaVar_[i] = IloNumVar(env_, IloNum(-std::numeric_limits<int>::max()), IloNum(std::numeric_limits<int>::max()), varName.c_str());
		}

		for (int i = 0; i < p_.GetTravelArcsSet().size(); ++i) {
			std::string varName = "beta(" + std::to_string(i + 1) + ")";
			betaVar_[i] = IloNumVar(env_, IloNum(-std::numeric_limits<int>::max()), IloNum(0), varName.c_str());
		}

		gammaVar_.resize(p_.GetPortSet().size());

		for (int i = 0; i < p_.GetPortSet().size(); ++i) {
			gammaVar_[i].resize(p_.GetTimePointSet().size());
			for (int t = 0; t < p_.GetTimePointSet().size(); ++t) {
				std::string varName = "gamma(" + std::to_string(i + 1) + ")(" + std::to_string(t) + ")";
				gammaVar_[i][t] = IloNumVar(env_, IloNum(0), IloNum(std::numeric_limits<int>::max()), varName.c_str());
			}
		}
	}


	void DualProblem::SetObjective() {
		IloEnv env = cplex_.getEnv();
		IloExpr Obj(env_);

		// I. Part one: sum(normal_demand * alpha + max_var_demand * u * alpha)
		for (int i = 0; i < p_.GetDemand().size(); ++i) {
			Obj += (p_.GetDemand()[i] + p_.GetMaximumDemandVariation()[i] * uVarValue_[i]) * alphaVar_[i];
		}

		// II. Sum (vessel capacity * V[h][r] * beta[arc])
		for (int n = 0; n < p_.GetTravelArcsSet().size(); ++n) {
			double capacity = 0;
			for (int r = 0; r < p_.GetVesselRouteSet().size(); ++r) {
				for (int w = 0; w < p_.GetVesselPathSet().size(); ++w) {
					for (int h = 0; h < p_.GetVesselSet().size(); ++h) {
						capacity += p_.GetArcAndVesselPath()[n][w]
							* p_.GetShipRouteAndVesselPath()[r][w]
							* p_.GetVesselTypeAndShipRoute()[h][r]
							* p_.GetVesselCapacity()[h]
							* vVarValue_[h][r];
					}
				}
			}
			Obj += capacity * betaVar_[n];
		}

		// III. Part three: -p_.GetInitialEmptyContainer()[pp] * gammaVar_[pp][t]
		for (int pp = 0; pp < p_.GetPortSet().size(); ++pp) {
			for (int t = 0; t < p_.GetTimePointSet().size(); ++t) {
				Obj += -p_.GetInitialEmptyContainer()[pp] * gammaVar_[pp][t];
			}
		}

		objective_ = IloMaximize(env_, Obj);

		model_.add(objective_);
	}


	void DualProblem::changeObjectiveCoefficients(const std::vector<std::vector<int>>& vValue,
		const std::vector<int>& uValue)
	{
		this->vVarValue_ = vValue;
		this->uVarValue_ = uValue;

		// I.part one : sum(normal_demand * alpha + max_var_demand*u*alpha) = sum(normal_demand * alpha + max_var_demand * lambda)
		// i ∈I
		for (int i = 0; i < p_.GetDemand().size(); i++)
		{
			objective_.setLinearCoef(alphaVar_[i], p_.GetDemand()[i] + p_.GetMaximumDemandVariation()[i] * uVarValue_[i]);
		}

		// II. sum (vessel capacity * V[h][r] * beta[arc])
		// V[h][r] : the solution come from the master problem (the only changeable input param in dual sub model)
		// <n,n'> ∈ A'
		for (int n = 0; n < p_.GetTravelArcsSet().size(); n++)
		{
			double capacity = 0;
			// r ∈R
			for (int r = 0; r < p_.GetVesselRouteSet().size(); r++)
			{
				// w∈Ω
				for (int w = 0; w < p_.GetVesselPathSet().size(); w++)
				{
					// r(w) = r
					for (int h = 0; h < p_.GetVesselSet().size(); h++)
					{
						capacity += p_.GetArcAndVesselPath()[n][w]
							* p_.GetShipRouteAndVesselPath()[r][w]
							* p_.GetVesselTypeAndShipRoute()[h][r]
							* p_.GetVesselCapacity()[h]
							* vVarValue_[h][r];
					}
				}
			}
			if (Setting::DebugEnable && Setting::DualEnable)
			{
				std::cout << "Dual" << n << ": " << capacity << std::endl;
			}
			// vValue[v][r] : come from solution of master problem
			objective_.setLinearCoef(betaVar_[n], capacity);
		}
	}

	void DualProblem::SetConstraints()
	{
		// dual constraints
		SetConstraint1();
		SetConstraint2();
		SetConstraint3();
		SetConstraint4();
	}

	// C1------X
	void DualProblem::SetConstraint1() {
		IloEnv env = cplex_.getEnv();

		// ∀i∈I (Iterate over all demands)
		for (int i = 0; i < p_.GetDemand().size(); ++i) {
			Request OD = in_.GetRequests()[i];

			// ∀φ∈Φi (Iterate over laden paths)
			for (int k1 = 0; k1 < OD.GetNumberOfLadenPath(); ++k1) {
				int j = OD.GetLadenPathIndexes()[k1];
				IloExpr left(env_);

				// First item
				left += alphaVar_[i];

				// Second item
				// <n,n'> ∈A' (Iterate over all travel arcs)
				for (int n = 0; n < p_.GetTravelArcsSet().size(); ++n) {
					left += p_.GetArcAndPath()[n][j] * betaVar_[n];
				}

				// Third item
				// t∈T (Iterate over time points)
				for (int t = 1; t < p_.GetTimePointSet().size(); ++t) {
					// p ∈P (Iterate over ports)
					for (int m = 0; m < p_.GetPortSet().size(); ++m) {
						// Item 3 and Item 1 logic
						// ... [Add terms to left expression based on your logic] ...
					}
				}

				// Add constraint to the model
				std::string constr_name = "C1_" + std::to_string(j + 1);
				IloRange c1(env_, left, p_.GetLadenPathCost()[j]);
				c1.setName(constr_name.c_str());
				model_.add(c1);
			}
		}
	}

	// C2------Y
	void DualProblem::SetConstraint2()
	{
		// ∀i∈I
		for (int i = 0; i < p_.GetDemand().size(); i++)
		{
			// ∀φ∈Φi
			for (int k1 = 0; k1 < in_.GetRequests()[i].GetNumberOfLadenPath(); k1++)
			{
				int j = in_.GetRequests()[i].GetLadenPathIndexes()[k1];

				IloExpr left(env_);

				// item1:
				left += (1, alphaVar_[i]);

				// <n,n'>∈A'
				for (int n = 0; n < p_.GetTravelArcsSet().size(); n++)
				{
					// item2:
					left += (p_.GetArcAndPath()[n][j] * betaVar_[n]);
				}

				// left <= c3 * g(φ) + c4φ
				std::string constr_name = "C2_" + std::to_string(i + 1) + "_" + std::to_string(j + 1);
				IloRange c2(env_, left, p_.GetRentalCost() * p_.GetTravelTimeOnPath()[j] + p_.GetLadenPathCost()[j]);
				c2.setName(constr_name.c_str());
				model_.add(c2);
			}
		}
	}


	void DualProblem::SetConstraint3() {
		// i∈I
		for (int i = 0; i < p_.GetDemand().size(); i++)
		{
			//  θ∈Θi
			for (int k1 = 0; k1 < in_.GetRequests()[i].GetNumberOfEmptyPath(); k1++)
			{
				int j = in_.GetRequests()[i].GetEmptyPathIndexes()[k1];

				IloExpr left(env_);

				// <n,n'>∈A'
				for (int n = 0; n < p_.GetTravelArcsSet().size(); n++)
				{
					// add item1:
					left += (p_.GetArcAndPath()[n][j] * betaVar_[n]);

					// t∈T
					for (size_t t = 0; t < p_.GetTimePointSet().size(); t++)
					{
						// p∈P
						for (size_t pp = 0; pp < p_.GetPortSet().size(); pp++)
						{
							// p == o(i)
							if (p_.GetPortSet()[pp] == p_.GetOriginOfDemand()[i]) {
								// add item2:
								// item4
								//p(n') == p
								// 1<=t(n')<= t
								if (in_.GetTravelArcs()[n].GetDestinationPort() == (p_.GetPortSet()[pp])
									&& in_.GetTravelArcs()[n].GetDestinationTime() <= t
									&& in_.GetTravelArcs()[n].GetDestinationTime() >= 1) {
									left += (p_.GetArcAndPath()[n][j] * gammaVar_[pp][t]);
								}
							}
							else {
								// p
								// add item3:
								// item2
								// p(n) == p
								// 1<= t(n)<=t
								if (in_.GetTravelArcs()[n].GetOriginPort() == (p_.GetPortSet()[pp])
									&& in_.GetTravelArcs()[n].GetOriginTime() <= t
									&& in_.GetTravelArcs()[n].GetOriginTime() >= 1) {
									left += (-p_.GetArcAndPath()[n][j] * gammaVar_[pp][t]);
								}
							}
						}
					}

					// left <= c5θ
					std::string constr_name = "C3_" + std::to_string(j + 1);
					IloRange c3(env_, left, p_.GetEmptyPathCost()[j]);
					c3.setName(constr_name.c_str());
					model_.add(c3);
				}
			}
		}
	}

	// C4------G
	void DualProblem::SetConstraint4() {
		// Iterate over all demands
		for (int i = 0; i < p_.GetDemand().size(); ++i) {
			// Create and add constraint: alphaVar_[i] <= p_.GetPenaltyCostForDemand()[i]
			IloRange constraint(env_, 0, alphaVar_[i], p_.GetPenaltyCostForDemand()[i]);
			model_.add(constraint);
		}
	}



	void DualProblem::SolveModel() {
		try {
			if (cplex_.solve()) {
				// If the model is solved successfully, retrieve the objective value
				objVal_ = cplex_.getObjValue();

				// Optionally, you can access and print variable values
				// For example: double alphaValue = cplex_.getValue(alphaVar_[0]);

				if (Setting::WhetherPrintProcess) {
					// Print the solution details here if debugging is enabled
					std::cout << "Objective Value: " << objVal_ << std::endl;
					// Additional printing can be done here
				}
			}
			else {
				// If the model is not solved, print an appropriate message
				std::cout << "No solution" << std::endl;
			}
		}
		catch (const IloException& ex) {
			// Catch and display exceptions from CPLEX
			std::cerr << "Concert Error: " << ex.getMessage() << std::endl;
		}
	}


	// here constant item is (sub objective - second item)
	// the second item contains the first stage decision V[h][r]
	double DualProblem::GetConstantItem() {
		double constantItem = getObjVal();
		for (int n = 0; n < p_.GetTravelArcsSet().size(); n++)
		{
			// r ∈R
			for (int r = 0; r < p_.GetVesselRouteSet().size(); r++)
			{
				for (int w = 0; w < p_.GetVesselPathSet().size(); w++)
				{
					// r(w) = r
					for (int h = 0; h < p_.GetVesselSet().size(); h++)
					{
						// vValue[v][r] : come from solution of master problem
						constantItem -= p_.GetArcAndVesselPath()[n][w]
							* p_.GetShipRouteAndVesselPath()[r][w]
							* p_.GetVesselTypeAndShipRoute()[h][r]
							* p_.GetVesselCapacity()[h] * vVarValue_[h][r]
							* cplex_.getValue(betaVar_[n]);
					}
				}
			}
		}

		return constantItem;
	}


	// here beta is used for cutting
	// second item (which contains the first stage decision ) in the cut = sum{λ * q * β * V}
	std::vector<double> DualProblem::GetBetaValue() {
		std::vector<double> beta_value = std::vector<double>(p_.GetTravelArcsSet().size());
		if (cplex_.getStatus() == IloCplex::Status::Optimal)
		{
			for (int i = 0; i < p_.GetTravelArcsSet().size(); i++) {
				beta_value[i] = cplex_.getValue(betaVar_[i]);
			}
		}
		return beta_value;
	}

	void DualProblem::WriteSolution() {
		std::ofstream fileWriter("DP.txt");

		// Check if the file is opened successfully
		if (!fileWriter.is_open()) {
			std::cerr << "Error: Unable to open file DP.txt" << std::endl;
			return;
		}

		fileWriter << "Alpha : " << "\n";
		for (int i = 0; i < p_.GetDemand().size(); i++) {
			fileWriter << "alpha[" << i << "] = " << cplex_.getValue(alphaVar_[i]) << "\n";
		}

		fileWriter << "Beta : " << "\n";
		for (int i = 0; i < p_.GetTravelArcsSet().size(); i++) {
			fileWriter << "beta[" << i << "] = " << cplex_.getValue(betaVar_[i]) << "\n";
		}

		fileWriter << "Gamma : " << "\n";
		for (int pp = 0; pp < p_.GetPortSet().size(); pp++) {
			for (int t = 1; t < p_.GetTimePointSet().size(); t++) {
				fileWriter << "gamma[" << pp << "][" << t << "] = " << cplex_.getValue(gammaVar_[pp][t]) << "\n";
			}
		}

		fileWriter.close();
	}


	void DualProblem::exportModel(const std::string& filename) {
		try {
			cplex_.exportModel(filename.c_str());
		}
		catch (const IloException& ex) {
			std::cerr << "Concert Error: " << ex << std::endl;
		}
	}




}