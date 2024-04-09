#include "DualSubProblem.h"
#include <SubProblem.h>

namespace fleetdeployment {


	DualSubProblem::DualSubProblem(const InputData& in, const Parameter& p, int tau)
		: in_(in), p_(p), tau_(tau), 
		vVarValue_(p_.GetVesselSet().size(), std::vector<int>(p_.GetVesselRouteSet().size(), 0)) {
		try {
			env_ = IloEnv();
			model_ = IloModel(env_);
			cplex_ = IloCplex(model_);


			cplex_.setOut(env_.getNullStream());
			cplex_.setParam(IloCplex::WorkMem, MaxWorkMem);
			cplex_.setParam(IloCplex::TiLim, MIPTimeLimit);
			cplex_.setParam(IloCplex::EpGap, MIPGapLimit);
			cplex_.setParam(IloCplex::Threads, MaxThreads);

			SetDecisionVars();
			SetObjective();
			SetConstraints();
		}
		catch (const IloException& e) {
			std::cerr << "Error: " << e.getMessage() << std::endl;
		}
	}

	DualSubProblem::~DualSubProblem()
	{
	}



	void DualSubProblem::SetDecisionVars() {
		// Create dual variables
		alphaVar_.resize(p_.GetDemand().size());
		betaVar_.resize(p_.GetTravelArcsSet().size());
		gammaVar_.resize(p_.GetPortSet().size(), std::vector<IloNumVar>(p_.GetTimePointSet().size()));

		// Create u variables: u[i] ∈ {0, 1}
		// The uncertain request can be inferred by u
		// f[i] = nf[i] + vf[i] * u[i]
		miuVar_.resize(p_.GetDemand().size());

		// Create auxiliary variables: λ[i] = α[i] * u[i]
		lambdaVar_.resize(p_.GetDemand().size());

		std::string varName;
		for (int i = 0; i < p_.GetDemand().size(); i++) {
			varName = "alpha(" + std::to_string(p_.GetDemand()[i]) + ")";
			alphaVar_[i] = IloNumVar(env_, -IloInfinity, p_.GetPenaltyCostForDemand()[i], varName.c_str());

			varName = "lambda(" + std::to_string(p_.GetDemand()[i]) + ")";
			lambdaVar_[i] = IloNumVar(env_, -IloInfinity, IloInfinity, varName.c_str());

			varName = "u(" + std::to_string(p_.GetDemand()[i]) + ")";
			miuVar_[i] = IloBoolVar(env_, varName.c_str());
		}

		for (int nn = 0; nn < p_.GetTravelArcsSet().size(); nn++) {
			// β <= 0
			varName = "beta(" + std::to_string(p_.GetTravelArcsSet()[nn]) + ")";
			betaVar_[nn] = IloNumVar(env_, -IloInfinity, 0, varName.c_str());
		}

		for (int pp = 0; pp < p_.GetPortSet().size(); pp++) {
			for (int t = 1; t < p_.GetTimePointSet().size(); t++) {
				// γ >= 0
				varName = "gamma(" + (p_.GetPortSet()[pp]) + ")(" + std::to_string(p_.GetTimePointSet()[t]) + ")";
				gammaVar_[pp][t] = IloNumVar(env_, 0, IloInfinity, varName.c_str());
			}
		}
	}


	void DualSubProblem::SetObjective() {
		IloExpr expr(env_);

		// I. Part one: sum(normal_demand * alpha + max_var_demand * lambda)
		for (int i = 0; i < p_.GetDemand().size(); i++) {
			expr += p_.GetDemand()[i] * alphaVar_[i];
			expr += p_.GetMaximumDemandVariation()[i] * lambdaVar_[i];
		}

		// II. Sum (vessel capacity * V[h][r] * beta[arc])
		for (int n = 0; n < p_.GetTravelArcsSet().size(); n++) {
			double capacity = 0;
			for (int w = 0; w < p_.GetVesselPathSet().size(); w++) {
				int r = in_.GetVesselPathSet()[w].GetRouteID() - 1;
				for (int h = 0; h < p_.GetVesselSet().size(); h++) {
					capacity += p_.GetArcAndVesselPath()[n][w] *
						p_.GetShipRouteAndVesselPath()[r][w] *
						p_.GetVesselTypeAndShipRoute()[h][r] *
						p_.GetVesselCapacity()[h] *
						vVarValue_[h][r];
				}
			}
			expr += capacity * betaVar_[n];
		}

		// III. Part three
		for (int pp = 0; pp < p_.GetPortSet().size(); pp++) {
			for (int t = 1; t < p_.GetTimePointSet().size(); t++) {
				expr -= p_.GetInitialEmptyContainer()[pp] * gammaVar_[pp][t];
			}
		}

		obj_ = IloMaximize(env_, expr);
		model_.add(obj_);
	}


	void DualSubProblem::changeObjectiveVCoefficients(const std::vector<std::vector<int>>& vValue) {
		vVarValue_ = vValue;

		// II. Sum (vessel capacity * V[h][r] * beta[arc])
		for (int n = 0; n < p_.GetTravelArcsSet().size(); n++) {
			double capacity = 0;
			for (int w = 0; w < p_.GetVesselPathSet().size(); w++) {
				int r = in_.GetVesselPathSet()[w].GetRouteID() - 1;
				for (int h = 0; h < p_.GetVesselSet().size(); h++) {
					capacity += p_.GetArcAndVesselPath()[n][w] *
						p_.GetShipRouteAndVesselPath()[r][w] *
						p_.GetVesselTypeAndShipRoute()[h][r] *
						p_.GetVesselCapacity()[h] *
						vVarValue_[h][r];
				}
			}
			obj_.setLinearCoef(betaVar_[n], capacity);
		}
	}



	void DualSubProblem::changeVVarCoefficients(const std::vector<std::vector<int>>& vValue) {
		changeObjectiveVCoefficients(vValue);
	}


	void DualSubProblem::SetConstraints() {
		// dual constraints
		SetConstraint1();
		SetConstraint2();
		SetConstraint3();
		SetConstraint4();

		// uncertain Set
		SetConstraint5();

		// linearize constraints
		// λ<=α
		SetConstraint6();
		// λ>= α-(1-u)M
		SetConstraint7();
		// λ<= M*u
		SetConstraint8();
		// λ>=- M*u
		SetConstraint9();
	}




	// C1------X
	void DualSubProblem::SetConstraint1() {
		// ∀i∈I
		for (int i = 0; i < p_.GetDemand().size(); i++) {
			// ∀φ∈Φi
			Request OD = in_.GetRequests()[i];
			for (int k = 0; k < OD.GetNumberOfLadenPath(); k++) {
				int j = OD.GetLadenPathIndexes()[k];

				IloExpr left(env_);

				// first item :
				left += alphaVar_[i];

				// second item :
				// <n,n'> ∈A'
				for (int nn = 0; nn < p_.GetTravelArcsSet().size(); nn++) {
					left += p_.GetArcAndPath()[nn][j] * betaVar_[nn];
				}

				// third item :
				// t∈T
				for (int t = 1; t < p_.GetTimePointSet().size(); t++) {
					// p ∈P
					for (int pp = 0; pp < p_.GetPortSet().size(); pp++) {
						// p == d(i)
						if (p_.GetPortSet()[pp] == p_.GetDestinationOfDemand()[i]) {
							// <n,n'>∈A'
							for (int n = 0; n < p_.GetTravelArcsSet().size(); n++) {
								// p(n') == p
								// 1 <= t(n') <= t - sp
								if (in_.GetTravelArcs()[n].GetDestinationPort() == p_.GetPortSet()[pp]
									&& in_.GetTravelArcs()[n].GetDestinationTime() <= t - p_.GetTurnOverTimeSet()[pp]
									&& in_.GetTravelArcs()[n].GetDestinationTime() >= 1) {
									left += p_.GetArcAndPath()[n][j] * gammaVar_[pp][t];
								}
							}
						}
						// p == o(i)
						else if (p_.GetPortSet()[pp] == p_.GetOriginOfDemand()[i]) {
							// <n,n'>∈A'
							for (int n = 0; n < p_.GetTravelArcsSet().size(); n++) {
								// p(n) == p
								// 1 <= t(n) <= t
								if (in_.GetTravelArcs()[n].GetOriginPort() == p_.GetPortSet()[pp]
									&& in_.GetTravelArcs()[n].GetOriginTime() <= t
									&& in_.GetTravelArcs()[n].GetOriginTime() >= 1) {
									left -= p_.GetArcAndPath()[n][j] * gammaVar_[pp][t];
								}
							}
						}
					}
				}

				IloRange constraint = (left <= p_.GetLadenPathCost()[j]);
				std::string constr_name = "C1_" + std::to_string(j + 1);
				constraint.setName(constr_name.c_str());
				model_.add(constraint);
			}
		}
	}


	// C2------Y
	void DualSubProblem::SetConstraint2() {
		// ∀i∈I
		for (int i = 0; i < p_.GetDemand().size(); i++) {
			// ∀φ∈Φi
			for (int k1 = 0; k1 < in_.GetRequests()[i].GetNumberOfLadenPath(); k1++) {
				int j = in_.GetRequests()[i].GetLadenPathIndexes()[k1];

				IloExpr left(env_);

				// item1:
				left += alphaVar_[i];

				// <n,n'>∈A'
				for (int n = 0; n < p_.GetTravelArcsSet().size(); n++) {
					// item2:
					left += p_.GetArcAndPath()[n][j] * betaVar_[n];
				}

				IloRange constraint = (left <= p_.GetRentalCost() * p_.GetTravelTimeOnPath()[j] + p_.GetLadenPathCost()[j]);
				std::string constr_name = "C2_" + std::to_string(i + 1) + "_" + std::to_string(j + 1);
				constraint.setName(constr_name.c_str());
				model_.add(constraint);
			}
		}
	}


	// C3------Z
	void DualSubProblem::SetConstraint3() {
		// i∈I
		for (int i = 0; i < p_.GetDemand().size(); i++) {
			//  θ∈Θi
			for (int k1 = 0; k1 < in_.GetRequests()[i].GetNumberOfEmptyPath(); k1++) {
				int j = in_.GetRequests()[i].GetEmptyPathIndexes()[k1];

				IloExpr left(env_);

				// <n,n'>∈A'
				for (int n = 0; n < p_.GetTravelArcsSet().size(); n++) {
					// add item1:
					left += p_.GetArcAndPath()[n][j] * betaVar_[n];

					// t∈T
					for (int t = 1; t < p_.GetTimePointSet().size(); t++) {
						// p∈P
						for (int pp = 0; pp < p_.GetPortSet().size(); pp++) {
							// p == o(i)
							if (p_.GetPortSet()[pp] == p_.GetOriginOfDemand()[i]) {
								// add item2:
								//p(n') == p
								// 1<=t(n')<= t
								if (in_.GetTravelArcs()[n].GetDestinationPort() == p_.GetPortSet()[pp]
									&& in_.GetTravelArcs()[n].GetDestinationTime() <= t
									&& in_.GetTravelArcs()[n].GetDestinationTime() >= 1) {
									left += p_.GetArcAndPath()[n][j] * gammaVar_[pp][t];
								}
							}

							// p
							// add item3:
							// p(n) == p
							// 1<= t(n)<=t
							if (in_.GetTravelArcs()[n].GetOriginPort() == p_.GetPortSet()[pp]
								&& in_.GetTravelArcs()[n].GetOriginTime() <= t
								&& in_.GetTravelArcs()[n].GetOriginTime() >= 1) {
								left += -p_.GetArcAndPath()[n][j] * gammaVar_[pp][t];
							}
						}
					}
				}

				IloRange constraint = (left <= p_.GetEmptyPathCost()[j]);
				std::string constr_name = "C3_2_" + std::to_string(j + 1);
				constraint.setName(constr_name.c_str());
				model_.add(constraint);
			}
		}
	}



	// C4------G
	void DualSubProblem::SetConstraint4() {
		for (int i = 0; i < p_.GetDemand().size(); i++) {
			IloRange constraint = (alphaVar_[i] <= p_.GetPenaltyCostForDemand()[i]);
			std::string constr_name = "C4_" + std::to_string(i + 1);
			constraint.setName(constr_name.c_str());
			model_.add(constraint);
		}
	}




	// budget uncertain Set
	void DualSubProblem::SetConstraint5() {
		IloExpr left(env_);
		for (int i = 0; i < p_.GetDemand().size(); i++) {
			left += miuVar_[i];
		}
		IloRange constraint = (left == tau_);
		constraint.setName("C5");
		model_.add(constraint);
		left.end();
	}



	// λ[i] <= α[i]
	void DualSubProblem::SetConstraint6() {
		for (int i = 0; i < p_.GetDemand().size(); i++) {
			IloExpr left(env_);
			left += lambdaVar_[i];
			left -= alphaVar_[i];
			IloRange constraint(left <= 0);
			std::string constr_name = "C6_" + std::to_string(i);
			constraint.setName(constr_name.c_str());
			model_.add(constraint);
			left.end();
		}
	}


	// λ[i] >= α[i] - M*(1-u[i])
	// λ[i] - M*u[i] - α[i] >= - M
	void DualSubProblem::SetConstraint7() {
		int M = std::numeric_limits<int>::max();

		for (int i = 0; i < p_.GetDemand().size(); i++) {
			IloExpr left(env_);
			left += lambdaVar_[i];
			left -= M * miuVar_[i];
			left -= alphaVar_[i];
			IloRange constraint = (left >= -M);
			std::string constr_name = "C7_" + std::to_string(i);
			constraint.setName(constr_name.c_str());
			model_.add(constraint);
			left.end();
		}
	}


	// λ[i] <= u[i]*M
	void DualSubProblem::SetConstraint8() {
		int M = std::numeric_limits<int>::max();

		for (int i = 0; i < p_.GetDemand().size(); i++) {
			IloExpr left(env_);
			left += lambdaVar_[i];
			left -= M * miuVar_[i];
			IloRange constraint = (left <= 0);
			std::string constr_name = "C8_" + std::to_string(i);
			constraint.setName(constr_name.c_str());
			model_.add(constraint);
			left.end();
		}
	}

	// λ[i] >= - u[i]*M
	void DualSubProblem::SetConstraint9() {
		int M = std::numeric_limits<int>::max();

		for (int i = 0; i < p_.GetDemand().size(); i++) {
			IloExpr left(env_);
			left += lambdaVar_[i];
			left += M * miuVar_[i];
			IloRange constraint = (left >= 0);
			std::string constr_name = "C9_" + std::to_string(i);
			constraint.setName(constr_name.c_str());
			model_.add(constraint);
			left.end();
		}
	}


	void DualSubProblem::SolveModel() {
		try {
			clock_t startTime = clock();
			cplex_.solve();
			clock_t endTime = clock();

			if (cplex_.getStatus() == IloAlgorithm::Optimal) {
				SetObjVal(cplex_.getObjValue());
				SetMipGap(cplex_.getMIPRelativeGap());

				std::vector<int>  uValue(p_.GetDemand().size(), 0);
				std::vector<double> request(p_.GetDemand().size(), 0.0);

				for (int i = 0; i < p_.GetDemand().size(); i++) {
					if (cplex_.getValue(miuVar_[i]) != 0.0) {
						request[i] = std::round(cplex_.getValue(miuVar_[i]));
						uValue[i] = static_cast<int>(request[i]);
					}
				}
				SetUVarValue(uValue);

				if (DebugEnable && DualSubEnable) {
					std::cout << "------------------------------------------------------------------------" << std::endl;
					std::cout << "SolveTime = " << (endTime - startTime) / (double)CLOCKS_PER_SEC << std::endl;
					PrintSolution();
					std::cout << "------------------------------------------------------------------------" << std::endl;

					SubProblem sp(in_, p_, uVarValue_);
					sp.changeConstraintCoefficients(vVarValue_, uVarValue_);
					sp.solveModel();
					sp.end();
					std::cout << "SP-Obj = " << sp.GetObjVal() << std::endl;
					std::cout << "DSP-Obj = " << GetObjVal() << std::endl;
				}

				Scene_.SetRequest(request);
			}
			else {
				std::cout << "DualSubProblem No Solution" << std::endl;
			}
		}
		catch (IloException& ex) {
			std::cerr << "Concert Error: " << ex << std::endl;
		}
	}


	void DualSubProblem::PrintSolution() {
		std::cout << "The Worst Case(DSP) : " << "(tau = " << tau_ << ")" << std::endl;
		for (int i = 0; i < p_.GetDemand().size(); i++) {
			if (uVarValue_[i] != 0) {
				std::cout << i << "(" << uVarValue_[i] << ")\t";
			}
		}
		std::cout << std::endl;
	}




	// here constant item is calculated to be (sub objective - second item)
	// the second item contains the first stage decision V[h][r]
	// the first item contains the second stage decision α[i] and β[arc]
	double DualSubProblem::GetConstantItem() {
		double constantItem = 0;
		for (int n = 0; n < p_.GetTravelArcsSet().size(); n++) {
			// r ∈ R
			for (int w = 0; w < p_.GetVesselPathSet().size(); w++) {
				int r = in_.GetVesselPathSet()[w].GetRouteID() - 1;
				// r(w) = r
				for (int h = 0; h < p_.GetVesselSet().size(); h++) {
					// vValue[v][r] : come from the solution of the master problem
					constantItem += p_.GetArcAndVesselPath()[n][w]
						* p_.GetShipRouteAndVesselPath()[r][w]
						* p_.GetVesselTypeAndShipRoute()[h][r]
						* p_.GetVesselCapacity()[h]
						* vVarValue_[h][r]
						* cplex_.getValue(betaVar_[n]);
				}
			}
		}
		return this->GetObjVal() - constantItem;
	}



	// here beta is used for cutting
	// second item (which contains the first stage decision ) in the cut = sum{λ * q * β * V}
	std::vector<double> DualSubProblem::GetBetaValue() {
		std::vector<double> beta_value(p_.GetTravelArcsSet().size(), 0);
		if (cplex_.getStatus() == IloCplex::Optimal) {
			for (int i = 0; i < p_.GetTravelArcsSet().size(); i++) {
				beta_value[i] = cplex_.getValue(betaVar_[i]);
			}
		}
		return beta_value;
	}





}