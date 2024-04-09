#include "SubDerModel.h"

namespace fleetdeployment {


	SubDerModel::SubDerModel(const InputData& in, const Parameter& p)
		: in_(in), p_(p)
		, uValue_(p_.GetDemand().size())
	{
		try {

			env_ = IloEnv();
			model_ = IloModel(env_);
			cplex_ = IloCplex(model_);


			cplex_.setOut(env_.getNullStream());
			cplex_.setParam(IloCplex::Param::WorkMem, MaxWorkMem);
			cplex_.setParam(IloCplex::Param::TimeLimit, MIPTimeLimit);
			cplex_.setParam(IloCplex::Param::MIP::Tolerances::MIPGap, MIPGapLimit);
			cplex_.setParam(IloCplex::Param::Threads, MaxThreads);

			// Create basic decision variables and add basic constraints
			SetDecisionVars();
			SetObjective();
			SetConstraints();
		}
		catch (const IloException& e) {
			std::cerr << "Error: " << e.getMessage() << std::endl;
		}
	}

#include <ilcplex/ilocplex.h>
#include <vector>
#include <string>

	// Assuming the rest of your class definition is as provided...

	void SubDerModel::SetDecisionVars() {
		// Initializing vVar_
		vVar_.resize(p_.GetVesselSet().size());
		for (size_t h = 0; h < p_.GetVesselSet().size(); ++h) {
			vVar_[h].resize(p_.GetVesselRouteSet().size());
			for (size_t r = 0; r < p_.GetVesselRouteSet().size(); ++r) {
				std::string varName = "v(" + std::to_string(h + 1) + "," + std::to_string(r + 1) + ")";
				vVar_[h][r] = IloIntVar(env_, 0, 1, varName.c_str());
			}
		}

		// Initializing xVar_, yVar_, zVar_, and gVar_
		xVar_.resize(p_.GetDemand().size());
		yVar_.resize(p_.GetDemand().size());
		zVar_.resize(p_.GetDemand().size());
		gVar_.resize(p_.GetDemand().size());

		for (size_t i = 0; i < p_.GetDemand().size(); ++i) {
			Request od = in_.GetRequests()[i];

			xVar_[i].resize(od.GetNumberOfLadenPath());
			yVar_[i].resize(od.GetNumberOfLadenPath());
			zVar_[i].resize(od.GetNumberOfEmptyPath());

			for (size_t k = 0; k < od.GetNumberOfLadenPath(); ++k) {
				std::string xVarName = "x(" + std::to_string(i + 1) + "," + std::to_string(k + 1) + ")";
				xVar_[i][k] = IloNumVar(env_, 0, IloInfinity, xVarName.c_str());

				std::string yVarName = "y(" + std::to_string(i + 1) + "," + std::to_string(k + 1) + ")";
				yVar_[i][k] = IloNumVar(env_, 0, IloInfinity, yVarName.c_str());
			}
			for (size_t k = 0; k < od.GetNumberOfEmptyPath(); ++k) {
				std::string zVarName = "z(" + std::to_string(i + 1) + "," + std::to_string(k + 1) + ")";
				zVar_[i][k] = IloNumVar(env_, 0, IloInfinity, zVarName.c_str());
			}

			std::string gVarName = "g(" + std::to_string(i + 1) + ")";
			gVar_[i] = IloNumVar(env_, 0, IloInfinity, gVarName.c_str());
		}
	}


	void SubDerModel::SetObjective() {
		IloEnv env = cplex_.getEnv();
		IloExpr Obj(env_);

		// item1: Operating Cost
		// Iterate over vessel routes (r∈R) and vessel paths (w∈Ω)
		for (int r = 0; r < p_.GetVesselRouteSet().size(); ++r) {
			for (int w = 0; w < p_.GetVesselPathSet().size(); ++w) {
				// Iterate over vessel set (h∈H)
				for (int h = 0; h < p_.GetVesselSet().size(); ++h) {
					// Add terms for vessel operating cost
					Obj += p_.GetVesselTypeAndShipRoute()[h][r]
						* p_.GetShipRouteAndVesselPath()[r][w]
						* p_.GetVesselOperationCost()[h]
						* vVar_[h][r];
				}
			}
		}

		// item2: Penalty Cost of unsatisfied Demand (c2i*Gi)
		// Iterate over all demands (i)
		for (int i = 0; i < p_.GetDemand().size(); ++i) {
			// Add terms for penalty cost
			Obj += p_.GetPenaltyCostForDemand()[i] * gVar_[i];

			Request od = in_.GetRequests()[i];
			// Laden and empty path costs
			// Iterate over laden paths (φi)
			for (int k = 0; k < od.GetNumberOfLadenPath(); ++k) {
				int j = od.GetLadenPathIndexes()[k];
				// Add terms for demurrage of self-owned and leased containers and rental cost on laden paths
				Obj += p_.GetLadenPathCost()[j] * xVar_[i][k];
				Obj += p_.GetLadenPathCost()[j] * yVar_[i][k];
				Obj += p_.GetRentalCost() * p_.GetTravelTimeOnPath()[j] * yVar_[i][k];
			}

			// Iterate over empty paths (θi)
			for (int k = 0; k < od.GetNumberOfEmptyPath(); ++k) {
				int j = od.GetEmptyPathIndexes()[k];
				// Add terms for demurrage of self-owned containers for empty path
				Obj += p_.GetEmptyPathCost()[j] * zVar_[i][k];
			}
		}

		// Add the objective function to the model
		model_.add(IloMinimize(env, Obj));
	}


	//(15)each ship route selected one vessel type
	void SubDerModel::SetConstraint0() {

		// r∈R (Iterate over vessel routes)
		for (int r = 0; r < p_.GetVesselRouteSet().size(); ++r) {
			IloExpr left(env_);

			// h∈H (Iterate over vessel set)
			for (int h = 0; h < p_.GetVesselSet().size(); ++h) {
				// Add terms to the expression
				left += p_.GetVesselTypeAndShipRoute()[h][r] * vVar_[h][r];
			}

			// Constraint name
			std::string constr_name = "C0(" + std::to_string(r + 1) + ")";
			// Add equality constraint to the model
			model_.add(IloRange(env_, 1, left, 1, constr_name.c_str()));
		}
	}


	//(21)demand equation
#include <ilcplex/ilocplex.h>
#include <vector>
#include <string>

// Inside the SubDerModel class...

	void SubDerModel::SetConstraint1() {
		C1_.resize(p_.GetDemand().size());
		IloEnv env = cplex_.getEnv();

		// ∀i∈I (Iterate over all demands)
		for (int i = 0; i < p_.GetDemand().size(); ++i) {
			IloExpr left(env_);
			Request od = in_.GetRequests()[i];

			// φ (Iterate over laden paths)
			for (int k = 0; k < od.GetNumberOfLadenPath(); ++k) {
				int j = od.GetLadenPathIndexes()[k];
				left += xVar_[i][k] + yVar_[i][k];
			}

			// Add gVar to the expression
			left += gVar_[i];

			// Construct constraint name
			std::string constr_name = "C1(" + std::to_string(i + 1) + ")";
			// Add equality constraint to the model
			C1_[i] = IloRange(env, left, p_.GetDemand()[i] + p_.GetMaximumDemandVariation()[i] * uValue_[i], constr_name.c_str());
			model_.add(C1_[i]);
		}
	}


	//(22) Vessel Capacity Constraint
	void SubDerModel::SetConstraint2() {
		IloEnv env = cplex_.getEnv();

		// ∀<n,n'>∈A' (Iterate over all travel arcs)
		for (int nn = 0; nn < p_.GetTravelArcsSet().size(); ++nn) {
			IloExpr left(env_);

			// i∈I (Iterate over all demands)
			for (int i = 0; i < p_.GetDemand().size(); ++i) {
				Request od = in_.GetRequests()[i];

				// φ (Iterate over laden paths)
				for (int k = 0; k < od.GetNumberOfLadenPath(); ++k) {
					int j = od.GetLadenPathIndexes()[k];
					left += p_.GetArcAndPath()[nn][j] * xVar_[i][k];
					left += p_.GetArcAndPath()[nn][j] * yVar_[i][k];
				}

				// θ (Iterate over empty paths)
				for (int k = 0; k < od.GetNumberOfEmptyPath(); ++k) {
					int j = od.GetEmptyPathIndexes()[k];
					left += p_.GetArcAndPath()[nn][j] * zVar_[i][k];
				}
			}

			// Vessel capacity
			// r∈R (Iterate over vessel routes)
			for (int r = 0; r < p_.GetVesselRouteSet().size(); ++r) {
				// ω∈Ω (Iterate over vessel paths)
				for (int w = 0; w < p_.GetVesselPathSet().size(); ++w) {
					// h∈H (Iterate over vessel set)
					for (int h = 0; h < p_.GetVesselSet().size(); ++h) {
						left -= p_.GetArcAndVesselPath()[nn][w]
							* p_.GetShipRouteAndVesselPath()[r][w]
							* p_.GetVesselTypeAndShipRoute()[h][r]
							* p_.GetVesselCapacity()[h]
							* vVar_[h][r];
					}
				}
			}

			// Construct constraint name
			std::string constr_name = "C3(" + std::to_string(nn + 1) + ")";
			// Add less-than-or-equal-to constraint to the model
			model_.add(IloRange(env, left, 0, constr_name.c_str()));
		}
	}

	// (24)Containers flow conservation
	void SubDerModel::SetConstraint3() {
		// p ∈ P (Iterate over ports)
		for (int pp = 0; pp < p_.GetPortSet().size(); ++pp) {
			// t ∈ T (Iterate over time points)
			for (int t = 1; t < p_.GetTimePointSet().size(); ++t) {
				IloExpr left(env_);

				// i ∈ I (Iterate over demands)
				for (int i = 0; i < p_.GetDemand().size(); ++i) {
					Request od = in_.GetRequests()[i];

					// Input Z flow:
					if (p_.GetPortSet()[pp] == p_.GetOriginOfDemand()[i]) {
						for (int k = 0; k < od.GetNumberOfEmptyPath(); ++k) {
							int j = od.GetEmptyPathIndexes()[k];
							for (int nn = 0; nn < p_.GetTravelArcsSet().size(); ++nn) {
								if (in_.GetTravelArcs()[nn].GetDestinationPort() == p_.GetPortSet()[pp] &&
									in_.GetTravelArcs()[nn].GetDestinationTime() <= t &&
									in_.GetTravelArcs()[nn].GetDestinationTime() >= 1) {
									left += p_.GetArcAndPath()[nn][j] * zVar_[i][k];
								}
							}
						}
					}

					// Input flow X
					if (p_.GetPortSet()[pp] == p_.GetDestinationOfDemand()[i]) {
						for (int k = 0; k < od.GetNumberOfLadenPath(); ++k) {
							int j = od.GetLadenPathIndexes()[k];
							for (int nn = 0; nn < p_.GetTravelArcsSet().size(); ++nn) {
								if (in_.GetTravelArcs()[nn].GetDestinationPort() == p_.GetPortSet()[pp] &&
									in_.GetTravelArcs()[nn].GetDestinationTime() <= t - p_.GetTurnOverTimeSet()[pp] &&
									in_.GetTravelArcs()[nn].GetDestinationTime() >= 1) {
									left += p_.GetArcAndPath()[nn][j] * xVar_[i][k];
								}
							}
						}
					}

					// Output flow X
					if (p_.GetPortSet()[pp] == p_.GetOriginOfDemand()[i]) {
						for (int k = 0; k < od.GetNumberOfLadenPath(); ++k) {
							int j = od.GetLadenPathIndexes()[k];
							for (int nn = 0; nn < p_.GetTravelArcsSet().size(); ++nn) {
								if (in_.GetTravelArcs()[nn].GetOriginPort() == p_.GetPortSet()[pp] &&
									in_.GetTravelArcs()[nn].GetOriginTime() <= t &&
									in_.GetTravelArcs()[nn].GetOriginTime() >= 1) {
									left -= p_.GetArcAndPath()[nn][j] * xVar_[i][k];
								}
							}
						}
					}

					// Output Flow Z
					for (int k = 0; k < od.GetNumberOfEmptyPath(); ++k) {
						int j = od.GetEmptyPathIndexes()[k];
						for (int nn = 0; nn < p_.GetTravelArcsSet().size(); ++nn) {
							if (in_.GetTravelArcs()[nn].GetOriginPort() == p_.GetPortSet()[pp] &&
								in_.GetTravelArcs()[nn].GetOriginTime() <= t &&
								in_.GetTravelArcs()[nn].GetOriginTime() >= 1) {
								left -= p_.GetArcAndPath()[nn][j] * zVar_[i][k];
							}
						}
					}
				}

				// Construct constraint name
				std::string constr_name = "C3(" + std::to_string(pp + 1) + "," + std::to_string(t) + ")";
				// Add greater-than-or-equal-to constraint to the model
				model_.add(IloRange(env_, -p_.GetInitialEmptyContainer()[pp], left, IloInfinity, constr_name.c_str()));
			}
		}
	}



	void SubDerModel::SolveModel() {
		try {
			// Uncomment the following line if you want to export the model
			// cplex_.exportModel("SDP1.lp");

			if (cplex_.solve()) {
				SetObjVal(cplex_.getObjValue());
				SetVVarValue();
				SetOperationCost();
			}
			else {
				std::cout << "No solution" << std::endl;
			}
		}
		catch (const IloException& ex) {
			std::cout << "Concert Error: " << ex.getMessage() << std::endl;
		}
	}

	// Assume SetObjVal, SetVVarValue, and SetOperationCost are correctly implemented


	void SubDerModel::changeConstraints(const std::vector<double>& uValue) {
		this->uValue_ = uValue; // Assuming uValue_ is a std::vector<double> member of SubDerModel

		// Change Demand Equation Constraint (Constraint1)'s Right Coefficients
		// ∀i∈I (Iterate over all demands)
		for (int i = 0; i < p_.GetDemand().size(); ++i) {
			double newBound = p_.GetDemand()[i] + p_.GetMaximumDemandVariation()[i] * this->uValue_[i];
			C1_[i].setBounds(newBound, newBound);
		}
	}


	void SubDerModel::SetVVarValue() {
		// Resize the 2D vector to match the dimensions of the vessel set and vessel route set
		vVarValue.resize(p_.GetVesselSet().size(), std::vector<int>(p_.GetVesselRouteSet().size()));

		// Iterate over vessel set and vessel route set
		for (int h = 0; h < p_.GetVesselSet().size(); ++h) {
			for (int r = 0; r < p_.GetVesselRouteSet().size(); ++r) {
				// Get the value of the decision variable and cast it to an integer
				vVarValue[h][r] = static_cast<int>(cplex_.getValue(vVar_[h][r]) + 0.5);
			}
		}
	}


	void SubDerModel::SetOperationCost() {
		double operCost = 0;
		// r∈R
		for (int r = 0; r < p_.GetVesselRouteSet().size(); r++)
		{
			// h∈H
			for (int h = 0; h < p_.GetVesselSet().size(); h++)
			{
				for (int w = 0; w < p_.GetVesselPathSet().size(); w++) {
					operCost += p_.GetVesselOperationCost()[h]
						* p_.GetShipRouteAndVesselPath()[r][w]
						* p_.GetVesselTypeAndShipRoute()[h][r]
						* (int)(cplex_.getValue(vVar_[h][r]) + 0.5);
				}
			}
		}
		operationCost = operCost;
	}




}