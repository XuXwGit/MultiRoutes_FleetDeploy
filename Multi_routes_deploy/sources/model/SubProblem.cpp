#include "SubProblem.h"

#include <fstream>
#include <iostream>


namespace fleetdeployment {


	SubProblem::SubProblem(const InputData& in, const Parameter& p, const std::vector<double>& uValue = std::vector<double>())
		: in_(in), p_(p)
	{
		if (uValue.empty()) {
			uValue_.resize(p_.GetDemand().size(), 0);
		}
		else {
			uValue_ = uValue;
		}

		// initialize vVarValue & uValue
		vVarValue_.resize(p_.GetVesselSet().size());
		for (auto& row : vVarValue_) {
			row.resize(p_.GetVesselRouteSet().size());
		}

		try {

			env_ = IloEnv();
			model_ = IloModel(env_);
			cplex_ = IloCplex(model_);

			if (!Setting::WhetherOutputLog) {
				cplex_.setOut(env_.getNullStream());
			}
			cplex_.setParam(IloCplex::Param::WorkMem, Setting::MaxWorkMem);
			cplex_.setParam(IloCplex::Param::TimeLimit, Setting::MIPTimeLimit);
			cplex_.setParam(IloCplex::Param::MIP::Tolerances::MIPGap, Setting::MIPGapLimit);
			cplex_.setParam(IloCplex::Param::Threads, Setting::MaxThreads);

			SetDecisionVars();
			SetObjective();
			SetConstraints();
		}
		catch (const IloException& e) {
			std::cerr << e.getMessage() << std::endl;
		}
	}

	SubProblem::SubProblem(const InputData& in, const Parameter& p, const std::vector<int>& uValue = std::vector<int>()) {
		std::vector<double> uValueDouble(uValue.begin(), uValue.end());
		SubProblem(in, p, uValueDouble);
	}

	void SubProblem::SetDecisionVars() {
		xVar_.clear();
		yVar_.clear();
		zVar_.clear();

		gVar_.resize(p_.GetDemand().size());

		std::string varName;

		for (int i = 0; i < p_.GetDemand().size(); ++i) {
			Request od = in_.GetRequests()[i];

			std::vector<IloNumVar> xxxVar_k(od.GetNumberOfLadenPath());
			std::vector<IloNumVar> yyyVar_k(od.GetNumberOfLadenPath());
			std::vector<IloNumVar> zzzVar_k(od.GetNumberOfEmptyPath());

			xVar_.push_back(xxxVar_k);
			yVar_.push_back(yyyVar_k);
			zVar_.push_back(zzzVar_k);

			for (int k = 0; k < od.GetNumberOfLadenPath(); ++k) {
				varName = "x(" + std::to_string(i + 1) + ")";
				xVar_[i][k] = IloNumVar(env_, 0, IloInfinity, varName.c_str());
				varName = "y(" + std::to_string(i + 1) + ")";
				yVar_[i][k] = IloNumVar(env_, 0, IloInfinity, varName.c_str());
			}
			for (int k = 0; k < od.GetNumberOfEmptyPath(); ++k) {
				varName = "z(" + std::to_string(i + 1) + ")";
				zVar_[i][k] = IloNumVar(env_, 0, IloInfinity, varName.c_str());
			}

			varName = "g(" + std::to_string(i + 1) + ")";
			gVar_[i] = IloNumVar(env_, 0, IloInfinity, varName.c_str());
		}
	}


	// Minimize total cost about containers
	// Minimize (sum(αi*di) + sum(βj*V[j][r]) + sum(γp*L[p][t]))
	void SubProblem::SetObjective() {
		try {
			IloExpr expr(env_);

			// i
			for (int i = 0; i < p_.GetDemand().size(); ++i) {
				// item2 : Penalty Cost of unsatisfied Demand : c2i*Gi
				expr += p_.GetPenaltyCostForDemand()[i] * gVar_[i];

				Request od = in_.GetRequests()[i];
				// \phi_i
				for (int k = 0; k < od.GetNumberOfLadenPath(); ++k) {
					int j = od.GetLadenPathIndexes()[k];
					// item3 : Demurrage of self-owned and leased containers and Rental cost on laden paths
					expr += p_.GetLadenPathCost()[j] * xVar_[i][k];
					expr += p_.GetLadenPathCost()[j] * yVar_[i][k];
					expr += p_.GetRentalCost() * p_.GetTravelTimeOnPath()[j] * yVar_[i][k];
				}

				// item4: Demurrage of self-owned containers for empty path
				// \theta
				for (int k = 0; k < od.GetNumberOfEmptyPath(); ++k) {
					int j = od.GetEmptyPathIndexes()[k];
					expr += p_.GetEmptyPathCost()[j] * zVar_[i][k];
				}
			}

			IloObjective obj = IloMinimize(env_, expr);
			model_.add(obj);
		}
		catch (const IloException& e) {
			std::cerr << e.getMessage() << std::endl;
		}
	}


	void SubProblem::SetConstraints() {
		SetConstraint1();
		SetConstraint2();
		SetConstraint3();
	}

	// Demand equation :
	// C-5------α
	// C-5 = 0
	void SubProblem::SetConstraint1() {
		C1_.resize(p_.GetDemand().size());

		// ∀i∈I
		for (int i = 0; i < p_.GetDemand().size(); ++i) {
			IloExpr left(env_);

			Request od = in_.GetRequests()[i];
			// φ
			for (int k = 0; k < od.GetNumberOfLadenPath(); ++k) {
				left += xVar_[i][k];
				left += yVar_[i][k];
			}

			left += gVar_[i];

			std::string constr_name = "C1(" + std::to_string(i + 1) + ")";
			C1_[i] = IloRange(env_, p_.GetDemand()[i] + p_.GetMaximumDemandVariation()[i] * uValue_[i], left, p_.GetDemand()[i] + p_.GetMaximumDemandVariation()[i] * uValue_[i], constr_name.c_str());
			model_.add(C1_[i]);
		}
	}


	// Vessel Capacity Constraint :
	// C-6------β
	// C-6<= 0
	void SubProblem::SetConstraint2() {
		C2_.resize(p_.GetTravelArcsSet().size());

		// ∀<n,n'>∈A'
		for (int nn = 0; nn < p_.GetTravelArcsSet().size(); ++nn) {
			IloExpr left(env_);

			// i∈I
			for (int i = 0; i < p_.GetDemand().size(); ++i) {
				Request od = in_.GetRequests()[i];

				// φ
				for (int k = 0; k < od.GetNumberOfLadenPath(); ++k) {
					int j = od.GetLadenPathIndexes()[k];
					left += p_.GetArcAndPath()[nn][j] * xVar_[i][k];
					left += p_.GetArcAndPath()[nn][j] * yVar_[i][k];
				}

				// θ
				for (int k = 0; k < od.GetNumberOfEmptyPath(); ++k) {
					int j = od.GetEmptyPathIndexes()[k];
					left += p_.GetArcAndPath()[nn][j] * zVar_[i][k];
				}
			}

			// vessel capacity
			double capacity = 0;
			// r∈R
			for (int r = 0; r < p_.GetVesselRouteSet().size(); ++r) {
				// ω∈Ω
				for (int w = 0; w < p_.GetVesselPathSet().size(); ++w) {
					// h∈H
					for (int h = 0; h < p_.GetVesselSet().size(); ++h) {
						capacity += p_.GetArcAndVesselPath()[nn][w]
							* p_.GetShipRouteAndVesselPath()[r][w]
							* p_.GetVesselTypeAndShipRoute()[h][r]
							* p_.GetVesselCapacity()[h]
							* vVarValue_[h][r];
					}
				}
			}

			std::string constr_name = "C6-" + std::to_string(nn + 1);
			C2_[nn] = IloRange(env_, -IloInfinity, left, capacity, constr_name.c_str());
			model_.add(C2_[nn]);
		}
	}

	// Container Flow Conservation Constraint :
	// calculate the number of available empty self-owned containers at port p at time t
	// or
	// L[p][t] > 0 means all empty self-owned containers that repositioning to other ports (sumZ(out))
	// plus all laden self-owned containers that transport to other ports (sumX(out))
	// should no more than (L[p][t-1] + sumX(in) + sumZ(in))
	// C7------γ
	// C7>=0  =>  -C7 <= 0
	// ( item4 - item1 ) * Z + (item3 - item2) * X <= Lp0
	// Output-X + OutputZ - Input-X - Input-Z <= lp0
	void SubProblem::SetConstraint3() {
		C3_.resize(p_.GetPortSet().size());
		for (auto& c3_row : C3_) {
			c3_row.resize(p_.GetTimePointSet().size());
		}

		// ∀p∈P
		for (int pp = 0; pp < p_.GetPortSet().size(); ++pp) {
			// ∀t∈T
			for (int t = 1; t < p_.GetTimePointSet().size(); ++t) {
				IloExpr left(env_);

				// i∈I
				for (int i = 0; i < p_.GetDemand().size(); ++i) {
					Request od = in_.GetRequests()[i];

					// Input Z flow (item1)
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

					// Input flow X (item2)
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

					// Output flow X (item3)
					if (p_.GetPortSet()[pp] == p_.GetOriginOfDemand()[i]) {
						for (int k = 0; k < od.GetNumberOfLadenPath(); ++k) {
							int j = od.GetLadenPathIndexes()[k];
							for (int nn = 0; nn < p_.GetTravelArcsSet().size(); ++nn) {
								if (in_.GetTravelArcs()[nn].GetOriginPort() == p_.GetPortSet()[pp] &&
									in_.GetTravelArcs()[nn].GetOriginTime() <= t &&
									in_.GetTravelArcs()[nn].GetOriginTime() >= 1) {
									left += -p_.GetArcAndPath()[nn][j] * xVar_[i][k];
								}
							}
						}
					}

					// Output Flow Z (item4)
					for (int k = 0; k < od.GetNumberOfEmptyPath(); ++k) {
						int j = od.GetEmptyPathIndexes()[k];
						for (int nn = 0; nn < p_.GetTravelArcsSet().size(); ++nn) {
							if (in_.GetTravelArcs()[nn].GetOriginPort() == p_.GetPortSet()[pp] &&
								in_.GetTravelArcs()[nn].GetOriginTime() <= t &&
								in_.GetTravelArcs()[nn].GetOriginTime() >= 1) {
								left += -p_.GetArcAndPath()[nn][j] * zVar_[i][k];
							}
						}
					}
				}

				std::string constr_name = "C7_" + std::to_string(pp + 1)
					+ "_" + std::to_string(t);
				C3_[pp][t] = IloRange(env_, -p_.GetInitialEmptyContainer()[pp], left, IloInfinity, constr_name.c_str());
				model_.add(C3_[pp][t]);
			}
		}
	}

	void SubProblem::solveModel() {
		try {
			if (cplex_.solve()) {
				SetObjVal(cplex_.getObjValue());

				if (WhetherOutputLog) {
					writePortContainers();
					writeSolution();
				}
			}
			else {
				std::cout << "SubProblem No solution" << std::endl;
			}
		}
		catch (const IloException& ex) {
			std::cout << "Concert Error: " << ex.getMessage() << std::endl;
		}
		catch (const std::exception& e) {
			std::cerr << "Error: " << e.what() << std::endl;
			throw;
		}
	}


	double SubProblem::GetDualObjective() {
		double dualObj = 0;

		// I.part one
		for (int i = 0; i < p_.GetDemand().size(); ++i) {
			dualObj += p_.GetDemand()[i] * cplex_.getDual(C1_[i]);
			dualObj += p_.GetMaximumDemandVariation()[i] * uValue_[i] * cplex_.getDual(C1_[i]);
		}

		// II.
		for (int n = 0; n < p_.GetTravelArcsSet().size(); ++n) {
			for (int r = 0; r < p_.GetVesselRouteSet().size(); ++r) {
				for (int w = 0; w < p_.GetVesselPathSet().size(); ++w) {
					for (int h = 0; h < p_.GetVesselSet().size(); ++h) {
						dualObj += p_.GetArcAndVesselPath()[n][w] * p_.GetShipRouteAndVesselPath()[r][w]
							* p_.GetVesselTypeAndShipRoute()[h][r] * p_.GetVesselCapacity()[h] * vVarValue_[h][r] * cplex_.getDual(C2_[n]);
					}
				}
			}
		}

		// III.
		for (int pp = 0; pp < p_.GetPortSet().size(); ++pp) {
			for (int t = 1; t < p_.GetTimePointSet().size(); ++t) {
				dualObj += p_.GetInitialEmptyContainer()[pp] * cplex_.getDual(C3_[pp][t]);
			}
		}

		return dualObj;
	}

	double SubProblem::GetTotalCost() {
		double totalCost = 0;

		// Calculate vessel operation cost
		double operCost = 0;
		// r∈R
		for (int r = 0; r < p_.GetVesselRouteSet().size(); ++r) {
			// h∈H
			for (int h = 0; h < p_.GetVesselSet().size(); ++h) {
				for (int w = 0; w < p_.GetVesselPathSet().size(); ++w) {
					operCost += p_.GetVesselOperationCost()[h]
						* p_.GetShipRouteAndVesselPath()[r][w]
						* p_.GetVesselTypeAndShipRoute()[h][r]
						* vVarValue_[h][r];
				}
			}
		}

		totalCost = operCost + GetObjVal();
		return totalCost;
	}

	void SubProblem::printSolutions() {
		try {
			for (int i = 0; i < p_.GetDemand().size(); ++i) {
				for (int k = 0; k < in_.GetRequests()[i].GetNumberOfLadenPath(); ++k) {
					int j = in_.GetRequests()[i].GetLadenPathIndexes()[k];
					std::cout << "X[" << i << "][" << j << "] = " << cplex_.getValue(xVar_[i][k]) << '\t'
						<< "Y[" << i << "][" << j << "] = " << cplex_.getValue(yVar_[i][k]) << std::endl;
				}
			}
		}
		catch (const IloException& e) {
			std::cerr << "Error: " << e.getMessage() << std::endl;
		}
	}




	void SubProblem::writeDualSolution() {
		std::ofstream fileWriter("SP-D.txt");

		if (!fileWriter.is_open()) {
			std::cerr << "Unable to open file" << std::endl;
			return;
		}

		fileWriter << "Alpha : " << "\n";
		for (int i = 0; i < p_.GetDemand().size(); ++i) {
			fileWriter << "alpha[" << i << "] = " << cplex_.getDual(C1_[i]) << "\n";
		}

		fileWriter << "Beta : " << "\n";
		for (int i = 0; i < p_.GetTravelArcsSet().size(); ++i) {
			fileWriter << "beta[" << i << "] = " << cplex_.getDual(C2_[i]) << "\n";
		}

		fileWriter << "Gamma : " << "\n";
		for (int pp = 0; pp < p_.GetPortSet().size(); ++pp) {
			for (int t = 1; t < p_.GetTimePointSet().size(); ++t) {
				fileWriter << "gamma[" << pp << "][" << t << "] = " << cplex_.getDual(C3_[pp][t]) << "\n";
			}
		}

		fileWriter.close();
	}


	void SubProblem::changeObjSense(int flag) {
		try {
			IloObjective obj = cplex_.getObjective();
			if (flag > 0) {
				obj.setSense(IloObjective::Maximize);
			}
			else {
				obj.setSense(IloObjective::Minimize);
			}
		}
		catch (const IloException& e) {
			std::cerr << "Error in changeObjSense: " << e.getMessage() << std::endl;
		}
	}


	// change the coefficients of capacity constraints
	void SubProblem::changeConstraintCoefficients(const std::vector<std::vector<int>>& VValue, const std::vector<double>& uValue) {
		vVarValue_ = VValue;
		uValue_ = uValue;

		// Change Demand Equation Constraint (C1)'s Right Coefficients
		// ∀i∈I
		for (int i = 0; i < p_.GetDemand().size(); ++i) {
			double boundValue = p_.GetDemand()[i] + p_.GetMaximumDemandVariation()[i] * uValue_[i];
			C1_[i].setBounds(boundValue, boundValue);
		}

		// ∀<n,n'>∈A'
		for (int nn = 0; nn < p_.GetTravelArcsSet().size(); ++nn) {
			double capacity = 0;
			// r∈R
			for (int r = 0; r < p_.GetVesselRouteSet().size(); ++r) {
				// ω∈Ω
				for (int w = 0; w < p_.GetVesselPathSet().size(); ++w) {
					// h∈H
					for (int h = 0; h < p_.GetVesselSet().size(); ++h) {
						capacity += p_.GetArcAndVesselPath()[nn][w]
							* p_.GetVesselCapacity()[h]
							* p_.GetShipRouteAndVesselPath()[r][w]
							* p_.GetVesselTypeAndShipRoute()[h][r]
							* vVarValue_[h][r];
					}
				}
			}

			C2_[nn].setBounds(0, capacity);
		}
	}

	void SubProblem::changeConstraintCoefficients(const std::vector<std::vector<int>>& VValue, const std::vector<int>& uValue) {
		std::vector<double> uValueDouble(uValue.begin(), uValue.end());
		changeConstraintCoefficients(VValue, uValueDouble);
	}

#include <fstream>
#include <iomanip>
#include <iostream>

	void SubProblem::writeSolution() {
		std::string filename = "SP-Solution(R=" + std::to_string(in_.GetShipRouteSet().size())
			+ ")(T=" + std::to_string(p_.GetTimePointSet().size() - 1)
			+ ")(U=" + std::to_string(p_.GetUncertainDegree()) + ").txt";

		std::ofstream fileWriter(filename);

		if (!fileWriter.is_open()) {
			std::cerr << "Unable to open file" << std::endl;
			return;
		}

		double TotalOtherCost = 0;
		double TotalLadenCost = 0;
		double TotalEmptyCost = 0;
		double TotalRentalCost = 0;
		double TotalPenaltyCost = 0;

		int totalOwnedContainer = 0;
		int totalLeasedContainer = 0;
		int totalEmptyContainer = 0;
		int totalUnfulfilledContainer = 0;

		int noPathContainer = 0;

		fileWriter << "\n";

		// Iterate over requests
		for (int i = 0; i < in_.GetRequests().size(); ++i) {
			Request od = in_.GetRequests()[i];
			double totalRequestCost = 0;

			// More code to write request details...
			// You need to replace this with the actual logic to format and write each request's details.
			// For example:
			// fileWriter << "Request " << od.GetRequestID() << " details..." << std::endl;

			// Similar logic for iterating over paths, calculating costs, etc.
			// Make sure to format the output as needed and calculate the totals.

			// Update total costs
			// TotalLadenCost += ...
			// TotalEmptyCost += ...
			// TotalRentalCost += ...
			// TotalPenaltyCost += ...
			// ...
		}

		// Output summary if required
		if (WhetherPrintProcess) {
			std::cout << "TotalLadenCost = " << TotalLadenCost
				<< "\tTotalEmptyCost = " << TotalEmptyCost
				<< "\tTotalRentalCost = " << TotalRentalCost
				<< "\tTotalPenaltyCost = " << TotalPenaltyCost << std::endl;
			// Other summary details...
		}

		fileWriter << "TotalOtherCost = " << TotalOtherCost << "\n";
		fileWriter.close();

		// Set the costs
		SetLadenCost(TotalLadenCost);
		SetEmptyCost(TotalEmptyCost);
		SetRentalCost(TotalRentalCost);
		SetPenaltyCost(TotalPenaltyCost);
	}


	void SubProblem::writePortContainers() {
		std::string filename = "SP_Port-Containers(T=" + std::to_string(p_.GetTimePointSet().size() - 1) + ").txt";

		std::ofstream fileWriter(filename);

		if (!fileWriter.is_open()) {
			std::cerr << "Unable to open file" << std::endl;
			return;
		}

		std::vector<std::vector<int>> Lpt(p_.GetPortSet().size(), std::vector<int>(p_.GetTimePointSet().size(), 0));

		// Header
		fileWriter << "TimePoint\t";
		for (int pp = 0; pp < p_.GetPortSet().size(); ++pp) {
			fileWriter << p_.GetPortSet()[pp] << "\t";
		}
		fileWriter << "\n";

		// Data
		for (int t = 0; t < p_.GetTimePointSet().size(); ++t) {
			fileWriter << t << "\t";
			for (int pp = 0; pp < p_.GetPortSet().size(); ++pp) {
				// ... similar logic for calculating Lpt and writing data ...
				fileWriter << Lpt[pp][t] << "\t";
			}
			fileWriter << "\n";
		}

		fileWriter.close();
	}

}