#include "MasterProblem.h"

namespace fleetdeployment {


	MasterProblem::MasterProblem(const InputData& in, const Parameter& p)
		: in_(in), p_(p), 
		mipGap(0.0)
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

			// create basic decision and add basic constraints
			SetDecisionVars();
			SetObjective();
			SetConstraints();
		}
		catch (IloException e) {
			std::cerr << "Error: " << e.getMessage() << std::endl;
		}
	}


	MasterProblem::MasterProblem(const InputData& in, const Parameter& p, bool Reactive)
		: in_(in), p_(p), env_(), cplex_(env_) {
		if (Reactive) {
			try {
				cplex_.setOut(env_.getNullStream());
				cplex_.setParam(IloCplex::Param::WorkMem, MaxWorkMem);
				cplex_.setParam(IloCplex::Param::TimeLimit, MIPTimeLimit);
				cplex_.setParam(IloCplex::Param::MIP::Tolerances::MIPGap, MIPGapLimit);
				cplex_.setParam(IloCplex::Param::Threads, MaxThreads);

				// Create basic decision variables and add basic constraints
				SetReactiveDecisionVars();
				SetReactiveObjective();
				SetConstraints();
			}
			catch (const IloException& e) {
				std::cerr << "Error: " << e.getMessage() << std::endl;
			}
		}
	}


	// Set V[h][r]
	void MasterProblem::SetDecisionVars()
	{
		// first-stage variable :
		// v[h][r] : binary variable ���� whether vessel type h is assigned to shipping route r
		// eta : auxiliary decision variable ���� the upper bound of second-stage objective under all scene
		// Resize the 2D vector for binary variables
		vVar_.resize(p_.GetVesselSet().size());
		for (size_t h = 0; h < p_.GetVesselSet().size(); ++h) {
			vVar_[h].resize(p_.GetVesselRouteSet().size());
		}

		// Create the auxiliary decision variable
		etaVar_ = IloNumVar(env_, 0, IloInfinity, ILOFLOAT, "eta");

		// Create binary variables for each vessel and route
		std::string varName;
		for (size_t h = 0; h < p_.GetVesselSet().size(); ++h) {
			for (size_t r = 0; r < p_.GetVesselRouteSet().size(); ++r) {
				varName = "V(" + std::to_string(p_.GetVesselSet()[h]) + ")(" + std::to_string(p_.GetVesselRouteSet()[r]) + ")";
				vVar_[h][r] = IloBoolVar(env_,  varName.c_str());
			}
		}
	}

	void MasterProblem::SetObjective()
	{
		IloExpr expr(env_);

		// add fixed operating cost to maintain shipping route
		// w
		// h�ʦ�
		for (int h = 0; h < p_.GetVesselSet().size(); ++h)
		{
			for (int w = 0; w < p_.GetVesselPathSet().size(); ++w)
			{
				// r(��) == r
				int r = in_.GetVesselPathSet()[w].GetRouteID() - 1;

				// vesselTypeAndShipRoute == 1 : r(h) = r
				//expr += p_.GetVesselTypeAndShipRoute()[h][r]
				//	* p_.GetShipRouteAndVesselPath()[r][w]
				//	* p_.GetVesselOperationCost()[h]
				//	* vVar_[h][r];
				expr += 
					p_.GetShipRouteAndVesselPath()[r][w]
					* p_.GetVesselOperationCost()[h]
					* vVar_[h][r];
			}
		}

		expr += etaVar_;

		IloObjective Obj = IloMinimize(env_, expr);

		model_.add(Obj);
	}

	void MasterProblem::SetReactiveDecisionVars()
	{
		// first-stage variable :
		// v[h][r] : binary variable ���� whether vessel type h is assigned to shipping route r
		// eta : auxiliary decision variable ���� the upper bound of second-stage objective under all scene
		// Resize the 2D vector for binary variables
		vVar_.resize(p_.GetVesselSet().size());
		for (size_t h = 0; h < p_.GetVesselSet().size(); ++h) {
			vVar_[h].resize(p_.GetVesselRouteSet().size());
		}

		// Create the auxiliary decision variable
		etaVar_ = IloNumVar(env_, 0, IloInfinity, ILOFLOAT, "Yita");

		// Create binary variables for each vessel and route
		std::string varName;
		for (size_t h = 0; h < p_.GetVesselSet().size(); ++h) {
			for (size_t r = 0; r < p_.GetVesselRouteSet().size(); ++r) {
				varName = "V(" + std::to_string(p_.GetVesselSet()[h]) + ")(" + std::to_string(p_.GetVesselRouteSet()[r]) + ")";
				vVar_[h][r] = IloIntVar(env_, 0, 1, varName.c_str());
			}
		}
		// first-stage variable :
		// v[h][w] : binary variable ���� whether vessel type h is assigned to shipping route r
		// eta : auxiliary decision variable ���� the upper bound of second-stage objective under all scene
		vVar2_.resize(p_.GetVesselSet().size());
		for (int h = 0; h < p_.GetVesselSet().size(); ++h) {
			vVar2_[h].resize(p_.GetVesselPathSet().size());
		}
		// V[h][w]
		for (int h = 0; h < p_.GetVesselSet().size(); ++h)
		{
			for (int w = 0; w < p_.GetVesselPathSet().size(); ++w)
			{
				varName = "V2(" + std::to_string(p_.GetVesselSet()[h]) + ")(" + std::to_string(p_.GetVesselPathSet()[w]) + ")";
				vVar2_[h][w] = IloIntVar(env_, 0, 1, varName.c_str());
			}
		}
	}

	void MasterProblem::SetReactiveObjective()
	{
		IloExpr expr(env_);

		// add fixed operating cost to maintain shipping route
		// r��R
		// w
		for (int w = 0; w < p_.GetVesselPathSet().size(); ++w)
		{
			int r = in_.GetVesselPathSet()[w].GetRouteID() - 1;
			// r(��) == r
			// h�ʦ�
			for (int h = 0; h < p_.GetVesselSet().size(); ++h)
			{
				// vesselTypeAndShipRoute == 1 : r(h) = r
				expr += p_.GetVesselTypeAndShipRoute()[h][r]
					* p_.GetShipRouteAndVesselPath()[r][w]
					* p_.GetVesselOperationCost()[h]
					* vVar_[h][r];
			}
		}

		// add fixed operating cost to maintain shipping route
		// r��R
		// w
		for (int w = 0; w < p_.GetVesselPathSet().size(); ++w)
		{
			int r = in_.GetVesselPathSet()[w].GetRouteID() - 1;

			// r(��) == r
			// h�ʦ�
			for (int h = 0; h < p_.GetVesselSet().size(); ++h)
			{
				// vesselTypeAndShipRoute == 1 : r(h) = r
				expr += p_.GetVesselTypeAndShipRoute()[h][r]
					* p_.GetShipRouteAndVesselPath()[r][w]
					* p_.GetVesselOperationCost()[h]
					* vVar2_[h][w];
			}
		}

		expr += etaVar_;
		IloObjective obj = IloMinimize(env_, expr);

		model_.add(obj);
	}


	void MasterProblem::SetConstraint1()
	{
		// r \in R
		for (int r = 0; r < p_.GetVesselRouteSet().size(); ++r)
		{
			IloExpr left(env_);

			// h \in H
			for (int h = 0; h < p_.GetVesselSet().size(); ++h)
			{
				// r(h) == r
				left += (p_.GetVesselTypeAndShipRoute()[h][r] * vVar_[h][r]);
			}

			IloRange c1(env_, 1, left, 1);
			std::string constr_name = "C1(" + std::to_string(r + 1) + ")";
			c1.setName(constr_name.c_str());
			model_.add(c1);
		}
	}

	/*
	cutting plane for scene k
	 */
	void MasterProblem::SetConstraint0(const std::vector<std::vector<IloNumVar>>& xVar,
		const std::vector<std::vector<IloNumVar>>& yVar,
		const std::vector<std::vector<IloNumVar>>& zVar,
		const std::vector<IloNumVar>& gVar) {
		IloExpr left(env_);

		// Iterate over all demands
		for (int i = 0; i < p_.GetDemand().size(); ++i) {
			// Penalty Cost of unsatisfied Demand
			left += p_.GetPenaltyCostForDemand()[i] * gVar[i];

			Request od = in_.GetRequests()[i];

			// Iterate over all laden paths for this demand
			for (int k = 0; k < od.GetNumberOfLadenPath(); ++k) {
				int j = od.GetLadenPathIndexes()[k];
				// Demurrage of self-owned and leased containers and Rental cost on laden paths
				left += p_.GetLadenPathCost()[j] * xVar[i][k];
				left += p_.GetLadenPathCost()[j] * yVar[i][k];
				left += p_.GetRentalCost() * p_.GetTravelTimeOnPath()[j] * yVar[i][k];
			}

			// Iterate over all empty paths for this demand
			for (int k = 0; k < od.GetNumberOfEmptyPath(); ++k) {
				int j = od.GetEmptyPathIndexes()[k];
				left += p_.GetEmptyPathCost()[j] * zVar[i][k];
			}
		}

		left -= etaVar_;

		IloRange c0 = (left <= 0);
		c0.setName("Eta");
		model_.add(c0);
	}

	/*
	demand equation
	/sum{X+Y} + G = f
	 */
	void MasterProblem::SetConstraint4(const std::vector<std::vector<IloNumVar>>& xVar,
		const std::vector<std::vector<IloNumVar>>& yVar,
		const std::vector<IloNumVar>& gVar,
		const std::vector<double>& uValue) {
		// Iterate over all demands
		for (int i = 0; i < p_.GetDemand().size(); ++i) {
			IloExpr left(env_);
			Request od = in_.GetRequests()[i];

			// Add laden path variables
			for (int k = 0; k < od.GetNumberOfLadenPath(); ++k) {
				left += xVar[i][k];
				left += yVar[i][k];
			}

			// Add unsatisfied demand variable
			left += gVar[i];

			// Create constraint name
			std::string constr_name = "C1(" + std::to_string(i + 1) + ")";

			// Add equality constraint to the model
			IloRange c4(env_, p_.GetDemand()[i] + p_.GetMaximumDemandVariation()[i] * uValue[i],
				left, p_.GetDemand()[i] + p_.GetMaximumDemandVariation()[i] * uValue[i]);
			c4.setName(constr_name.c_str());
			model_.add(c4);

			// Clean up
			left.end();
		}
	}

	/*
	vessel capacity constraint
	/sum{X+Y+Z} <= V
	 */
	void MasterProblem::SetConstraint5(const std::vector<std::vector<IloNumVar>>& xVar,
		const std::vector<std::vector<IloNumVar>>& yVar,
		const std::vector<std::vector<IloNumVar>>& zVar) {
		// ∀<n,n'>∈A'
		for (int nn = 0; nn < p_.GetTravelArcsSet().size(); ++nn) {
			IloExpr left(env_);

			// i∈I
			for (int i = 0; i < p_.GetDemand().size(); ++i) {
				Request od = in_.GetRequests()[i];

				// φ
				for (int k = 0; k < od.GetNumberOfLadenPath(); ++k) {
					int j = od.GetLadenPathIndexes()[k];
					left += p_.GetArcAndPath()[nn][j] * xVar[i][k];
					left += p_.GetArcAndPath()[nn][j] * yVar[i][k];
				}

				// θ
				for (int k = 0; k < od.GetNumberOfEmptyPath(); ++k) {
					int j = od.GetEmptyPathIndexes()[k];
					left += p_.GetArcAndPath()[nn][j] * zVar[i][k];
				}
			}

			// r \in R, w \in \Omega, h \in H_r
			for (int w = 0; w < p_.GetVesselPathSet().size(); ++w) {
				int r = in_.GetVesselPathSet()[w].GetRouteID() - 1;

				for (int h = 0; h < p_.GetVesselSet().size(); ++h) {
					if (p_.GetVesselTypeAndShipRoute()[h][r] == 1) {
						left -= p_.GetVesselTypeAndShipRoute()[h][r] *
							p_.GetShipRouteAndVesselPath()[r][w] *
							p_.GetArcAndVesselPath()[nn][w] *
							p_.GetVesselCapacity()[h] *
							vVar_[h][r];
					}
				}
			}

			std::string constr_name = "C3(" + std::to_string(nn + 1) + ")";
			model_.add(IloRange(env_, left, 0, constr_name.c_str()));
		}
	}

	void MasterProblem::SetConstraint5_Reactive1(const std::vector<std::vector<IloNumVar>>& xVar,
		const std::vector<std::vector<IloNumVar>>& yVar) {
		// ∀<n,n'>∈A'
		for (int nn = 0; nn < p_.GetTravelArcsSet().size(); ++nn) {
			IloExpr left(env_);

			// i∈I
			for (int i = 0; i < p_.GetDemand().size(); ++i) {
				Request od = in_.GetRequests()[i];

				// φ
				for (int k = 0; k < od.GetNumberOfLadenPath(); ++k) {
					int j = od.GetLadenPathIndexes()[k];

					left += (p_.GetArcAndPath()[nn][j] * xVar[i][k]);
					left += (p_.GetArcAndPath()[nn][j] * yVar[i][k]);
				}
			}

			// w \in \Omega
			// r(w) = r : p_.GetShipRouteAndVesselPath()[r][w] == 1
			for (int w = 0; w < p_.GetVesselPathSet().size(); ++w) {
				int r = in_.GetVesselPathSet()[w].GetRouteID() - 1;
				// h \in H_r
				// r(h) = r : p_.GetVesselTypeAndShippingRoute()[h][r] == 1
				for (int h = 0; h < p_.GetVesselSet().size(); ++h)
				{
					left += (-p_.GetVesselTypeAndShipRoute()[h][r]
						* p_.GetShipRouteAndVesselPath()[r][w]
						* p_.GetArcAndVesselPath()[nn][w]
						* p_.GetVesselCapacity()[h]
						, vVar_[h][r]
						);
				}
			}
			std::string constr_name = "C5-1(" + std::to_string(nn + 1) + ")";
			IloRange c5_1(env_, left, 0);
			c5_1.setName(constr_name.c_str());
			model_.add(c5_1);
		}
	}

	void MasterProblem::SetConstraint5_Reactive2(const std::vector<std::vector<IloNumVar>>& zVar) {
		// ∀<n,n'>∈A'
		for (int nn = 0; nn < p_.GetTravelArcsSet().size(); ++nn) {
			IloExpr left(env_);

			// i∈I
			for (int i = 0; i < p_.GetDemand().size(); ++i) {
				Request od = in_.GetRequests()[i];

				//θ
				for (int k = 0; k < od.GetNumberOfEmptyPath(); ++k) {
					int j = od.GetEmptyPathIndexes()[k];

					left += (p_.GetArcAndPath()[nn][j] * zVar[i][k]);
				}
			}

			// w \in \Omega
			// r(w) = r : p_.GetShipRouteAndVesselPath()[r][w] == 1
			for (int w = 0; w < p_.GetVesselPathSet().size(); ++w)
			{
				int r = in_.GetVesselPathSet()[w].GetRouteID() - 1;
				// h \in H_r
				// r(h) = r : p_.GetVesselTypeAndShippingRoute()[h][r] == 1
				for (int h = 0; h < p_.GetVesselSet().size(); ++h)
				{
					left += (-p_.GetVesselTypeAndShipRoute()[h][r]
						* p_.GetShipRouteAndVesselPath()[r][w]
						* p_.GetArcAndVesselPath()[nn][w]
						* p_.GetVesselCapacity()[h]
						* vVar2_[h][w]
						);
				}
			}
			std::string constr_name = "C5_2(" + std::to_string(nn + 1) + ")";
			IloRange c5_2(env_, left, 0);
			c5_2.setName(constr_name.c_str());
			model_.add(c5_2);
		}
	}



	//Empty containers flow balance
	//l_{pt} + /sum{ Z + X - Z - X} >= 0
	void MasterProblem::SetConstraint6(const std::vector<std::vector<IloNumVar>>& xVar, const std::vector<std::vector<IloNumVar>>& zVar) {
		// p \in P
		for (int pp = 0; pp < p_.GetPortSet().size(); ++pp)
		{
			IloExpr left(env_);
			// t \in T
			for (int t = 1; t < p_.GetTimePointSet().size(); ++t)
			{

				// i \in I
				for (int i = 0; i < p_.GetDemand().size(); ++i) {
					Request od = in_.GetRequests()[i];

					// Input Z flow:
					// (item1)
					// o(i) == p
					if (p_.GetPortSet()[pp] == (p_.GetOriginOfDemand()[i])) {
						//θi
						for (int k = 0; k < od.GetNumberOfEmptyPath(); ++k) {
							int j = od.GetEmptyPathIndexes()[k];

							// <n,n'> ∈A'
							for (int nn = 0; nn < p_.GetTravelArcsSet().size(); ++nn) {
								// p(n') == p
								// 1<= t(n')<= t
								if (in_.GetTravelArcs()[nn].GetDestinationPort() == (p_.GetPortSet()[pp])
									&& in_.GetTravelArcs()[nn].GetDestinationTime() == t) {
									left += (p_.GetArcAndPath()[nn][j] * zVar[i][k]);
								}
							}
						}
					}


					// Input flow X
					// item2
					// d(i) == p
					if (p_.GetPortSet()[pp] == (p_.GetDestinationOfDemand()[i])) {
						for (int k = 0; k < od.GetNumberOfLadenPath(); ++k) {
							int j = od.GetLadenPathIndexes()[k];

							// <n,n'>∈A'
							for (int nn = 0; nn < p_.GetTravelArcsSet().size(); ++nn) {
								// p(n‘)∈p
								// 1 <= t(n')<= t-sp
								if (in_.GetTravelArcs()[nn].GetDestinationPort() == (p_.GetPortSet()[pp])
									&& in_.GetTravelArcs()[nn].GetDestinationTime() == t - p_.GetTurnOverTimeSet()[pp]) {
									left += (p_.GetArcAndPath()[nn][j] * xVar[i][k]);
								}
							}
						}
					}


					//Output  flow X
					// item3
					// o(i) == p
					if (p_.GetPortSet()[pp] == (p_.GetOriginOfDemand()[i])) {
						// φi
						for (int k = 0; k < od.GetNumberOfLadenPath(); ++k) {
							int j = od.GetLadenPathIndexes()[k];

							// <n.n'>∈A'
							for (int nn = 0; nn < p_.GetTravelArcsSet().size(); ++nn) {
								//p(n) == p
								// t(n) <= t
								if (in_.GetTravelArcs()[nn].GetOriginPort() == (p_.GetPortSet()[pp])
									&& in_.GetTravelArcs()[nn].GetOriginTime() == t) {
									left += (-p_.GetArcAndPath()[nn][j] * xVar[i][k]);
								}
							}
						}
					}


					// Output Flow Z
					// item4
					// θ
					for (int k = 0; k < od.GetNumberOfEmptyPath(); ++k) {
						int j = od.GetEmptyPathIndexes()[k];

						// <n,n'>∈A'
						for (int nn = 0; nn < p_.GetTravelArcsSet().size(); ++nn) {
							// p(n) == p
							// t(n) <= t
							if (in_.GetTravelArcs()[nn].GetOriginPort() == (p_.GetPortSet()[pp])
								&& in_.GetTravelArcs()[nn].GetOriginTime() == t) {
								left += (-p_.GetArcAndPath()[nn][j] * zVar[i][k]);
							}
						}
					}
				}
				std::string constr_name = "C6(" + std::to_string(pp + 1) + ")(" + std::to_string(t) + ")";
				IloRange c6(env_, -p_.GetInitialEmptyContainer()[pp], left, IloInfinity, constr_name.c_str());
				c6.setName(constr_name.c_str());
				model_.add(c6);
			}
		}
	}


	void MasterProblem::addScene(const Scenario& scene_k) {
		// second-stage variable :
		// by adding
		// x[i][p][k] : continue variable ���� number of self-owned containers shipped on path p for demand i in scene k
		// y[i][p][k] : continue variable ���� number of leased containers shipped on path p for demand i in scene k
		// z[i][q][k] : continue variable ���� number of self-owned containers repositioned on path q for demand i in scene k
		// g[i][k] : continue variable ���� number of unfulfilled containers for demand i on path p in scene k
		// l[p][t][k] : continue variable ���� number of self-owned containers shipped at port p at time t in scene k
		// create decision for scenery k
		std::vector<std::vector<IloNumVar>> xxVar_k(p_.GetDemand().size());
		std::vector<std::vector<IloNumVar>> yyVar_k(p_.GetDemand().size());
		std::vector<std::vector<IloNumVar>> zzVar_k(p_.GetDemand().size());
		std::vector<IloNumVar> gVar_k(p_.GetDemand().size());

		for (int i = 0; i < p_.GetDemand().size(); ++i)
		{
			Request od = in_.GetRequests()[i];
			xxVar_k[i].resize(od.GetNumberOfLadenPath());
			yyVar_k[i].resize(od.GetNumberOfLadenPath());
			zzVar_k[i].resize(od.GetNumberOfEmptyPath());

			for (int j = 0; j < od.GetNumberOfLadenPath(); ++j) {
				xxVar_k[i][j] = IloNumVar(env_, 0, IloInfinity, ILOFLOAT);
				yyVar_k[i][j] = IloNumVar(env_, 0, IloInfinity, ILOFLOAT);
			}
			for (int j = 0; j < od.GetNumberOfEmptyPath(); ++j) {
				zzVar_k[i][j] = IloNumVar(env_, 0, IloInfinity, ILOFLOAT);
			}

			gVar_k[i] = IloNumVar(env_, 0, IloInfinity, ILOFLOAT);
		}

		std::vector<double> request = scene_k.GetRequest();

		// Assuming these methods exist and are correctly implemented
		SetConstraint0(xxVar_k, yyVar_k, zzVar_k, gVar_k);
		SetConstraint4(xxVar_k, yyVar_k, gVar_k, request);
		SetConstraint5(xxVar_k, yyVar_k, zzVar_k);
		SetConstraint6(xxVar_k, zzVar_k);
	}

	void MasterProblem::addReactiveScene(Scenario scene_k) {
		// second-stage variable :
		// by adding
		// x[i][p][k] : continue variable ���� number of self-owned containers shipped on path p for demand i in scene k
		// y[i][p][k] : continue variable ���� number of leased containers shipped on path p for demand i in scene k
		// z[i][q][k] : continue variable ���� number of self-owned containers repositioned on path q for demand i in scene k
		// g[i][k] : continue variable ���� number of unfulfilled containers for demand i on path p in scene k
		// l[p][t][k] : continue variable ���� number of self-owned containers shipped at port p at time t in scene k
		// create decision for scenery k
		std::vector<std::vector<IloNumVar>> xxVar_k(p_.GetDemand().size());
		std::vector<std::vector<IloNumVar>> yyVar_k(p_.GetDemand().size());
		std::vector<std::vector<IloNumVar>> zzVar_k(p_.GetDemand().size());
		std::vector<IloNumVar> gVar_k(p_.GetDemand().size());

		for (int i = 0; i < p_.GetDemand().size(); ++i)
		{
			Request od = in_.GetRequests()[i];
			xxVar_k[i].resize(od.GetNumberOfLadenPath());
			yyVar_k[i].resize(od.GetNumberOfLadenPath());
			zzVar_k[i].resize(od.GetNumberOfEmptyPath());

			for (int j = 0; j < od.GetNumberOfLadenPath(); ++j) {
				xxVar_k[i][j] = IloNumVar(env_, 0, IloInfinity, ILOFLOAT);
				yyVar_k[i][j] = IloNumVar(env_, 0, IloInfinity, ILOFLOAT);
			}
			for (int j = 0; j < od.GetNumberOfEmptyPath(); ++j) {
				zzVar_k[i][j] = IloNumVar(env_, 0, IloInfinity, ILOFLOAT);
			}

			gVar_k[i] = IloNumVar(env_, 0, IloInfinity, ILOFLOAT);
		}

		std::vector<double> request = scene_k.GetRequest();

		SetConstraint0(xxVar_k, yyVar_k, zzVar_k, gVar_k);
		SetConstraint4(xxVar_k, yyVar_k, gVar_k, request);
		SetConstraint5_Reactive1(xxVar_k, yyVar_k);
		SetConstraint5_Reactive2(zzVar_k);
		SetConstraint6(xxVar_k, zzVar_k);
	}

	void MasterProblem::addOptimalityCut(double constantItem, const std::vector<double>& beta_value) {
		IloExpr left(env_);
		for (int n = 0; n < p_.GetTravelArcsSet().size(); n++)
		{
			// r ∈R
			for (int r = 0; r < p_.GetVesselRouteSet().size(); r++)
			{
				for (int w = 0; w < p_.GetVesselPathSet().size(); ++w)
				{
					// r(w) = r
					for (int h = 0; h < p_.GetVesselSet().size(); ++h)
					{
						// vValue[v][r] : come from solution of master problem
						left += (p_.GetArcAndVesselPath()[n][w] * p_.GetShipRouteAndVesselPath()[r][w]
							* p_.GetVesselTypeAndShipRoute()[h][r] * p_.GetVesselCapacity()[h] * beta_value[n]
							* vVar_[h][r]);
					}
				}
			}
		}
		left -= etaVar_;
		std::string constr_name = "OptimalityCut";
		IloRange OptimalityCut(env_, left, -constantItem, constr_name.c_str());
		model_.add(OptimalityCut);
	}

	void MasterProblem::addFeasibilityCut(double constantItem, const std::vector<double>& beta_value) {
		IloExpr left(env_);
		for (int n = 0; n < p_.GetTravelArcsSet().size(); n++)
		{
			// r ∈R
			for (int r = 0; r < p_.GetVesselRouteSet().size(); r++)
			{
				for (int w = 0; w < p_.GetVesselPathSet().size(); ++w)
				{
					// r(w) = r
					for (int h = 0; h < p_.GetVesselSet().size(); ++h)
					{
						// vValue[v][r] : come from solution of master problem
						left += (p_.GetArcAndVesselPath()[n][w] * p_.GetShipRouteAndVesselPath()[r][w]
							* p_.GetVesselTypeAndShipRoute()[h][r] * p_.GetVesselCapacity()[h] * beta_value[n] * vVar_[h][r]);
					}
				}
			}
		}
		std::string constr_name = "FeasibilityCut";
		model_.add(IloRange(env_, left, -constantItem, constr_name.c_str()));

	}

	void MasterProblem::addReactiveOptimalityCut(double constantItem, const std::vector<double>& beta1_value, const std::vector<double>& beta2_value) {
		IloExpr left(env_);
		for (int n = 0; n < p_.GetTravelArcsSet().size(); n++)
		{
			// r ∈R
			for (int r = 0; r < p_.GetVesselRouteSet().size(); r++)
			{
				for (int w = 0; w < p_.GetVesselPathSet().size(); ++w)
				{
					// r(w) = r
					for (int h = 0; h < p_.GetVesselSet().size(); ++h)
					{
						// vValue[v][r] : come from solution of master problem
						left += (p_.GetArcAndVesselPath()[n][w] * p_.GetShipRouteAndVesselPath()[r][w]
							* p_.GetVesselTypeAndShipRoute()[h][r] * p_.GetVesselCapacity()[h] * beta1_value[n]
							* vVar_[h][r]);

						// vValue[v][r] : come from solution of master problem
						left += (p_.GetArcAndVesselPath()[n][w] * p_.GetShipRouteAndVesselPath()[r][w]
							* p_.GetVesselTypeAndShipRoute()[h][r] * p_.GetVesselCapacity()[h] * beta2_value[n]
							* vVar_[h][r]);
					}
				}
			}
		}
		left -= etaVar_;

		std::string constr_name = "ReactiveOptimalityCut";
		model_.add(IloRange(env_, left, -constantItem, constr_name.c_str()));
	}

	void MasterProblem::addReactiveFeasibilityCut(double constantItem, const std::vector<double>& beta1_value, const std::vector<double>& beta2_value) {
		IloExpr left(env_);
		for (int n = 0; n < p_.GetTravelArcsSet().size(); n++)
		{
			// r ∈R
			for (int r = 0; r < p_.GetVesselRouteSet().size(); r++)
			{
				for (int w = 0; w < p_.GetVesselPathSet().size(); ++w)
				{
					// r(w) = r
					for (int h = 0; h < p_.GetVesselSet().size(); ++h)
					{
						// vValue[v][r] : come from solution of master problem
						left += (p_.GetArcAndVesselPath()[n][w] * p_.GetShipRouteAndVesselPath()[r][w]
							* p_.GetVesselTypeAndShipRoute()[h][r] * p_.GetVesselCapacity()[h] * beta1_value[n]
							* vVar_[h][r]);

						// vValue[v][r] : come from solution of master problem
						left += (p_.GetArcAndVesselPath()[n][w] * p_.GetShipRouteAndVesselPath()[r][w]
							* p_.GetVesselTypeAndShipRoute()[h][r] * p_.GetVesselCapacity()[h] * beta2_value[n]
							* vVar2_[h][r]);
					}
				}
			}
		}

		std::string constr_name = "ReactiveFeasibilityCut";
		model_.add(IloRange(env_, left, -constantItem, constr_name.c_str()));

	}


	void MasterProblem::SolveModel()
	{
		try
		{
			if (Setting::wetherExportModel)
			{
				cplex_.exportModel("Model/MP.lp");
			}

			time_t begin = time(0);

			if (cplex_.solve())
			{
				std::cout<<"MP Solution Status : " << GetSolveStatus() << std::endl;

				time_t end = time(0);
				std::vector<std::vector<int>> vvv(p_.GetVesselSet().size(), std::vector<int>(p_.GetVesselRouteSet().size()));
				for (size_t r = 0; r < p_.GetVesselRouteSet().size(); ++r) {
					for (size_t h = 0; h < p_.GetVesselSet().size(); ++h) {
						vvv[h][r] = (int)(cplex_.getValue(vVar_[h][r]) + 0.5);
					}
				}
				double eta = cplex_.getValue(etaVar_);

				mipGap = cplex_.getMIPRelativeGap();

				SetVVarValue(vvv);
				SetEtaValue(eta);
				SetObjVal(cplex_.getObjValue());
				SetOperationCost(cplex_.getObjValue() - eta);

				// print master problem solution
				if (DebugEnable && MasterEnable)
				{ // Replace with actual condition
					auto solveTime = difftime(end, begin);
					std::cout << "------------------------------------------------------------------------" << std::endl;
					std::cout << "SolveTime = " << solveTime << std::endl;
					PrintSolution(); // Implement this method to print the solution
					std::cout << "------------------------------------------------------------------------" << std::endl;
				}
			}
			else
			{
				std::cout << "MasterProblem No solution";
			}
		}
		catch (IloException ex) {
			std::cerr << "Concert Error: " << ex.getMessage();
		}
	}

	void MasterProblem::SolveReactiveModel()
	{
		try
		{
			time_t begin = time(0);
			if (cplex_.solve())
			{
				//				std::cout<<"MP Solution Status : "+GetSolveStatus());

				time_t end = time(0);
				time_t startTime = time(0);
				std::vector<std::vector<int>> vvv(p_.GetVesselSet().size(), std::vector<int>(p_.GetVesselRouteSet().size()));
				std::vector<std::vector<int>> vvv2(p_.GetVesselSet().size(), std::vector<int>(p_.GetVesselPathSet().size()));
				double eta = cplex_.getValue(etaVar_);

				for (int r = 0; r < p_.GetVesselRouteSet().size(); r++) {
					for (int h = 0; h < p_.GetVesselSet().size(); ++h) {
						vvv[h][r] = (int)(cplex_.getValue(vVar_[h][r]) + 0.5);
					}
				}

				for (int w = 0; w < p_.GetVesselPathSet().size(); ++w) {
					for (int h = 0; h < p_.GetVesselSet().size(); ++h) {
						vvv2[h][w] = (int)(cplex_.getValue(vVar2_[h][w]) + 0.5);
					}
				}

				SetVVarValue(vvv);
				SetVVarValue2(vvv2);
				SetEtaValue(eta);
				SetObjVal(cplex_.getObjValue());
				SetOperationCost(cplex_.getObjValue() - eta);

				mipGap = cplex_.getMIPRelativeGap();


				time_t endTime = time(0);
				// print master problem solution
				if (DebugEnable && MasterEnable)
				{
					std::cout << "------------------------------------------------------------------------" << std::endl;
					std::cout << "SolveTime = " << difftime(endTime, startTime) << std::endl;
					PrintSolution();
					std::cout << "------------------------------------------------------------------------" << std::endl;
				}
			}
			else
			{
				std::cout << "MasterProblem No solution" << std::endl;
			}
			/*cplex_.end();*/
		}
		catch (IloException ex) {
			std::cout << "Concert Error: " << ex.getMessage();
		}
	}


	void MasterProblem::PrintSolution() {
		std::cout << "Master Objective = " << std::fixed << std::setprecision(2) << GetObjVal() << std::endl;
		std::cout << "Mp-OperationCost = " << std::fixed << std::setprecision(2) << GetOperationCost() << std::endl;
		std::cout << "Mp-OtherCost = " << std::fixed << std::setprecision(2) << GetEtaValue() << std::endl;

		std::cout << "Vessel Decision vVar (MP) : ";
		for (int r = 0; r < p_.GetVesselRouteSet().size(); ++r) {
			std::cout << p_.GetVesselRouteSet()[r] << "(";
			for (int h = 0; h < p_.GetVesselSet().size(); ++h) {
				if (vVarValue_[h][r] != 0) {
					std::cout << p_.GetVesselSet()[h] << ")\t";
				}
			}
		}
		std::cout << std::endl; // Add a new line 
	}

	void MasterProblem::PrintReactiveSolution() {
		std::cout << "V[h][w] : ";
		for (int w = 0; w < p_.GetVesselPathSet().size(); ++w)
		{
			for (int h = 0; h < p_.GetVesselSet().size(); ++h)
			{
				if (vVarValue2_[h][w] != 0)
				{
					std::cout << p_.GetVesselPathSet()[w] + "(" + std::to_string(p_.GetVesselSet()[h]) + ")";
				}
			}
		}
		std::cout << std::endl;
	}

}