// LCA : liner container assignement
// aims : which part of TTS demand to fufill	and how to allocate the container paths

#include "lca_origin_model.h"

#include <ilcplex/ilocplex.h>
ILOSTLBEGIN

#include <vector>
#include <fstream>
#include <string>

#include "input_data.h"
#include "Arc.h"
#include "OD.h"
#include "Port.h"
#include "input_data.h"
#include "ship_route.h"
#include "space_time_network.h"

using namespace std;
using namespace fleetdeployment;

void lca_origin_model()
{
	// Input data of liner shipping network (R, P, W)
	// R : the set of ship routes / ship rotations
	// P : the set of ports
	// W: the set of O-D pairs
  map<string, Port> P;
  vector<ship_route> R;
  vector<fleetdeployment::OD> W;
  string path = "/Data";
  if (!inputData(path, P, R, W)) {
    cout << "can't input data";
  }

	// Generate Space-time Network G = (N, A)
	// N : the set of Nodes
	// A : the set of arcs (voyage arcs and transshipment arcs)
	// Wod : the set of new OD pairs W_od
  ST_network ST(R, P, W);
  vector<vector<pair<Node, Node>>> Wod = ST.GetWod();
  vector<Node> N = ST.GetN();
  vector<Node> N1 = ST.GetN1();
  vector<Arc> A = ST.GetA();
  vector<Arc> Av = ST.GetAv();
  vector<Arc> At = ST.GetAt();

	// Formulate LCA model
	// Origin-link-based linear programming formulation

	IloEnv env;
	try
	{
		IloModel ODmodel(env);
		// define the decision variables
		// y(u, v)		: the volume of containers from the origin ports u （ N1 to destination ports v （ N
		// f(u, v)		: the volume of containers from the origin ports u （ N1 that flow on the arc (m , n) （  A  =  Av “ At

		IloModel originModel(env);

		// decision variable: y！|N1|*|N|
		// decision variable: f！|N1|*|A|
		IloArray<IloNumVarArray> y(env, N1.size());
		IloArray<IloNumVarArray> f(env, N1.size());
		for (IloInt u = 0; u < (IloInt)N1.size(); u++)
		{
			y[u] = IloNumVarArray(env, N.size(), 0, IloInfinity, IloNumVar::Float);
			f[u] = IloNumVarArray(env, A.size(), 0, IloInfinity, IloNumVar::Float);

			// add the constraint about y[u][v]
			for (IloInt v = 0; v < (IloInt)N.size(); v++)
			{
				for (int od = 0; od < W.size(); od++)
				{
					int r1 = N1[u].GetRouteID();
					int i1 = N1[u].GetCall();
					int r2 = N[v].GetRouteID();
					int i2 = N[v].GetCall();
					string s1 = R[r1 - 1].GetPort(i1);
					string s2 = R[r2 - 1].GetPort(i2);
					if (W[od].GetOrigin() == s1 && W[od].GetDestination() == s2)
					{
						if (find(Wod[od].begin(), Wod[od].end(), pair<Node, Node>(N1[u], N[v])) == Wod[od].end())
						{
							originModel.add(IloRange(y[u][v] == 0));
						}
					}
				}
			}
		}
		// the objective function
		IloExpr totalProfit2(env), sumRevenue2(env), sumCost2(env);
		for (int u = 0; u < N1.size(); u++)
		{
			for (int v = 0; v < N.size(); v++)
			{
				int flag = 0;
				int od = 0;
				for (; od < W.size(); od++)
				{
					int r1 = N1[u].GetRouteID();
					int i1 = N1[u].GetCall();
					int r2 = N[v].GetRouteID();
					int i2 = N[v].GetCall();
					if (W[od].GetOrigin() == R[r1 - 1].GetPort(i1) && W[od].GetDestination() == R[r2 - 1].GetPort(i2))
					{
						if (find(Wod[od].begin(), Wod[od].end(), pair<Node, Node>(N1[u], N[v])) != Wod[od].end())
						{
							flag = 1;
							break;
						}
					}
				}
				if (flag)
				{
					sumRevenue2 += W[od].GetMaxProfitRate() * y[u][v];
				}
			}
		}
		for (int u = 0; u < N1.size(); u++)
		{
			for (int mn = 0; mn < A.size(); mn++)
			{
				if (find(At.begin(), At.end(), A[mn]) != At.end())
					sumCost2 += A[mn].GetCost() * f[u][mn];
			}
		}
		totalProfit2 = sumRevenue2 - sumCost2;
		IloObjective objective2 = IloMaximize(env, totalProfit2);
		originModel.add(objective2);

		// flow conservation equation
		for (int u = 0; u < N1.size(); u++)
		{
			IloExpr sumf(env), outFlow2(env) , inFlow2(env);
			for (int m = 0; m < N.size(); m++)
			{
				for (int n = 0; n < A.size(); n++)
				{
					if (N[m] == A[n].GetTail())
					{
						outFlow2 += f[u][n];
					}
					if (N[m] == A[n].GetHead())
					{
						inFlow2 += f[u][n];
					}
				}
				sumf = outFlow2 - inFlow2;
				// m == u
				if (N1[u] == N[m])
				{
					IloExpr sumy(env);
					for (int v = 0; v < N.size(); v++)
					{
						sumy += y[u][v];
					}
					originModel.add(sumf - sumy == 0);
				}
				// m != u
				else
				{
					originModel.add(sumf + y[u][m] == 0);
				}
			}
		}

		// capacity constraint
		for (int r = 0; r < R.size(); r++)
		{
			for (int i = 0; i < R[r].GetPortNum(); i++)
			{
				IloExpr sumf(env);
				for (int u = 0; u < N1.size(); u++)
				{
						for (int m = 0; m < Av.size(); m++)
						{
                                                if (Av[m].GetTail()
                                                            .GetRouteID() ==
                                                        R[r].GetID() &&
                                                    Av[m].GetTail().GetCall() ==
                                                        R[r].GetPath()[i].ID)
							{
								int mn = ST.GetAIndex(Av[m]);
								sumf += f[u][mn];
							}
						}
				}
				ODmodel.add(IloRange(sumf <= R[r].GetCapacity()));
			}
		}

		// demand constraint
		for (int od = 0; od < W.size(); od++)
		{
			IloExpr sumy(env);
			for (int j = 0; j < Wod[od].size(); j++)
			{
				int u = ST.GetN1Index(Wod[od][j].first);
				int v = ST.GetNodeIndex(Wod[od][j].second);
				if (u > 0 && v > 0)
				{
					sumy += y[u][v];
				}

				//for (int u = 0; u < N1.size(); u++)
				//{
				//	if (N1[u] == Wod[od][j].first)
				//	{
				//		for (int v = 0; v < N.size(); v++)
				//		{
				//			if (N[v] == Wod[od][j].second)
				//			{
				//				sumy += y[u][v];
				//			}
				//		}
				//	}
				//}
			}
			originModel.add(IloRange(sumy <= W[od].GetDemand()));
		}

		IloCplex cplex2(originModel);

		cplex2.exportModel("origin_model.lp");

		if (!cplex2.solve())
		{
			cout << "failed to solve this problem!";
			throw(-1);
		}
		else
		{
			cout << "The solution Status: " << cplex2.getStatus() << endl;
			cout << "The objective value: " << cplex2.getObjValue() << endl;
			IloNumArray var(env);
			//cplex.GetValues(y, var);
			cout << "The finally solution: " << var << endl;
		}
	}
	catch (const IloException& e)
	{
		cout << "concert exception caught:" << e;
	}
	catch (...)
	{
		cout << "unknown exception caught";
	}

	env.end();
}
