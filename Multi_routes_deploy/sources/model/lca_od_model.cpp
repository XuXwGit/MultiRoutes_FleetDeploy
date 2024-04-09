// LCA : liner container assignement
// aims : which part of TTS demand to fufill	and how to allocate the
// container paths
#include "lca_od_model.h"

#include <ilcplex/ilocplex.h>

#include <fstream>
#include <string>
#include <vector>
ILOSTLBEGIN

// #include "input_data.h"
#include "Arc.h"
#include "OD.h"
#include "Port.h"
#include "input_data.h"
#include "ship_route.h"
#include "space_time_network.h"

using namespace std;
using namespace fleetdeployment;

void lca_od_model() {
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
  // OD-link-based linear programming formulatation

  IloEnv env;
  try {
    IloModel ODmodel(env);
    // define the decision variables
    // y(u, v)         : the volume of containers between new OD pairs (u, v) ¡Ê
    // Wod f(u, v, m, n) : the volume of containers between new OD pairs (u, v)
    // ¡Ê Wod that flow on the arc (m , n) ¡Ê A = Av ¡È At
    IloArray<IloNumVarArray> y(env, W.size());
    IloArray<IloArray<IloNumVarArray>> f(env, W.size());
    for (int u = 0; u < Wod.size(); u++) {
      y[u] =
          IloNumVarArray(env, Wod[u].size(), 0, IloInfinity, IloNumVar::Float);
      f[u] = IloArray<IloNumVarArray>(env, Wod[u].size());
      for (int v = 0; v < Wod[u].size(); v++) {
        // y[u][v].setName("y[" + u + ']' + '[' + v + ']');
        f[u][v] =
            IloNumVarArray(env, A.size(), 0, IloInfinity, IloNumVar::Float);
      }
    }

    // add the objective
    IloExpr totalRevenue(env), totalCost(env);
    for (int u = 0; u < (int)W.size(); u++) {
      for (int v = 0; v < (int)Wod[u].size(); v++) {
        totalRevenue += W[u].GetMaxProfitRate() * y[u][v];

        for (int mn = 0; mn < (int)A.size(); mn++) {
          totalCost += A[mn].GetCost() * f[u][v][mn];
        }
      }
    }
    IloObjective objective = IloMaximize(env, totalRevenue - totalCost);
    ODmodel.add(objective);

    // add the constraints
    // flow conservation equation
    for (int m = 0; m < N.size(); m++) {
      for (int od = 0; od < W.size(); od++) {
        for (int uv = 0; uv < Wod[od].size(); uv++) {
          IloExpr sumF(env), outFlow(env), inFlow(env);
          for (int n = 0; n < A.size(); n++) {
            if (A[n].GetTail() == N[m]) {
              outFlow += f[od][uv][n];
            }
            if (A[n].GetHead() == N[m]) {
              inFlow += f[od][uv][n];
            }
          }
          sumF = outFlow - inFlow;
          // m = u
          if (N[m] == Wod[od][uv].first) {
            ODmodel.add(IloRange(sumF - y[od][uv] == 0));
          }
          // m = v
          else if (N[m] == Wod[od][uv].second) {
            ODmodel.add(IloRange(sumF + y[od][uv] == 0));
          }
          // otherwise
          else {
            ODmodel.add(IloRange(sumF == 0));
          }
        }
      }
    }

    // capacity constraint
    for (int r = 0; r < R.size(); r++) {
      for (int i = 0; i < R[r].GetNumPort(); i++) {
        IloExpr sumf(env);
        for (int od = 0; od < W.size(); od++) {
          for (int uv = 0; uv < Wod[od].size(); uv++) {
            for (int m = 0; m < Av.size(); m++) {
              if (Av[m].GetTail().GetRouteID() == R[r].GetID() &&
                  Av[m].GetTail().GetCall() == R[r].GetPath()[i].ID) {
                int mn = ST.GetAIndex(Av[m]);
                sumf += f[od][uv][mn];
              }
            }
          }
        }
        ODmodel.add(IloRange(sumf <= R[r].GetCapacity()));
      }
    }

    // demand capacity
    for (int od = 0; od < W.size(); od++) {
      IloExpr sumy(env);
      for (int j = 0; j < Wod[od].size(); j++) {
        sumy += y[od][j];
      }
      ODmodel.add(IloRange(sumy <= W[od].GetDemand()));
    }

    IloCplex cplex(ODmodel);

    cplex.exportModel("od_model.lp");

    if (!cplex.solve()) {
      cout << "failed to solve this problem!";
      throw(-1);
    } else {
      cout << "The solution Status: " << cplex.getStatus() << endl;
      cout << "The objective value: " << cplex.getObjValue() << endl;
      IloNumArray var(env);
      // cplex.getValues(y, var);
      cout << "The finally solution: " << var << endl;
    }
  } catch (const IloException& e) {
    cout << "concert exception caught:" << e;
  } catch (...) {
    cout << "unknown exception caught";
  }

  env.end();
}
