package multi.model;

import ilog.concert.IloException;
import ilog.concert.IloLinearNumExpr;
import ilog.cplex.IloCplex;
import multi.data.InputData;
import multi.data.Parameter;
// import multi.structure.Request;
// import multi.structure.TravelingArc;

import java.io.IOException;
import java.util.Arrays;

public class DetermineModel extends BasePrimalModel {
	private final String modelType;

	public DetermineModel(InputData in, Parameter p) {
		super();
		this.in = in;
		this.p = p;
		this.Model = "DM";
		this.ModelName = Model + "-R"+ in.getShipRouteSet().size()
				+ "-T" + p.getTimeHorizon()
				+ "-"+ FleetType
				+ "-S" + randomSeed
				+ "-V" + vesselCapacityRange;
		this.modelType = "UseMeanValue";
		try{
			if(WhetherPrintProcess || WhetherPrintIteration){
				System.out.println("=========DetermineModel==========");
			}

			cplex = new IloCplex();
			publicSetting(cplex);

			double start = System.currentTimeMillis();
			frame();
			double end = System.currentTimeMillis();
			System.out.println("BuildTime = "+ ( end - start));
			start = System.currentTimeMillis();
			solveModel();
			end = System.currentTimeMillis();
			System.out.println("SolveTime = "+ ( end - start));

			if(WhetherCalculateMeanPerformance){
				calculateMeanPerformance();
			}
		}catch (IloException e) {
			e.printStackTrace();
		} catch (IOException e) {
			throw new RuntimeException(e);
		}
	}

	public DetermineModel(InputData in, Parameter p, String modelType) {
		super();
		this.in = in;
		this.p = p;
		this.Model = "DM";
		this.ModelName = Model + "-R"
				+ in.getShipRouteSet().size()
				+ "-T" + p.getTimeHorizon()
				+ "-"+ FleetType
				+ "-S" + randomSeed
				+ "-V" + vesselCapacityRange + "-" + modelType;
		this.modelType = modelType;
		try{
			if(WhetherPrintProcess || WhetherPrintIteration){
				System.out.println("=========DetermineModel==========");
			}
			cplex = new IloCplex();
			publicSetting(cplex);

			frame();

			solveModel();
		}catch (IloException e) {
			e.printStackTrace();
		}
	}

	@Override
	protected void setDecisionVars() throws IloException {
		// v[h][r/w] : binary variable ���� whether vessel type h is assigned to shipping route r/w
		SetVesselDecisionVars();

		// x[i][k]
		// y[i][k]
		// z[i][k]
		// g[i]
		SetRequestDecisionVars();
	}

	@Override
	protected void setObjectives() throws IloException {
		IloLinearNumExpr Obj = cplex.linearNumExpr();
		Obj = GetVesselOperationCostObj(Obj);
		Obj = GetRequestTransCostObj(Obj);

		cplex.addMinimize(Obj);
	}

	@Override
	protected void setConstraints() throws IloException
	{
		// each ship route assigned to one vessel
		long start = System.currentTimeMillis();
		setConstraint1();
		//System.out.println("Set Constraint1 Time = "+ (System.currentTimeMillis() - start));
		start = System.currentTimeMillis();

		// Demand Equation Constraints
		setConstraint2();
		//System.out.println("Set Constraint2 Time = "+ (System.currentTimeMillis() - start));
		start = System.currentTimeMillis();

		// Transport Capacity Constraints
		setConstraint3();
		//System.out.println("Set Constraint3 Time = "+ (System.currentTimeMillis() - start));
		start = System.currentTimeMillis();

		// Containers Flow Conservation Constraints
		setConstraint4();
		//System.out.println("Set Constraint4 Time = "+ (System.currentTimeMillis() - start));
	}

	// (2)
	// Each Route should be assigned only one Vessel
	private void setConstraint1() throws IloException
	{
		setVesselConstraint();
	}

	// (3)
	// Demand Equation Constraints
	private void setConstraint2() throws IloException
	{
		double [] uValueDouble = new double[p.getDemand().length];

		if(modelType.equals("UseMaxValue")) {
			Arrays.fill(uValueDouble, 1.0);
		}

		setDemandConstraint(xVar, yVar, gVar, uValueDouble);
	}

	// (4)
	// Capacity Constraint on each travel arc
	private void setConstraint3() throws IloException
	{
		setCapacityConstraint();
	}

	// (29)
	// Containers flow conservation
	// Containers of each port p at each time t
	private void setConstraint4() throws IloException
	{
		setEmptyConservationConstraint();
	}

	public void solveModel()
	{
		try
		{
			if (WhetherExportModel)
				exportModel();
			long startTime = System.currentTimeMillis();
			if (cplex.solve())
			{
				long endTime = System.currentTimeMillis();

				setVVarsSolution();

				setObjVal(cplex.getObjValue());
				setSolveTime(endTime - startTime);
				setObjGap(cplex.getMIPRelativeGap());

				System.out.println("SolveTime = " + getSolveTime());

				if(WhetherPrintProcess){
					printSolution();
				}

			}
			else
			{
				System.out.println("No solution");
			}
			cplex.end();
		}
		catch (IloException ex) {
			System.out.println("Concert Error: " + ex);
		}
	}
}
