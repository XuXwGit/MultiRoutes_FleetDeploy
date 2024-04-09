package multi.algo;

import ilog.concert.IloException;
import ilog.cplex.IloCplex;
import multi.data.InputData;
import multi.data.Parameter;
import multi.model.DetermineModel;
import multi.model.DualProblem;
import multi.model.DualSubProblem;
import multi.model.MasterProblem;

import java.io.IOException;

public class BDwithPareto extends BD {
	public BDwithPareto(InputData in, Parameter p) throws IloException, IOException {
		super();
		this.in = in;
		this.p = p;
		this.tau = p.getTau();
		this.Algo = "BD&Pareto";
		this.AlgoID = Algo + "-R"+ in.getShipRouteSet().size() + "-T" + p.getTimeHorizon() + "-"+ FleetType + "-S" + randomSeed + "-V" + VesselCapacityRange;
		frame();
	}
	public BDwithPareto(InputData in, Parameter p, int tau) throws IloException, IOException {
		super();
		this.in = in;
		this.p = p;
		this.tau = tau;
		this.Algo = "BD&Pareto";
		this.AlgoID = Algo + "-R"+ in.getShipRouteSet().size() + "-T" + p.getTimeHorizon() + "-"+ FleetType + "-S" + randomSeed + "-V" + VesselCapacityRange;
		frame();
	}

	private double[][] corePoint;
	private DualProblem dp;
	@Override
	protected double initialModel() throws IloException, IOException {
		dsp =new DualSubProblem(in, p, tau);
		dp = new DualProblem(in, p);
		mp=new MasterProblem(in, p);

		dp.changeObjectiveVCoefficients(corePoint);

		if(WhetherAddInitializeSce){
			mp.addScene(sce.get(0));
		}

		if(WhetherSetInitialSolution){
			DetermineModel dm = new DetermineModel(in, p);
			mp.setInitialSolution(dm.getVVarValue());
		}
		return 0;
	}
	@Override
	protected void frame() throws IloException, IOException {
		initialize();
		if(WhetherCalculateMeanPerformance && UseHistorySolution){
			calculateMeanPerformance();
			return;
		}

		if(WhetherAddInitializeSce)
			initializeSce(sce);

		if(FleetType.equals("Homo")){
			corePoint = new double[p.getVesselSet().length][p.getShippingRouteSet().length];
			if (UseParetoOptimalCut){
				for (int h = 0; h < p.getVesselSet().length; h++) {
					for (int r = 0; r < p.getShippingRouteSet().length; r++) {
						corePoint[h][r] = (double)1 / ( p.getShippingRouteVesselNum()[r]) * p.getVesselTypeAndShipRoute()[h][r];
					}
				}
			}
		} else if (FleetType.equals("Hetero")) {
			corePoint = new double [p.getVesselSet().length][p.getVesselPathSet().length];
			if (UseParetoOptimalCut){
				for (int h = 0; h < p.getVesselSet().length; h++) {
					for (int w = 0; w < p.getVesselPathSet().length; w++) {
						int r = in.getVesselPathSet().get(w).getRouteID() - 1;
						corePoint[h][w] = (double)1 /( p.getShippingRouteVesselNum()[r]) * p.getVesselTypeAndShipRoute()[h][r];
					}
				}
			}

		}
		else{
			System.out.println("Error in Fleet type!");
		}

		double time0 = System.currentTimeMillis();
		initialModel();

		printIterTitle(fileWriter, System.currentTimeMillis() - time0);
		printIteration(fileWriter, lower[0], upper[0],
				0, 0, 0,
				"--", 0, "--", 0);

		// check core point
		if(!mp.checkVesselConstraint(corePoint)){
			System.out.println("Core point is not feasible");
			System.exit(0);
		}

		// add the initial scene to make the MP feasible
		/*initializeSce(sce);
		mp.addScene(sce.get(iteration));*/
		int flag = 0;
		double start0 = System.currentTimeMillis();
		while(upperBound - lowerBound > boundGapLimit
				&& flag == 0
				&& iteration<maxIterationNum
				&& (System.currentTimeMillis() - start0)/1000 < maxIterationTime
		)
		{
			double start1 = System.currentTimeMillis();
			mp.solveModel();
			double end1 = System.currentTimeMillis();

			// check if the mp-solution changed
			if(! addSolutionPool(mp.getSolution())){
				System.out.println("MP solution duplicate");
				flag=1;
			}

			// LB = max{LB , MP.Objective}
			// LB = MP.Objective = MP.OperationCost + Eta
			if(mp.getObjVal()>lowerBound
					&& mp.getSolveStatus() == IloCplex.Status.Optimal) {
				setLowerBound(mp.getObjVal());
			}

			double start2 = System.currentTimeMillis();
			dsp.changeObjectiveVCoefficients(mp.getVVarValue());
			dsp.solveModel();
			double end20 = System.currentTimeMillis();
			//System.out.println("Time to solve DSP: " + (end20 - start2) + "ms");
			dp.changeObjectiveUCoefficients(dsp.getUValue());
			dp.changeParetoConstr(mp.getVVarValue(), dsp.getUValue(), dsp.getObjVal());
			dp.solveModel();
			double end2 = System.currentTimeMillis();

			if(!updateBoundAndMP()){
				System.out.println("DSP not optimal");
				flag = 2;
			}

			System.out.println("Time to solve DSP-Pareto: " + (end2 - end20) + "ms");
			System.out.println("Primal Bound: " + dsp.getObjVal());
			System.out.println("Pareto Bound: " + dp.getObjVal());

			iteration++;
			upper[iteration]=upperBound;
			lower[iteration]=lowerBound;

			printIteration(fileWriter, lower[iteration], upper[iteration],
					end2 - start2, end1 - start1, System.currentTimeMillis() - start0,
					dsp.getSolveStatusString(), dsp.getObjGap(), mp.getSolveStatusString(), mp.getObjGap());
		}

		setAlgoResult();
		end();
	}

	@Override
	public boolean updateBoundAndMP() throws IloException {
		//  the SP is optimal :  add optimality cut
		if(dsp.getSolveStatus() == IloCplex.Status.Optimal)
		{
			//System.out.println("DSP is Optimal");
			//  update UB : UB = min{UB, MP.OperationCost + SP.Objective}
			if(dsp.getObjVal()+mp.getOperationCost()<upperBound)
			{
				setUpperBound(dsp.getObjVal()+mp.getOperationCost());
			}
			// add optimality cut
			//mp.addOptimalityCut(dsp.getConstantItem(), dsp.getBetaValue());
			mp.getCplex().add(dp.constructOptimalCut(mp.getCplex(), mp.getVVars(), mp.getEtaVar()));
			// add the worst scene (extreme point) to scene set
			if(! addScenarioPool(dsp.getScene())){
				sce.add(dsp.getScene());
			}
		}
		else if(dsp.getSolveStatus() == IloCplex.Status.Feasible){
			System.out.println("DSP is Feasible");
			return false;
		}
		// the SP is unbounded : add feasibility cut
		else if(dsp.getSolveStatus() == IloCplex.Status.Unbounded)
		{
			System.out.println("DSP is Unbounded");
			// ! here beta is extreme ray !
			mp.addFeasibilityCut(dsp.getConstantItem(), dsp.getBetaValue());
		}
		else if(dsp.getSolveStatus() == IloCplex.Status.Infeasible)
		{
			System.out.println("DSP is InFeasible");
		}

		else if(dsp.getSolveStatus() == IloCplex.Status.Bounded)
		{
			System.out.println("DSP is Bounded");
		}
		else {
			System.out.println("DSP is error");
		}
		return true;
	}

}
