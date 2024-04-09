package multi.algo;

import ilog.concert.IloException;
import ilog.cplex.IloCplex;
import multi.data.InputData;
import multi.data.Parameter;

import java.io.IOException;

public class BD extends AlgoFrame {
	public BD(InputData in, Parameter p) throws IloException, IOException {
		super();
		this.in = in;
		this.p = p;
		this.tau = p.getTau();
		this.Algo = "BD";
		this.AlgoID = Algo + "-R"+ in.getShipRouteSet().size()
				+ "-T" + p.getTimeHorizon()
				+ "-"+ FleetType
				+ "-S" + randomSeed
				+ "-V" + vesselCapacityRange;
		frame();
	}
	public BD(InputData in, Parameter p, int tau) throws IloException, IOException {
		super();
		this.in = in;
		this.p = p;
		this.tau = tau;
		this.Algo = "BD";
		this.AlgoID = Algo + "-R"+ in.getShipRouteSet().size()
				+ "-T" + p.getTimeHorizon()
				+ "-"+ FleetType
				+ "-S" + randomSeed
				+ "-V" + vesselCapacityRange;
		frame();
	}
	public BD() {
	}

	@Override
	protected void frame() throws IOException, IloException {
		initialize();

		if(WhetherAddInitializeSce){
			initializeSce(sce);
		}

		double time0 = System.currentTimeMillis();
		initialModel();

		printIterTitle(fileWriter, System.currentTimeMillis() - time0);
		printIteration(fileWriter, lower[iteration], upper[iteration],0, 0, 0,
				"--", 0, "--", 0 );

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
				flag=1;
			}

			// LB = max{LB , MP.Objective}
			// LB = MP.Objective = MP.OperationCost + Eta
			if(mp.getObjVal()>lowerBound
					&& mp.getSolveStatus() == IloCplex.Status.Optimal) {
				setLowerBound(mp.getObjVal());
			}

			dsp.changeObjectiveVCoefficients(mp.getVVarValue());
			double start2 = System.currentTimeMillis();
			dsp.solveModel();
			double end2 = System.currentTimeMillis();

			if(!updateBoundAndMP()){
				flag = 3;
			}

			iteration++;
			upper[iteration]=upperBound;
			lower[iteration]=lowerBound;

			printIteration(fileWriter, lower[iteration], upper[iteration],
					end2 - start2, end1 - start1, System.currentTimeMillis() - start0,
					dsp.getSolveStatusString(), dsp.getObjGap(),
					mp.getSolveStatusString(), mp.getObjGap());
		}

		// end the loop
		if(flag == 1){
			System.out.println("MP solution duplicate");
		}
		else if(flag == 2){
			System.out.println("Worse case duplicate");
		}
		else if(flag == 3){
			System.out.println("DSP solution infeasible");
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
			mp.getCplex().add(dsp.constructOptimalCut(mp.getCplex(), mp.getVVars(), mp.getEtaVar()));
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
