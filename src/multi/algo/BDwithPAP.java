package multi.algo;

import ilog.concert.IloException;
import ilog.cplex.IloCplex;
import multi.data.InputData;
import multi.data.Parameter;
import multi.data.Scenario;

import java.io.IOException;
import java.util.List;

public class BDwithPAP extends BD {
    public BDwithPAP(InputData in, Parameter p) throws IloException, IOException {
        super();
        this.in = in;
        this.p = p;
        this.tau = p.getTau();
        this.Algo = "BD&PAP";
        this.AlgoID = Algo + "-R"+ in.getShipRouteSet().size()
                + "-T" + p.getTimeHorizon()
                + "-"+ FleetType
                + "-S" + randomSeed
                + "-V" + VesselCapacityRange;
        frame();
    }
    public BDwithPAP(InputData in, Parameter p, int tau) throws IloException, IOException {
        super();
        this.in = in;
        this.p = p;
        this.tau = tau;
        this.Algo = "BD&PAP";
        this.AlgoID = Algo + "-R"+ in.getShipRouteSet().size()
                + "-T" + p.getTimeHorizon()
                + "-"+ FleetType
                + "-S" + randomSeed
                + "-V" + VesselCapacityRange;
        frame();
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

        // change  MaxVarDemand
        // beta = min(k, m/k)=tau

        double[] maxDemandVar = p.getMaximumDemandVariation();
        p.changeMaximunDemandVariation(tau);

        double time0 = System.currentTimeMillis();
        initialModel();

        printIterTitle(fileWriter, System.currentTimeMillis() - time0);
        printIteration(fileWriter, lower[iteration], upper[iteration],0, 0, 0,
                "--", 0, "--", 0 );

//        SubDerModel sdp = new SubDerModel(in, p);
//        SubProblem sp = new SubProblem(in, p);

        // add the initial scene to make the MP feasible
        /*initialize(sce);*/
        /*mp.addScene(sce.get(iteration));*/
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
//            printSolution(mp.getVVarValue());

            dsp.changeObjectiveVCoefficients(mp.getVVarValue());
            double start2 = System.currentTimeMillis();
            dsp.solveModel();
            double end2 = System.currentTimeMillis();

            if(!updateBoundAndMP()){flag = 3;
            }

            iteration++;
            upper[iteration]=upperBound;
            lower[iteration]=lowerBound;

            printIteration(fileWriter, lower[iteration], upper[iteration],
                    end2 - start2, end1 - start1, System.currentTimeMillis() - start0,
                    dsp.getSolveStatusString(), dsp.getObjGap(), mp.getSolveStatusString(), mp.getObjGap());
        }
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

        p.setMaximumDemandVariation(maxDemandVar);
    }

    @Override
    protected void initializeSce(List<Scenario> sce)
    {
        double[] sss  =new double [in.getRequestSet().size()];

        // |v| = tau/I |e| = tau/I
        double v=(double)tau/(double)p.getDemand().length * 1/ (Math.sqrt(p.getDemand().length));
        for(int i=0;i<p.getDemand().length;i++)
        {
            sss[i]=v ;
        }

        sce.add(new Scenario(sss));
    }
}
