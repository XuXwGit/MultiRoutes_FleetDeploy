package multi.algo;

import ilog.concert.IloException;
import ilog.concert.IloIntVar;
import ilog.concert.IloNumVar;
import ilog.concert.IloRange;
import ilog.cplex.IloCplex;
import multi.DefaultSetting;
import multi.data.IntArrayWrapper;
import multi.model.DualSubProblem;

import java.util.Set;

// The class BendersLazyConsCallback
// Allows to add Benders' cuts as lazy constraints.
public class BendersLazyConsCallback
        extends IloCplex.LazyConstraintCallback {
    IloCplex cplex;
    IloIntVar[][] V;
    IloNumVar eta;
    DualSubProblem dsp;
    Set<IntArrayWrapper> solutionPool;
    int numVessels;
    int numRoutes;
    int numSolutions;
    public BendersLazyConsCallback(IloCplex cplex, IloIntVar[][] V, IloNumVar eta, DualSubProblem dsp, Set<IntArrayWrapper> solutionPool) throws IloException {
        this.cplex = cplex;
        this.V = V;
        this.eta = eta;
        this.dsp = dsp;
        this.numVessels = V.length;
        this.numRoutes = V[0].length;
        this.solutionPool = solutionPool;
        if (solutionPool != null)
            this.numSolutions = solutionPool.size();
    }

    private void frame2() throws IloException {
        // Get the current x solution
        double[][] sol = new double[numVessels][numRoutes];
        int[] solution = new int[numRoutes];
        for (int r = 0; r < numRoutes; r++)
        {
            for (int h = 0; h < numVessels; ++h)
            {
                sol[h][r] = getValue(V[h][r]);
                if (sol[h][r] == 1)
                {
                    solution[r] = h + 1;
                }
            }
        }

        if (solutionPool.contains(new IntArrayWrapper(solution)))
        {
            return;
        }
        else{
            solutionPool.add(new IntArrayWrapper(solution));
            numSolutions += 1;
        }

        // Benders' cut separation
        IloRange cut = dsp.separate(sol, cplex,  V, eta);

        System.out.print("Add cut: " + solutionPool.size() + "\t");
        printSolution(sol);

        if ( cut != null)
            add(cut, IloCplex.CutManagement.UseCutForce);
    }

    private void frame3() throws IloException {
        // Get the current x solution
        int numVesselPaths = V[0].length;
        double[][] sol = new double[numVessels][numVesselPaths];
        int[] solution = new int[numVesselPaths];
        for (int w = 0; w < numVesselPaths; w++)
        {
            for (int h = 0; h < numVessels; ++h)
            {
                sol[h][w] = getValue(V[h][w]);
                if (sol[h][w] == 1)
                {
                    solution[w] = h + 1;
                }
            }
        }

        if (solutionPool.contains(new IntArrayWrapper(solution)))
        {
            return;
        }
        else{
            solutionPool.add(new IntArrayWrapper(solution));
            numSolutions += 1;
        }

        // Benders' cut separation
        IloRange cut = dsp.separate(sol, cplex,  V, eta);

        System.out.print("Add cut: " + solutionPool.size() + "\t");
        printSolution(sol);

        if ( cut != null)
            add(cut, IloCplex.CutManagement.UseCutForce);
    }

    @Override
    protected void main() throws IloException {
        if (V != null) {
            if(DefaultSetting.FleetType == "Homo")
                frame2();
            else if (DefaultSetting.FleetType == "Hetero")
                frame3();
            else{
                System.out.println("Error in Get Fleet Type");
            }
        }
    }

    public void printSolution(double[][] vVarValue) {
        for (int r = 0; r < numRoutes; r++) {
            System.out.print(r + 1);
            System.out.print(":");

            for (int h = 0; h < numVessels; ++h) {
                if(vVarValue[h][r] > 0.5){
                    System.out.print("(" + (h + 1)+ ")" + "\t");
                }
            }
        }
        System.out.println();
    }

} // END BendersLazyConsCallback