package multi.algo;

import ilog.concert.IloException;
import ilog.cplex.IloCplex;
import multi.DefaultSetting;
import multi.data.IntArrayWrapper;
import multi.data.Scenario;
import multi.model.DetermineModel;
import multi.model.DualSubProblem;
import multi.model.MasterProblem;
import multi.model.SubProblem;

import java.io.File;
import java.io.FileWriter;
import java.io.IOException;
import java.util.ArrayList;
import java.util.LinkedHashSet;
import java.util.List;
import java.util.Set;

public class AlgoFrame extends BaseAlgoFrame {
    protected int tau;
    protected String Algo;
    protected String AlgoID;
    protected FileWriter fileWriter;
    protected DualSubProblem dsp;
    protected MasterProblem mp;
    protected AlgoFrame() {
    }
    protected long start;
    private int[][] solution;
    protected Set<IntArrayWrapper> SolutionPool;
    protected Set<Scenario> ScenarioPool;
    protected List<Scenario> sce;
    protected double [] masterObj =new double [maxIterationNum+1];
    protected double [] subObj =new double [maxIterationNum+1];
    private double worstPerformance;
    private double meanPerformance;
    private double worstSecondStageCost;
    private double meanSecondStageCost;
    protected void frame() throws IOException, IloException {
        initialize();

        if(WhetherAddInitializeSce){
            initializeSce(sce);
        }

        double buildModelTime = initialModel();

        printIterTitle(fileWriter, buildModelTime);
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

            // solve the DSP
            dsp.changeObjectiveVCoefficients(mp.getVVarValue());
            double start2 = System.currentTimeMillis();
            dsp.solveModel();
            double end2 = System.currentTimeMillis();

            // update lower upper bound and add cuts to MP
            updateBoundAndMP();

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
    protected void createFileWriter(String fileName) throws IOException {
        File file = new File(RootPath + AlgoLogPath + fileName);
        if (!file.exists()) {
            try {
                file.createNewFile();
            } catch (IOException e) {
                e.printStackTrace();
            }
        }
        fileWriter = new FileWriter(file, true);
        writeSettings(fileWriter);
        fileWriter.write("=====================================================================");
        fileWriter.write("\n");
        in.writeStatus(fileWriter);
    }

    protected void initialize() throws IOException, IloException {
        System.out.println("===============" + Algo + "================");
        createFileWriter(AlgoID + ".txt");

        sce =new ArrayList<>();

        upper[0]=upperBound;
        lower[0]=lowerBound;

        SolutionPool = new LinkedHashSet<>();
        ScenarioPool = new LinkedHashSet<>();

        start = System.currentTimeMillis();
    }

    protected void initializeSce(List<Scenario> sce)   {
        double [] sss =new double [in.getRequestSet().size()];

        // beta = min{k , I/k}
        double beta=(double)tau > p.getDemand().length/(double)tau ?
                p.getDemand().length/(double)tau : (double)tau;

        double v=(double)tau/(double)p.getDemand().length * (1/ (Math.sqrt(p.getDemand().length)));
        for(int i=0;i<p.getDemand().length;i++)
        {
            sss[i]=beta * v;
        }

        sce.add(new Scenario(sss));
    }
    protected double initialModel() throws IloException, IOException {
        double start = System.currentTimeMillis();

        dsp =new DualSubProblem(in, p, tau);
        mp=new MasterProblem(in, p);

        if(WhetherAddInitializeSce){
            mp.addScene(sce.get(0));
        }

        if(WhetherSetInitialSolution){
            DetermineModel dm = new DetermineModel(in, p);
            mp.setInitialSolution(dm.getVVarValue());
        }

        return System.currentTimeMillis() - start;
    }
    protected void calculateMeanPerformance() throws IOException, IloException {
        System.out.println("Calculating Mean Performance ...");
        if(UseHistorySolution)
        {
            if((in.getHistorySolutionSet().get(AlgoID) != (null))) {
                calculateSampleMeanPerformance(p.solutionToVValue(in.getHistorySolutionSet().get(AlgoID)));
            }
        }else {
            calculateSampleMeanPerformance(mp.getVVarValue());
        }

        fileWriter.write("MeanPerformance = " + getMeanPerformance() + "\n");
        fileWriter.write("WorstPerformance = "+ getWorstPerformance() + "\n");
        fileWriter.write("WorstSecondStageCost = " + getWorstSecondStageCost() + "\n");
        fileWriter.write("MeanSecondStageCost = " + getMeanSecondStageCost() + "\n");
        fileWriter.write("AlgoObjVal = "+ getObjVal() + "\n");
        fileWriter.flush();
        System.out.println("MeanPerformance = " + getMeanPerformance());
        System.out.println("WorstPerformance = "+ getWorstPerformance());
        System.out.println("WorstSecondStageCost = " + getWorstSecondStageCost());
        System.out.println("MeanSecondStageCost = " + getMeanSecondStageCost());
        System.out.println("AlgoObjVal = "+ getObjVal());
    }
    public boolean updateBoundAndMP() throws IloException {
        // LB = max{LB , MP.Objective}
        // LB = MP.Objective = MP.OperationCost + Eta
        if(mp.getObjVal()>lowerBound
                && mp.getSolveStatus() == IloCplex.Status.Optimal) {
            setLowerBound(mp.getObjVal());
        }


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

    protected boolean addSolutionPool(int[] solution) {
        if (SolutionPool.contains(new IntArrayWrapper(solution)))
        {
            return false;
        }
        else{
            SolutionPool.add(new IntArrayWrapper(solution));
            return true;
        }
    }
    protected boolean addScenarioPool(Scenario scenario) {
        if (ScenarioPool.contains(scenario))
        {
            return false;
        }
        else{
            ScenarioPool.add(scenario);
            return true;
        }
    }
    protected void setAlgoResult() throws IloException, IOException {
        setSolveTime(System.currentTimeMillis() - start);
        setObj(upperBound);
        setIter(iteration);
        setvValue(mp.getVVarValue());
        setGap(( upperBound - lowerBound)/lowerBound);
        setSolution(mp.getVVarValue());
        writeSolution(mp.getVVarValue(), fileWriter);

        if(WhetherPrintProcess){
            printSolution(vValue);
        }

        if(WhetherCalculateMeanPerformance){
            calculateMeanPerformance();
        }
    }
    protected void end() throws IOException, IloException {

        if(WhetherPrintProcess || WhetherPrintIteration){
            System.out.println(Algo + " Objective = "+ String.format("%.2f",getObjVal()));
            System.out.println(Algo + " SolveTime = "+getSolveTime()+ "ms");
            System.out.println("==================================");
        }

        if(DefaultSetting.WhetherWriteFileLog){
            fileWriter.write(Algo + "Objective = "+ getObjVal() + "\n");
            fileWriter.write(Algo + " SolveTime = "+getSolveTime()+ "ms" + "\n");
            fileWriter.write("==================================");
            fileWriter.close();
        }

        fileWriter.close();

        endModel();
    }

    protected void endModel() throws IloException {
        mp.end();
        dsp.end();
    }
    protected double calculateSampleMeanPerformance(int[][] vValue) throws IloException, IOException {
        String filename = Algo + "-R"+ in.getShipRouteSet().size()
                + "-T" + p.getTimeHorizon()
                + "-"+ FleetType
                + "-Tau" + p.getTau()
                + "-U" + p.getUncertainDegree()
                + "-S" + randomSeed
                + "-SampleTestResult"+ ".txt";
        File file = new File(RootPath + AlgoLogPath + filename);
        if (!file.exists()) {
            try {
                file.createNewFile();
            } catch (IOException e) {
                e.printStackTrace();
            }
        }

        FileWriter filewriter = new FileWriter(file, false);
        filewriter.write("Sample\tOperationCost\t TotalTransCost\t LadenCost\t EmptyCost\t RentalCost\t PenaltyCost\tTotalCost\n");
        filewriter.flush();

        double mp_operation_cost = p.getOperationCost(vValue);

        double[] sample_sub_opera_costs = new double[numSampleScenes];
        double[] sample_laden_costs = new double[numSampleScenes];
        double[] sample_empty_costs = new double[numSampleScenes];
        double[] sample_rental_costs = new double[numSampleScenes];
        double[] sample_penalty_costs = new double[numSampleScenes];

        double sum_sub_opera_costs = 0;
        double worst_total_cost = 0;
        double worst_second_cost = 0;
        SubProblem sp = new SubProblem(in, p, vValue);
        for (int sce = 0; sce < numSampleScenes; sce++) {
            sp.changeDemandConstraintCoefficients(p.getSampleScenes()[sce]);
            sp.solveModel();

            sample_sub_opera_costs[sce] = sp.getTotalCost();
            sample_laden_costs[sce] = sp.getLadenCost();
            sample_empty_costs[sce] = sp.getEmptyCost();
            sample_rental_costs[sce] = sp.getRentalCost();
            sample_penalty_costs[sce] = sp.getPenaltyCost();

            sum_sub_opera_costs += sp.getTotalCost();
            if((mp_operation_cost + sample_sub_opera_costs[sce]) > worst_total_cost){
                worst_total_cost = mp_operation_cost + sample_sub_opera_costs[sce];
                worst_second_cost = sample_sub_opera_costs[sce];
            }

            drawProgressBar((sce) * 100 / numSampleScenes);

            filewriter.write(sce + "\t" + mp_operation_cost + "\t"
                    + sample_sub_opera_costs[sce] + "\t"
                    + sample_laden_costs[sce] + "\t"
                    + sample_empty_costs[sce] + "\t"
                    + sample_rental_costs[sce] + "\t"
                    + sample_penalty_costs[sce] + "\t"
                    + (mp_operation_cost + sample_sub_opera_costs[sce])
                    + "\n");
            filewriter.flush();
        }
        this.setWorstPerformance(worst_total_cost);
        this.setWorstSecondStageCost(worst_second_cost);
        this.setMeanPerformance(mp_operation_cost + sum_sub_opera_costs / numSampleScenes);
        this.setMeanSecondStageCost(sum_sub_opera_costs / numSampleScenes);

        filewriter.close();

        System.out.println();

        return mp_operation_cost + sum_sub_opera_costs/ numSampleScenes;
    }

    public int[][] getSolution() {
        return solution;
    }

    protected void setSolution(int[][] solution) {
        this.solution = solution;
    }
    public double getWorstPerformance() {
        return worstPerformance;
    }
    protected void setWorstPerformance(double worstPerformance) {
        this.worstPerformance = worstPerformance;
    }
    public double getMeanPerformance() {
        return meanPerformance;
    }

    public void setMeanPerformance(double meanPerformance) {
        this.meanPerformance = meanPerformance;
    }
    public double getWorstSecondStageCost() {
        return worstSecondStageCost;
    }

    protected void setWorstSecondStageCost(double worstSecondStageCost) {
        this.worstSecondStageCost = worstSecondStageCost;
    }
    public double getMeanSecondStageCost() {
        return meanSecondStageCost;
    }

    protected void setMeanSecondStageCost(double meanSecondStageCost) {
        this.meanSecondStageCost = meanSecondStageCost;
    }


    protected void printIterTitle(FileWriter fileWriter, double bulidModelTime) throws IOException {
        if(WhetherPrintProcess || WhetherPrintIteration){
            System.out.println("BuildModelTime = "+String.format("%.2f", bulidModelTime));
            System.out.println("k"+"\t\t"
                    +"LB"+"\t\t"
                    +"UB"+"\t\t"
                    +"DSP-SolveTime(s)"+"\t\t"
                    +"MP-SolveTime(s)"+"\t\t"
                    +"Total Time"+"\t\t"
                    +"DSP-Status"+"\t\t"
                    +"MP-Status"
            );
        }
        if(WhetherWriteFileLog){
            fileWriter.write("k"+"\t\t"
                    +"LB"+"\t\t"
                    +"UB"+"\t\t"
                    +"DSP-SolveTime(ms)"+"\t\t"
                    +"MP-SolveTime(ms)"+"\t\t"
                    +"Total Time(ms)"+"\t\t"
                    +"DSP-Status(Gap)"+"\t\t"
                    +"MP-Status(Gap)"
            );
            fileWriter.write("\n");
            fileWriter.flush();
        }
    }
    protected void printIteration(FileWriter fileWriter,
                               double LB, double UB,
                               double dsp_time, double mp_time, double total_time,
                               String dspSolveStatusString, double dspMipGap,
                               String mpSolveStatusString, double mpMipGap) throws IOException {
        if(WhetherPrintProcess || WhetherPrintIteration){
            System.out.println(iteration+"\t\t"
                    +String.format("%.2f", LB)+"\t\t"
                    +String.format("%.2f", UB)+"\t\t"
                    +String.format("%.2f", dsp_time)+"\t\t"
                    +String.format("%.2f", mp_time)+"\t\t"
                    +String.format("%.2f", total_time)+"\t\t"
                    +dspSolveStatusString+"("+String.format("%.4f", dspMipGap)+")"+"\t\t"
                    +mpSolveStatusString+"("+String.format("%.4f", mpMipGap)+")"
            );
        }
        if(WhetherWriteFileLog){
            fileWriter.write(iteration+"\t\t"
                    +String.format("%.2f", LB) +"\t\t"
                    +String.format("%.2f", UB)+"\t\t"
                    +String.format("%.2f", dsp_time)+"\t\t"
                    +String.format("%.2f", mp_time)+"\t\t"
                    +String.format("%.2f", total_time)+"\t\t"
                    +dspSolveStatusString+"("+String.format("%.4f", dspMipGap)+")"+"\t\t"
                    +mpSolveStatusString+"("+String.format("%.4f", mpMipGap)+")"
            );
            fileWriter.write("\n");
            fileWriter.flush();
        }
    }

    protected int[][] vValue;
    protected void setvValue(int[][] vValue) {
        this.vValue = vValue;
    }
    protected int[][] getVValue() {
        return vValue;
    }
    public void printSolution(int[][] vValue) throws IloException {
        System.out.println("Vessel Decision vVar : ");
        for(int r = 0; r<p.getShippingRouteSet().length; r++)
        {
            System.out.print(p.getShippingRouteSet()[r]);
            System.out.print(":");

            if(FleetType.equals("Homo")){
                for(int h=0;h<p.getVesselSet().length;++h)
                {
                    if(vValue[h][r] != 0)
                    {
                        System.out.print(p.getVesselSet()[h]+"\t");
                    }
                }
            } else if (FleetType.equals("Hetero")) {
                for(int w=0;w<p.getVesselPathSet().length;++w)
                {
                    if (p.getShipRouteAndVesselPath()[r][w] != 1)
                    {
                        continue;
                    }
                    for(int h=0;h<p.getVesselSet().length;++h)
                    {
                        if(vValue[h][w] != 0 && p.getShipRouteAndVesselPath()[r][w] == 1)
                        {
                            System.out.print(p.getVesselPathSet()[w]+"(" + p.getVesselSet()[h]+")\t");
                        }
                    }
                }
                System.out.println();
            }
            else{
                System.out.println("Error in Fleet type!");
            }
        }
        System.out.println();
    }
    public void writeSolution(int[][] vValue, FileWriter fileWriter) throws IloException, IOException {
        fileWriter.write("Vessel Decision vVar : " + "\n");
        for(int r = 0; r<p.getShippingRouteSet().length; r++)
        {
            fileWriter.write(p.getShippingRouteSet()[r] + ": ");

            if(FleetType.equals("Homo")){
                for(int h=0;h<p.getVesselSet().length;++h)
                {
                    if(vValue[h][r] != 0)
                    {
                        fileWriter.write(p.getVesselSet()[h]+"\t");
                    }
                }
            } else if (FleetType.equals("Hetero")) {
                for(int w=0;w<p.getVesselPathSet().length;++w)
                {
                    if (p.getShipRouteAndVesselPath()[r][w] != 1)
                    {
                        continue;
                    }
                    for(int h=0;h<p.getVesselSet().length;++h)
                    {
                        if(vValue[h][w] != 0 && p.getShipRouteAndVesselPath()[r][w] == 1)
                        {
                            fileWriter.write(p.getVesselPathSet()[w]+"(" + p.getVesselSet()[h]+")\t");
                        }
                    }
                }
                fileWriter.write("\n");
            }
            else{
                System.out.println("Error in Fleet type!");
            }
        }
        fileWriter.write("\n");
    }

    protected double DiffVesselValue(int[][] old_vVarValue, int[][] new_vVarValue){
        double diff=0;
        for(int h = 0; h<old_vVarValue.length; h++){
            for(int w=0;w<old_vVarValue[0].length;w++){
                diff += Math.abs(new_vVarValue[h][w]-old_vVarValue[h][w]);
            }
        }
        return diff;
    }
    protected int[][] updateVVV(int [][] vvv, int [][] new_vVarValue){
        for(int h = 0; h<new_vVarValue.length; h++){
            for(int w=0;w<new_vVarValue[0].length;w++){
                vvv[h][w] = new_vVarValue[h][w];
            }
        }
        return vvv;
    }
    private double totalCost;
    private double operationCost;
    private double rentalCost;
    private double ladenCost;
    private double emptyCost;
    private double penaltyCost;
    public double getTotalCost() {
        return totalCost;
    }
    protected void setTotalCost(double totalCost) {
        this.totalCost = totalCost;
    }
    public double getOperationCost() {
        return operationCost;
    }
    protected void setOperationCost(double operationCost) {
        this.operationCost = operationCost;
    }
    protected void setLadenDemurrageCost(double ladenDemurrageCost)
    {
        this.ladenCost = ladenDemurrageCost;
    }
    public double getLadenDemurrageCost()
    {
        return ladenCost;
    }
    protected void setEmptyDemurrageCost(double emptyDemurrageCost)
    {
        this.emptyCost = emptyDemurrageCost;
    }
    public double getEmptyDemurrageCost()
    {
        return emptyCost;
    }
    protected void setRentalCost(double rentalCost)
    {
        this.rentalCost = rentalCost;
    }
    public double getRentalCost()
    {
        return rentalCost;
    }
    protected void setPenaltyCost(double penaltyDemurrageCost)
    {
        this.penaltyCost = penaltyDemurrageCost;
    }
    public double getPenaltyCost()
    {
        return penaltyCost;
    }
}
