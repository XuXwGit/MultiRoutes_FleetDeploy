package multi.section;

import ilog.concert.IloException;
import multi.DefaultSetting;
import multi.algo.*;
import multi.data.GenerateParameter;
import multi.data.InputData;
import multi.data.Parameter;
import multi.data.ReadData;
import multi.model.DetermineModel;
import multi.model.DetermineModelReactive;
import multi.strategy.SelectPaths;

import java.io.File;
import java.io.FileWriter;
import java.io.IOException;

public class PerformanceExperiment extends DefaultSetting {
    private static FileWriter fileWriter;
    private static int[] timeHorizonSet;
    private static double uncertainDegree = defaultUncertainDegree;
    private static int defaultTimeHorizon;
    public PerformanceExperiment(int instance, int type) throws IloException, IOException {

        File file = new File(RootPath +  TestResultPath
                + "Performance" + instance
                + "-" + type
                + "-" + randomSeed+ ".txt");
        if (!file.exists()) {
            try {
                file.createNewFile();
            } catch (IOException e) {
                e.printStackTrace();
            }
        }
        fileWriter = new FileWriter(file, true);

        // instance:
        // 1-> data1 : R=2,P=10,V=10 /small scale with 2 ship routes
        // 2-> data2 : R=8,P=29,V=40 /large scale with 8 ship routes
        // 3-> data3 : R=3,P=21,V=35 /middle scale with 3 ship routes
        String fileName = DataPath;
        if(instance == 1){
            fileName += "data1/";
            defaultTimeHorizon = 70;
            timeHorizonSet = new int[]{56, 63, 70, 77, 84, 91};
        } else if (instance == 2) {
            fileName += "data2/";
            defaultTimeHorizon = 90;
            timeHorizonSet = new int[]{90};
            //timeHorizonSet = new int[]{60, 75, 90, 105, 120, 135};
        }
        else if (instance == 3) {
            fileName += "data3";
            //timeHorizonSet = new int[]{180, 165, 150, 135, 120, 105, 90};
            timeHorizonSet = new int[]{90, 105, 120, 135, 150, 165, 180};
        }

        //print_strategy_status(fileName);

        // experiment type
        // 1: compare the performance of the four algorithms (CCG/BD/CCG&PAP/BD&PAP)
        // 2: compare the empty container reposition strategy with reactive strategy
        // 3: compare different laden/empty paths select strategy
        // 4: compare different fleet composition: Homo V\S Hetero
        // 5: compare different vessel capacity range
        // 6: compare different demand distribution
        // 7: compare mean-performance and worst-case performance
        // 8: compare two-stage robust with two-stage stochastic programming
        System.out.print("Experiment " + type + ": ");
        if(type == 1){
            System.out.println("Compare the performance of the four algorithms (CCG/BD/CCG&PAP/BD&PAP)");
            experiment_test1(fileName);
        } else if (type == 2) {
            System.out.println("Compare the empty container reposition strategy with reactive strategy");
            experiment_test2(fileName);
        }else if (type == 3){
            System.out.println("Compare performance under different laden/empty paths select strategy");
            experiment_test3(fileName);
        }else if (type == 4){
            System.out.println("Compare performance under different fleet composition: Homo V/S Hetero");
            experiment_test4(fileName);
        }else if (type == 5){
            System.out.println("Compare performance under different vessel capacity range");
            experiment_test5(fileName);
        }else if (type == 6){
            System.out.println("Compare performance under different demand distribution");
            experiment_test6(fileName);
        }else if (type == 7){
            System.out.println("Compare mean-performance and worst-case performance");
            experiment_test7(fileName);
        }else if (type == 8){
            System.out.println("Compare two-stage robust with two-stage stochastic programming");
            experiment_test8(fileName);
        }else if (type == 9){
            System.out.println("Compare two-stage robust with two-stage stochastic programming");
            experiment_test9(fileName);
        }

        fileWriter.close();
    }

    static private void experiment_test1(String filename) throws IloException, IOException {
        System.out.println("========================== Begin Performance Test =========================");
        System.out.println("==============================" + "Experiment 1" + "=============================");
        for (int T : timeHorizonSet) {
            InputData inputData = new InputData();
            new ReadData(filename, inputData, T);
            Parameter para = new Parameter();
            new GenerateParameter(para, inputData, T, uncertainDegree);
            inputData.showStatus();
            new SelectPaths(inputData, para, 0.4);

            //DetermineModel de = new DetermineModel(inputData, para);
            //de.solveModel();
            //DualSubProblem dsp = new DualSubProblem(inputData, para, para.getTau());
            //dsp.changeObjectiveVCoefficients(de.getVVarValue());
            //dsp.solveModel();

            CCG ccg = new CCG(inputData, para);
            CCGwithPAP ccgp = new CCGwithPAP(inputData, para);
            // BDwithPAP bdp = new BDwithPAP(inputData, para, para.getTau());
            BD bd = new BD(inputData, para);
            DetermineModel dm = new DetermineModel(inputData, para);
            // BDwithPareto bdpa = new BDwithPareto(inputData, para, para.getTau());


            System.out.println("=====================================================================");

            System.out.println("Algorithm :" + "\t"
                    //+ "BD&Pareto"+ "\t"
                    //+ "BD&PAP"+ "\t"
                    + "BD" + "\t"
                    + "CCG&PAP" + "\t"
                    + "CCG" + "\t"
            );
            System.out.println("SolveTime :" + "\t"
                    //+ bdpa.getSolveTime() + "\t"
                    // + bdp.getSolveTime() + "\t"
                    + bd.getSolveTime() + "\t"
                    + ccgp.getSolveTime() + "\t"
                    + ccg.getSolveTime() + "\t"
            );
            System.out.println("Objective  :" + "\t"
                    //+ String.format("%.2f", bdpa.getObj()) + "\t"
                    // + String.format("%.2f", bdp.getObjVal())+ "\t"
                    + String.format("%.2f", bd.getObjVal())+ "\t"
                    + String.format("%.2f", ccgp.getObjVal()) + "\t"
                    + String.format("%.2f", ccg.getObjVal()) + "\t"
            );
            System.out.println("Iteration    :" + "\t"
                    //+ bdpa.getIter() + "\t"
                    // + bdp.getIter()+ "\t"
                    + bd.getIter()+ "\t"
                    + ccgp.getIter() + "\t"
                    + ccg.getIter() + "\t"
            );
            System.out.println("=====================================================================");
            System.out.println();

            if(DefaultSetting.WhetherWriteFileLog){
                fileWriter.write("\n" + "TimeHorizon : " + T + "\n");
                fileWriter.write("UncertainDegree : " + uncertainDegree + "\n");

                inputData.writeStatus(fileWriter);

                fileWriter.write("Algorithm :" + "\t"
                        //+ "BD&Pareto"+ "\t"
                        // + "BD&PAP"+ "\t"
                        + "BD" + "\t"
                        + "CCG&PAP" + "\t"
                        + "CCG" + "\t"
                        + "\n"
                );
                fileWriter.write("SolveTime :" + "\t"
                        //+ bdpa.getSolveTime() + "\t"
                        // + bdp.getSolveTime() + "\t"
                        + bd.getSolveTime() + "\t"
                        + ccgp.getSolveTime() + "\t"
                        + ccg.getSolveTime() + "\t"
                        + "\n"
                );
                fileWriter.write("Objective  :" + "\t"
                        //+ String.format("%.2f", bdpa.getObj()) + "\t"
                        // + String.format("%.2f", bdp.getObjVal())+ "\t"
                        + String.format("%.2f", bd.getObjVal())+ "\t"
                        + String.format("%.2f", ccgp.getObjVal()) + "\t"
                        + String.format("%.2f", ccg.getObjVal()) + "\t"
                        + "\n"
                );
                fileWriter.write("Iteration    :" + "\t"
                        //+ bdpa.getIter() + "\t"
                        //+ bdp.getIter()+ "\t"
                        + bd.getIter()+ "\t"
                        + ccgp.getIter() + "\t"
                        + ccg.getIter() + "\t"
                        + "\n"
                );
                fileWriter.write("\n");
                fileWriter.flush();
            }
        }
    }

    static private void experiment_test2(String filename) throws IloException, IOException {
        System.out.println("========================== Begin Performance Test =========================");
        System.out.println("=============================" + "Experiment 2" + "=============================");

        for (int T : timeHorizonSet) {
            System.out.print("TimeHorizon : " + T + "\n");
            System.out.print("UncertainDegree : " + uncertainDegree + "\n");

            InputData inputData = new InputData();
            new ReadData(filename, inputData, T);
            Parameter para = new Parameter();
            new GenerateParameter(para, inputData, T, uncertainDegree);

            DetermineModel                              de = new DetermineModel(inputData, para);
            DetermineModelReactive          der = new DetermineModelReactive(inputData, para);
            CCGwithPAP                                      cp = new CCGwithPAP(inputData, para);
            CCGwithPAP_Reactive                 cpr = new CCGwithPAP_Reactive(inputData, para);

            System.out.println("==========================================");
            System.out.println("Algorithm :" + "\t"
                    + "Determine" + "\t"
                    + "Determine&Reactive" + "\t"
                    + "CCG&PAP"+ "\t"
                    + "CCG&PAP&Reactive" + "\t"
            );
            System.out.println("SolveTime :" + "\t"
                    + de.getSolveTime() + "\t"
                    +  der.getSolveTime() + "\t"
                    + cp.getSolveTime()  + "\t"
                    + cpr.getSolveTime()  + "\t"
            );
            System.out.println("Objective  :" + "\t"
                    + String.format("%.2f", de.getObjVal()) + "\t"
                    + String.format("%.2f", der.getObjVal()) + "\t"
                    + String.format("%.2f", cp.getObjVal())+ "\t"
                    + String.format("%.2f", cpr.getObjVal()) + "\t"
            );
            System.out.println("===========================================");
            System.out.println();

            if(DefaultSetting.WhetherWriteFileLog){
                fileWriter.write("\n" + "TimeHorizon : " + T + "\n");
                fileWriter.write("UncertainDegree : " + uncertainDegree + "\n");
                inputData.writeStatus(fileWriter);

                fileWriter.write("Algorithm :" + "\t"
                        + "Determine" + "\t"
                        + "Determine&Strategy" + "\t"
                        + "CCG&PAP"+ "\t"
                        + "CCG&PAP&Reactive" + "\t"
                        + "\n"
                );
                fileWriter.write("SolveTime :" + "\t"
                        + de.getSolveTime() + "\t"
                        + der.getSolveTime() + "\t"
                        + cp.getSolveTime() + "\t"
                        + cpr.getSolveTime() + "\t"
                        + "\n"
                );
                fileWriter.write("Objective  :" + "\t"
                        + String.format("%.2f", de.getObjVal()) + "\t"
                        + String.format("%.2f", der.getObjVal())+ "\t"
                        + String.format("%.2f", cp.getObjVal()) + "\t"
                        + String.format("%.2f", cpr.getObjVal())
                        + "\n"
                );
                fileWriter.write("Iteration    :" + "\t"
                        + 1 					  + "\t"
                        + 1 + "\t"
                        + cp.getIter()+ "\t"
                        + cpr.getIter()+ "\t"
                        + "\n"
                );
                fileWriter.write("\n");
                fileWriter.flush();
            }
        }
    }

    static private void print_data_status(String filename) throws IOException {
        System.out.println("========================== Print Data Status =========================");

        for (int T : timeHorizonSet) {
            System.out.print("TimeHorizon : " + T + "\n");
            InputData inputData = new InputData();
            new ReadData(filename, inputData, T);
            Parameter para = new Parameter();
            new GenerateParameter(para, inputData, T, uncertainDegree);
            System.out.print("UncertainDegree : " + uncertainDegree + "\n");

            inputData.showStatus();
        }
    }

    static private void experiment_test3(String filename) throws IOException, IloException {
        print_data_status(filename);

        System.out.println("========================== Begin Performance Test =========================");
        System.out.println("=============================" + "Experiment 3" + "=============================");

        for (int T : timeHorizonSet) {
            System.out.print("TimeHorizon : " + T + "\n");
            double[] percentSet = new double[]{0.05, 0.15, 0.25, 0.35, 0.45, 0.55, 0.65, 0.75, 0.85, 0.95, 1.0};
            //double[] percentSet = new double[]{0, 0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7, 0.8, 0.9, 1.0};
            for (double percent : percentSet) {
                System.out.print("Path Percent : " + percent + "\n");
                InputData inputData = new InputData();
                new ReadData(filename, inputData, T);
                Parameter para = new Parameter();
                new GenerateParameter(para, inputData, T, uncertainDegree);
                System.out.print("UncertainDegree : " + uncertainDegree + "\n");
                inputData.showStatus();
                new SelectPaths(inputData, para, percent);

                CCGwithPAP ccgp = new CCGwithPAP(inputData, para);
                CCG ccg = new CCG(inputData, para);
                BD bd = new BD(inputData, para);
                BDwithPAP bdp = new BDwithPAP(inputData, para);

                System.out.println("=====================================================================");

                System.out.println("Algorithm :" + "\t"
                        + "BD&PAP" + "\t"
                        + "BD" + "\t"
                        + "CCG&PAP" + "\t"
                        + "CCG" + "\t"
                );
                System.out.println("SolveTime :" + "\t\t"
                        + bdp.getSolveTime() + "\t"
                        + bd.getSolveTime() + "\t"
                        + ccgp.getSolveTime() + "\t"
                        + ccg.getSolveTime() + "\t"
                );
                System.out.println("Objective :" + "\t\t"
                        + String.format("%.2f", bdp.getObjVal()) + "\t"
                        + String.format("%.2f", bd.getObjVal()) + "\t"
                        + String.format("%.2f", ccgp.getObjVal()) + "\t"
                        + String.format("%.2f", ccg.getObjVal()) + "\t"
                );
                System.out.println("Iteration    :" + "\t\t"
                        + bdp.getIter() + "\t"
                        + bd.getIter() + "\t"
                        + ccgp.getIter() + "\t"
                        + ccg.getIter() + "\t"
                        + "\n"
                );
                System.out.println("=====================================================================");
                System.out.println();

                if (WhetherWriteFileLog) {
                    fileWriter.write("\n");
                    fileWriter.write("=====================================================================");
                    fileWriter.write("\n");
                    fileWriter.write("Path Percent : " + (1 - percent) + "\n");
                    inputData.writeStatus(fileWriter);

                    fileWriter.write("Algorithm :" + "\t"
                            + "BD&PAP" + "\t"
                            + "BD" + "\t"
                            + "CCG&PAP" + "\t"
                            + "CCG" + "\t"
                            + "\n"
                    );
                    fileWriter.write("SolveTime :" + "\t"
                            + bdp.getSolveTime() + "\t"
                            + bd.getSolveTime() + "\t"
                            + ccgp.getSolveTime() + "\t"
                            + ccg.getSolveTime() + "\t"
                            + "\n"
                    );
                    fileWriter.write("Objective  :" + "\t"
                            + String.format("%.2f", bdp.getObjVal()) + "\t"
                            + String.format("%.2f", bd.getObjVal()) + "\t"
                            + String.format("%.2f", ccgp.getObjVal()) + "\t"
                            + String.format("%.2f", ccg.getObjVal()) + "\t"
                            + "\n"
                    );
                    fileWriter.write("Iteration    :" + "\t"
                            + bdp.getIter() + "\t"
                            + bd.getIter() + "\t"
                            + ccgp.getIter() + "\t"
                            + ccg.getIter() + "\t"
                            + "\n"
                    );
                    fileWriter.write("=====================================================================");
                    fileWriter.write("\n");
                    fileWriter.flush();
                }
            }
        }
    }

    static private void experiment_test4(String filename) throws IOException, IloException {
        CCG_PAP_Use_Sp = true;
        print_data_status(filename);
        System.out.println("========================== Begin Performance Test =========================");
        System.out.println("=============================" + "Experiment 4" + "=============================");

        fileWriter.write("\n" + "===========================================" + "\n");
        fileWriter.write("TimeHorizon" + "\t" + "Homo-Obj" + "\t" + "Hetero-Obj"  + "\t"
                + "Homo-OC" + "\t" + "Hetero-OC"  + "\t"
                + "Homo-LC" + "\t" + "Hetero-LC"  + "\t"
                + "Homo-EC" + "\t" + "Hetero-EC"  + "\t"
                + "Homo-RC" + "\t" + "Hetero-RC"  + "\t"
                + "Homo-PC" + "\t" + "Hetero-PC"  + "\t"
                + "Homo-WP" + "\t" + "Hetero-WP"  + "\t"
                + "\n");

        for (int T : timeHorizonSet) {
            System.out.print("TimeHorizon : " + T + "\n");
            System.out.print("UncertainDegree : " + uncertainDegree + "\n");
            InputData inputData = new InputData();
            new ReadData(filename, inputData, T);
            Parameter para = new Parameter();
            new GenerateParameter(para, inputData, T, uncertainDegree);
            inputData.showStatus();
            new SelectPaths(inputData, para, 0.4);

            FleetType = "Homo";
            CCGwithPAP ccgp_homo = new CCGwithPAP(inputData, para);
            FleetType = "Hetero";
            CCGwithPAP ccgp_hetero = new CCGwithPAP(inputData, para);

            if(DefaultSetting.WhetherWriteFileLog) {
                fileWriter.write(T + "\t");
                fileWriter.write(ccgp_homo.getObjVal() + "\t");
                fileWriter.write(ccgp_hetero.getObjVal() + "\t");
                fileWriter.write(ccgp_homo.getOperationCost() + "\t");
                fileWriter.write(ccgp_hetero.getOperationCost() + "\t");
                fileWriter.write(ccgp_homo.getLadenDemurrageCost() + "\t");
                fileWriter.write(ccgp_hetero.getLadenDemurrageCost() + "\t");
                fileWriter.write(ccgp_homo.getEmptyDemurrageCost() + "\t");
                fileWriter.write(ccgp_hetero.getEmptyDemurrageCost() + "\t");
                fileWriter.write(ccgp_homo.getRentalCost() + "\t");
                fileWriter.write(ccgp_hetero.getRentalCost() + "\t");
                fileWriter.write(ccgp_homo.getPenaltyCost() + "\t");
                fileWriter.write(ccgp_hetero.getPenaltyCost() + "\t");
                fileWriter.write(ccgp_homo.getWorstPerformance() + "\t");
                fileWriter.write(ccgp_hetero.getWorstPerformance() + "\t");
                fileWriter.write("\n");
                fileWriter.flush();
            }
        }
    }

    static private void experiment_test5(String filename) throws IOException, IloException {
        CCG_PAP_Use_Sp = true;
        System.out.println("========================== Begin Performance Test =========================");
        System.out.println("==========================" + "Experiment 5" + "==========================");

        fileWriter.write("\n" + "===========================================" + "\n");
        fileWriter.write("TimeHorizon" + "\t" + "Obj." + "\t"
                + "OC" + "\t"
                + "LC" + "\t"
                + "C" + "\t"
                + "RC" + "\t"
                + "PC" + "\t"
                + "WP" + "\t"
                + "\n");

        for(String vessel_type: new String[]{"II", "III"}){
            writeSettings(fileWriter);
            printSettings();


            VesselCapacityRange = vessel_type;

            for (int T : timeHorizonSet) {
                System.out.print("TimeHorizon : " + T + "\n");

                InputData inputData = new InputData();
                new ReadData(filename, inputData, T);

                inputData.showStatus();

                Parameter para = new Parameter();
                new GenerateParameter(para, inputData, T, uncertainDegree);
                System.out.print("UncertainDegree : " + uncertainDegree + "\n");

                new SelectPaths(inputData, para, 0.4);

                CCGwithPAP ccgp = new CCGwithPAP(inputData, para, para.getTau());

                if(DefaultSetting.WhetherWriteFileLog) {
                    fileWriter.write(T + "\t");
                    fileWriter.write(ccgp.getObjVal() + "\t");
                    fileWriter.write(ccgp.getOperationCost() + "\t");
                    fileWriter.write(ccgp.getLadenDemurrageCost() + "\t");
                    fileWriter.write(ccgp.getEmptyDemurrageCost() + "\t");
                    fileWriter.write(ccgp.getRentalCost() + "\t");
                    fileWriter.write(ccgp.getPenaltyCost() + "\t");
                    fileWriter.write(ccgp.getWorstPerformance() + "\t");
                    fileWriter.write("\n");
                    fileWriter.flush();
                }

            }
        }
    }

    static private void experiment_test6(String fileName) throws IOException, IloException {
        CCG_PAP_Use_Sp = true;
        System.out.println("========================== Begin Performance Test =========================");
        System.out.println("==========================" + "Experiment 6" + "==========================");
        System.out.println("DistributionType : " + distributionType + "\n");
        String filename = fileName+"/";

        fileWriter.write("\n" + "===========================================" + "\n");
        fileWriter.write("TimeHorizon" + "\t" + "Obj." + "\t"
                + "OC" + "\t"
                + "LC" + "\t"
                + "C" + "\t"
                + "RC" + "\t"
                + "PC" + "\t"
                + "WP" + "\t"
                + "\n");
        int T = defaultTimeHorizon;
        System.out.print("TimeHorizon : " + T + "\n");
        //double[] sigma_factor_set = new double[]{0.5, 1.0, 1.5, 2.0, 2.5, 3.0, 3.5, 4.0};
        double[] sigma_factor_set = new double[]{1};
        for (double sigma_factor : sigma_factor_set) {
            System.out.print("UncertainDegree : " + uncertainDegree + "\n");
            log_normal_sigma_factor = sigma_factor;
            System.out.print("log_normal_sigma_factor : " + log_normal_sigma_factor + "\n");

            InputData inputData = new InputData();
            new ReadData(filename, inputData, T);
            inputData.showStatus();
            Parameter para = new Parameter();

            //distributionType = "Log-Normal";
            distributionType = "Uniform";
            new GenerateParameter(para, inputData, T, uncertainDegree);
            new SelectPaths(inputData, para, 0.4);

            CCGwithPAP ccgp = new CCGwithPAP(inputData, para, para.getTau());

            if(DefaultSetting.WhetherWriteFileLog) {
                fileWriter.write(sigma_factor + "\t");
                fileWriter.write(ccgp.getObjVal() + "\t");
                fileWriter.write(ccgp.getOperationCost() + "\t");
                fileWriter.write(ccgp.getLadenDemurrageCost() + "\t");
                fileWriter.write(ccgp.getEmptyDemurrageCost() + "\t");
                fileWriter.write(ccgp.getRentalCost() + "\t");
                fileWriter.write(ccgp.getPenaltyCost() + "\t");
                fileWriter.write(ccgp.getWorstPerformance() + "\t");
                fileWriter.write(ccgp.getMeanPerformance() + "\t");
                fileWriter.write("\n");
                fileWriter.flush();
            }
        }
    }

    static private void experiment_test7(String filename) throws IOException, IloException {
        WhetherGenerateSamples = true;
        WhetherCalculateMeanPerformance = true;
        UseHistorySolution = true;
        CCG_PAP_Use_Sp = true;

        System.out.println("========================== Begin Performance Test =========================");
        System.out.println("=============================" + "Experiment 7" + "=============================");

        fileWriter.write("\n");
        fileWriter.write("=====================================================================");
        fileWriter.write("\n");
        fileWriter.write("Methods" + "\t"
                + "DM-MeanPerformance" + "\t"
                + "DM-WorstPerformance" + "\t"
                + "BD-MeanPerformance" + "\t"
                + "BD-WorstPerformance" + "\t"
                + "CCG-MeanPerformance" + "\t"
                + "CCG-WorstPerformance" + "\t"
                + "CCG&PAP-MeanPerformance" + "\t"
                + "CCG&PAP-WorstPerformance" + "\t"
                + "\n");

        writeSettings(fileWriter);
        printSettings();
        int T = defaultTimeHorizon;

        UseHistorySolution = false;

        System.out.print("TimeHorizon : " + T + "\n");
        System.out.print("UncertainDegree : " + uncertainDegree + "\n");
        InputData inputData = new InputData();
        new ReadData(filename, inputData, T);
        Parameter para = new Parameter();
        new GenerateParameter(para, inputData, T, uncertainDegree);
        inputData.showStatus();
        new SelectPaths(inputData, para, 0.4);
        System.out.println("Tau : " + para.getTau());
        // String[] methods = new String[]{"DM", "BD", "CCG", "CCG&PAP"};
            /*String[] methods = new String[]{"BD", "CCG", "CCG&PAP"};
            for(String method : methods){
                String AlgoID = method + "-R"+ inputData.getShipRouteSet().size() + "-T" + T + "-"+ FleetType + "-S" + randomSeed + "-V" + VesselCapacityRange;
                if((inputData.getHistorySolutionSet().get(AlgoID) != (null))) {
                    int[][] vValue = para.solutionToVValue(inputData.getHistorySolutionSet().get(AlgoID));
                    String samplefilename = method + "-R"+ inputData.getShipRouteSet().size() + "-T"
                            + para.getTimeHorizon() + "-"+ FleetType + "-S" + randomSeed + "-SampleTestResult"+ ".csv";
                    File file = new File(RootPath + AlgoLogPath + samplefilename);
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

                    double mp_operation_cost = 0;
                    if(UseHistorySolution){
                        mp_operation_cost = para.getOperationCost(vValue);
                    }

                    double[] sample_sub_opera_costs = new double[numSampleScenes];
                    double[] sample_laden_costs = new double[numSampleScenes];
                    double[] sample_empty_costs = new double[numSampleScenes];
                    double[] sample_rental_costs = new double[numSampleScenes];
                    double[] sample_penalty_costs = new double[numSampleScenes];

                    double sum_sub_opera_costs = 0;
                    double worst_total_cost = 0;
                    double worst_second_cost = 0;
                    SubProblem sp = new SubProblem(inputData, para, vValue);
                    for (int sce = 0; sce < numSampleScenes; sce++) {
                        sp.changeDemandConstraintCoefficients(para.getSampleScenes()[sce]);
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

                    System.out.println("Worst Performance = " + worst_total_cost);
                    System.out.println("Mean Performance = " + (mp_operation_cost + sum_sub_opera_costs / numSampleScenes));
                    System.out.println("Worst Second Stage Cost = " + worst_second_cost);
                    System.out.println("Mean Second Stage Cost = " + sum_sub_opera_costs / numSampleScenes);

                    filewriter.close();
                }
            }*/


        ////////////////////////////////////////
        DetermineModel dm = new DetermineModel(inputData, para);
        CCGwithPAP ccgp = new CCGwithPAP(inputData, para);
        //BD bd = new BD(inputData, para, para.getTau());
        //CCG ccg = new CCG(inputData, para, para.getTau());

        fileWriter.write("\n" + "TimeHorizon : " + T + "\n");
        fileWriter.write("UncertainDegree : " + uncertainDegree + "\n");
        fileWriter.write("Tau : " + para.getTau() + "\n");
        fileWriter.write(T + "\t"
                + dm.getMeanPerformance() + "\t"
                + dm.getWorstPerformance() + "\t"
                //+ bd.getMeanPerformance() + "\t"
                //+ bd.getWorstPerformance() + "\t"
                //+ ccg.getMeanPerformance() + "\t"
                // + ccg.getWorstPerformance() + "\t"
                + ccgp.getMeanPerformance() + "\t"
                + ccgp.getWorstPerformance() + "\t"
                + "\n");
        fileWriter.write(T + "\t"
                + dm.getMeanSecondStageCost() + "\t"
                + dm.getWorstSecondStageCost() + "\t"
                //+ bd.getMeanSecondStageCost() + "\t"
                //+ bd.getWorstSecondStageCost() + "\t"
                // + ccg.getMeanSecondStageCost() + "\t"
                //+ ccg.getWorstSecondStageCost() + "\t"
                + ccgp.getMeanSecondStageCost() + "\t"
                + ccgp.getWorstSecondStageCost() + "\t"
                + "\n");
        fileWriter.write("=====================================================================");
        fileWriter.flush();
        fileWriter.close();
    }

    static private void experiment_test8(String filename) throws IOException, IloException {
        System.out.println("========================== Begin Performance Test =========================");
        System.out.println("=============================" + "Experiment 8" + "=============================");

        writeSettings(fileWriter);
        printSettings();
        for (int T : timeHorizonSet) {

            System.out.print("TimeHorizon : " + T + "\n");
            System.out.print("UncertainDegree : " + uncertainDegree + "\n");
            InputData inputData = new InputData();
            new ReadData(filename, inputData, T);
            Parameter para = new Parameter();
            new GenerateParameter(para, inputData, T, uncertainDegree);
            inputData.showStatus();
            new SelectPaths(inputData, para, 0.4);

            SOwithBD so = new SOwithBD(inputData, para);
            System.out.println("SOwithBD : " + so.getObjVal() + "\t" + so.getSolveTime() + "\t" + so.getIter());
        }
    }

    static private void experiment_test9(String fileName) throws IOException, IloException {
        System.out.println("========================== Begin Performance Test =========================");
        System.out.println("=============================" + "Experiment 9" + "=============================");

        String filename = fileName+"/";
        writeSettings(fileWriter);
        printSettings();
        for (int T : timeHorizonSet) {

            System.out.print("TimeHorizon : " + T + "\n");
            System.out.print("UncertainDegree : " + uncertainDegree + "\n");
            InputData inputData = new InputData();
            new ReadData(filename, inputData, T);
            Parameter para = new Parameter();
            new GenerateParameter(para, inputData, T, uncertainDegree);
            inputData.showStatus();
            new SelectPaths(inputData, para, 0.4);

            BDwithPareto bdpa = new BDwithPareto(inputData, para);
            BD bd = new BD(inputData, para);

            System.out.println("BDwithPareto : \t" + bdpa.getObjVal() + "\t" + bdpa.getSolveTime() + "\t" + bdpa.getIter());
            System.out.println("BD : \t" + bd.getObjVal() + "\t" + bd.getSolveTime() + "\t" + bd.getIter());
        }
    }
}