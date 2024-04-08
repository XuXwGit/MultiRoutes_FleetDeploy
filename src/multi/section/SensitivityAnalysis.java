package multi.section;

import ilog.concert.IloException;
import multi.DefaultSetting;
import multi.algo.CCGwithPAP;
import multi.data.*;
import multi.strategy.SelectPaths;
import multi.structure.Port;

import java.io.File;
import java.io.FileWriter;
import java.io.IOException;

public class SensitivityAnalysis extends DefaultSetting {
//    private double[] uncertainDegreeSet = {0.01, 0.02, 0.03,0.04,0.06,0.07,0.08,0.09};
    private static int defaultTimeHorizon;
    private double[] uncertainDegreeSet = {0.005, 0.015, 0.025, 0.035,0.045, 0.055, 0.065, 0.075, 0.085, 0.095};
    private double[] ContainerPathCostSet = {0.80, 0.825, 0.85, 0.875, 0.90, 0.925, 0.95, 0.975,
        1.025,1.05, 1.0725, 1.10,1.125, 1.15, 1.175, 1.20};
    private double[] rentalContainerCostSet = {0.60, 0.65, 0.70, 0.75, 0.80, 0.85, 0.90, 0.95, 1.00,
            1.05, 1.10, 1.15,  1.20, 1.25, 1.30, 1.35, 1.40};
    //private double[] penaltyCostSet = {0.80, 0.85, 0.90,  0.95, 1.00, 1.05, 1.10, 1.15, 1.20，0.825, 0.875, 0.925, 0.975};
 private double[] penaltyCostSet = {1.025, 1.075, 1.125, 1.175};

    private int[] turnOverTimeSet = {0,1, 2, 3, 4,5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20, 21, 22, 23,  24, 25,  26, 27, 28};
    private int[] initialContainerSet = { 0,1, 2, 3, 4,5, 6, 7,
                                                                        8, 9, 10, 11, 12, 13, 14,
                                                                        15, 16, 17, 18, 19, 20, 21,
                                                                        22, 23,  24, 25,  26, 27, 28,
                                                                        29, 30, 31, 32, 33, 34, 35,
                                                                        36, 37, 38, 39, 40, 41, 42};
    private  int[] timeHorizonSet = {60, 75, 90, 105, 120, 135, 150, 165, 180};

    private double uncertainDegree = 0.05;
    private FileWriter fileWriter;
    private String Algo;
    public SensitivityAnalysis(int instance, int type, String algo)
    {
        super();
        this.Algo = algo;

        try{

            if(DefaultSetting.WhetherWriteFileLog){
                File file = new File(
                RootPath +  TestResultPath
                        + "SensitivityAnalysis" + instance
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

            }

            String fileName = DataPath;
            if(instance == 1){
                fileName += "data1/";
                defaultTimeHorizon = 70;
            } else if (instance == 2) {
                fileName += "data2/";
                defaultTimeHorizon = 90;
                //timeHorizonSet = new int[]{60, 75, 90, 105, 120, 135};
            }
            else if (instance == 3) {
                fileName += "data3/";
                defaultTimeHorizon = 90;
            }

            InputData input = new InputData();
            new ReadData(fileName, input, defaultTimeHorizon);
            input.showStatus();

            System.out.print("Experiment " + type + ": ");
            if(type == 1){
                System.out.println("Sensitivity Analysis Varying TurnOverTime");
                VaryTurnOverTime(input);
            } else if (type == 2) {
                System.out.println("Sensitivity Analysis Varying Penalty Cost");
                VaryPenaltyCost(input);
            } else if (type == 3) {
                System.out.println("Sensitivity Analysis Varying Unit Rental Cost");
                VaryRentalCost(input);
            }
           //VaryInitialContainers(in);
           //VaryUncertainDegree(in);
           //VaryLoadAndDischargeCost(in);

        }catch (IOException e) {
            e.printStackTrace();
        } catch (IloException e) {
            throw new RuntimeException(e);
        }
    }

    private void VaryUncertainDegree(InputData in) throws IloException, IOException {
        System.out.println("=========Varying UncertainDegree from 0 to 0.20==========");

        if(DefaultSetting.WhetherWriteFileLog){
            fileWriter.write("=========Varying UncertainDegree from 0 to 0.20==========");
            fileWriter.write("\n");
        }

        double UD;
        for (int i = 0; i < uncertainDegreeSet.length; i++) {

            System.out.println("uncertainDegreeSet = "+uncertainDegreeSet[i]);

            Parameter p = new Parameter();
            UD = uncertainDegreeSet[i];
            new GenerateParameter(p, in, defaultTimeHorizon, UD);

            CCGwithPAP cp = new CCGwithPAP(in, p, (int) Math.sqrt(in.getRequestSet().size()));

            System.out.println("UD"
                    + '\t' + "LPC"
                    + '\t' + "EPC"
                    + '\t' + "LC+EC"
                    + '\t' + "RC"
                    + '\t' + "PC"
                    + '\t' + "OC"
                    + '\t' + "TC");
            System.out.println(UD
                    + "\t" + String.format("%.2f", cp.getLadenDemurrageCost())
                    + "\t" + String.format("%.2f", cp.getEmptyDemurrageCost())
                    + "\t" + String.format("%.2f", cp.getEmptyDemurrageCost()+cp.getLadenDemurrageCost())
                    + "\t" + String.format("%.2f", cp.getRentalCost())
                    + "\t" + String.format("%.2f", cp.getPenaltyCost())
                    + "\t" + String.format("%.2f", cp.getOperationCost())
                    + "\t" + String.format("%.2f", cp.getTotalCost())+"\n");
            System.out.println("=================================");
            System.out.println();

            if(DefaultSetting.WhetherWriteFileLog){
                fileWriter.write("UncertainDegree"
                        + '\t' + "LadenPathCost"
                        + '\t' + "EmptyPathCost"
                        + '\t' + "LC+EC"
                        + '\t' + "RentalCost"
                        + '\t' + "PenaltyCost"
                        + '\t' + "OperationCost"
                        +'\t'+"TotalCost"+'\n');
                fileWriter.write(UD
                        + "\t" + String.format("%.2f", cp.getLadenDemurrageCost())
                        + "\t" + String.format("%.2f", cp.getEmptyDemurrageCost())
                        + "\t" + String.format("%.2f", cp.getEmptyDemurrageCost()+cp.getLadenDemurrageCost())
                        + "\t" + String.format("%.2f", cp.getRentalCost())
                        + "\t" + String.format("%.2f", cp.getPenaltyCost())
                        + "\t" + String.format("%.2f", cp.getOperationCost())
                        +"\t"+String.format("%.2f", cp.getTotalCost())+"\n");
            }
        }
    }

    private void VaryLoadAndDischargeCost(InputData in) throws IOException, IloException {
        System.out.println("=========Varying Unit L&D&T Cost========");

        double LDTCoeff;
        for (int i = 0; i < ContainerPathCostSet.length; i++) {

            System.out.println("Unit ContainerPath Cost = "+ ContainerPathCostSet[i]);

            Parameter p = new Parameter();
            new GenerateParameter(p, in, defaultTimeHorizon, uncertainDegree);
            LDTCoeff = ContainerPathCostSet[i];
            double[] ladenPathCost = p.getLadenPathCost();
            double[] emptyPathCost = p.getEmptyPathCost();
            for (int j = 0; j < p.getPathSet().length; j++) {
                ladenPathCost[j] = in.getContainerPathSet().get(j).getPathCost() * LDTCoeff + p.getLadenPathDemurrageCost()[j];
                emptyPathCost[j] = in.getContainerPathSet().get(j).getPathCost() * 0.5 * LDTCoeff + p.getEmptyPathDemurrageCost()[j];
            }
            p.setLadenPathCost(ladenPathCost);
            p.setEmptyPathCost(emptyPathCost);

            CCGwithPAP cp = new CCGwithPAP(in, p);

            System.out.println("DemurrageCostCoeff"
                    + '\t' + "LPC"
                    + '\t' + "EPC"
                    + '\t' + "LC+EC"
                    + '\t' + "RC"
                    + '\t' + "PC"
                    + '\t' + "OC"
                    + '\t' + "TC");
            System.out.println(LDTCoeff
                    + "\t" + String.format("%.2f", cp.getLadenDemurrageCost())
                    + "\t" + String.format("%.2f", cp.getEmptyDemurrageCost())
                    + "\t" + String.format("%.2f", cp.getEmptyDemurrageCost()+cp.getLadenDemurrageCost())
                    + "\t" + String.format("%.2f", cp.getRentalCost())
                    + "\t" + String.format("%.2f", cp.getPenaltyCost())
                    + "\t" + String.format("%.2f", cp.getOperationCost())
                    + "\t" + String.format("%.2f", cp.getTotalCost())+"\n");
            System.out.println("=================================");
            System.out.println();
        }
    }

    private void VaryRentalCost(InputData input) throws IloException, IOException {
        System.out.println("=========Varying Unit Container Rental Cost (0.5~1.5)x20========");

        if(DefaultSetting.WhetherWriteFileLog){
            fileWriter.write("=========Varying Unit Container Rental Cost (0.5~1.5)x20========");
            fileWriter.write("\n");
        }


        double RentalCostCoeff;
        for (int i = 0; i < rentalContainerCostSet.length; i++) {

            System.out.println("RentalCost = "+rentalContainerCostSet[i]);

            Parameter para = new Parameter();
            new GenerateParameter(para, input, defaultTimeHorizon, uncertainDegree);
            new SelectPaths(input, para, 0.4);

            RentalCostCoeff = rentalContainerCostSet[i];
            para.changeRentalCost(RentalCostCoeff);

            CCGwithPAP cp = new CCGwithPAP(input, para);

            System.out.println("RentalCostCoeff"
                    + '\t' + "LPC"
                    + '\t' + "EPC"
                    + '\t' + "LC+EC"
                    + '\t' + "RC"
                    + '\t' + "PC"
                    + '\t' + "OC"
                    + '\t' + "TC");
            System.out.println(RentalCostCoeff
                    + "\t" + String.format("%.2f", cp.getLadenDemurrageCost())
                    + "\t" + String.format("%.2f", cp.getEmptyDemurrageCost())
                    + "\t" + String.format("%.2f", cp.getEmptyDemurrageCost()+cp.getLadenDemurrageCost())
                    + "\t" + String.format("%.2f", cp.getRentalCost())
                    + "\t" + String.format("%.2f", cp.getPenaltyCost())
                    + "\t" + String.format("%.2f", cp.getOperationCost())
                    + "\t" + String.format("%.2f", cp.getTotalCost())+"\n");
            System.out.println("=================================");


            if(DefaultSetting.WhetherWriteFileLog){
                fileWriter.write("RentalCostCoeff"
                        + '\t' + "LPC"
                        + '\t' + "EPC"
                        + '\t' + "LC+EC"
                        + '\t' + "RC"
                        + '\t' + "PC"
                        + '\t' + "OC"
                        + '\t' + "TC");
                fileWriter.write(RentalCostCoeff
                        + "\t" + String.format("%.2f", cp.getLadenDemurrageCost())
                        + "\t" + String.format("%.2f", cp.getEmptyDemurrageCost())
                        + "\t" + String.format("%.2f", cp.getEmptyDemurrageCost()+cp.getLadenDemurrageCost())
                        + "\t" + String.format("%.2f", cp.getRentalCost())
                        + "\t" + String.format("%.2f", cp.getPenaltyCost())
                        + "\t" + String.format("%.2f", cp.getOperationCost())
                        + "\t" + String.format("%.2f", cp.getTotalCost())+"\n");
            }

        }
    }

    private void VaryPenaltyCost(InputData input) throws IloException, IOException {
        System.out.println("=========Varying Unit Demand Penalty Cost (80%~120%)=========");

        if(DefaultSetting.WhetherWriteFileLog){
            fileWriter.write("=========Varying Unit Demand Penalty Cost (80%~120%)=========");
            fileWriter.write("\n");
        }
        
        double PenaltyCostCoeff;
        for (int i = 0; i <penaltyCostSet.length; i++) {
            System.out.println("PenaltyCostCoeff = " + penaltyCostSet[i]);

            Parameter para = new Parameter();
            new GenerateParameter(para, input, defaultTimeHorizon, uncertainDegree);
            new SelectPaths(input, para, 0.4);

            PenaltyCostCoeff = penaltyCostSet[i];
            para.changePenaltyCostForDemand(PenaltyCostCoeff);

            CCGwithPAP cp = new CCGwithPAP(input, para);

            System.out.println("PenaltyCostCoeff"
                    + '\t' + "LPC"
                    + '\t' + "EPC"
                    + '\t' + "RC"
                    + '\t' + "PC"
                    + '\t' + "OC"
                    + '\t' + "TC");
            System.out.println(PenaltyCostCoeff
                    + "\t" + String.format("%.2f", cp.getLadenDemurrageCost())
                    + "\t" + String.format("%.2f", cp.getEmptyDemurrageCost())
                    + "\t" + String.format("%.2f", cp.getEmptyDemurrageCost()+cp.getLadenDemurrageCost())
                    + "\t" + String.format("%.2f", cp.getRentalCost())
                    + "\t" + String.format("%.2f", cp.getPenaltyCost())
                    + "\t" + String.format("%.2f", cp.getOperationCost())
                    + "\t" + String.format("%.2f", cp.getTotalCost())+"\n");
            System.out.println("=================================");
        }
    }

    private void VaryTurnOverTime(InputData input) throws IloException, IOException {
        System.out.println("=========Varying TurnOverTime (0~28) =========");

        int turnOverTime;
        for (int i = 0; i <turnOverTimeSet.length; i++) {
            System.out.println("******************** TurnOverTime = " + turnOverTimeSet[i]+"********************");

            Parameter para = new Parameter();
            new GenerateParameter(para, input, defaultTimeHorizon, uncertainDegree);
            new SelectPaths(input, para, 0.4);

            turnOverTime = turnOverTimeSet[i];
            para.setTurnOverTime(turnOverTime);

            CCGwithPAP cp = new CCGwithPAP(input, para);

            System.out.println("turnOverTime"
                    + '\t' + "LPC"
                    + '\t' + "EPC"
                    + '\t' + "LC+EC"
                    + '\t' + "RC"
                    + '\t' + "PC"
                    + '\t' + "OC"
                    + '\t' + "TC");
            System.out.println(turnOverTime
                    + "\t" + String.format("%.2f", cp.getLadenDemurrageCost())
                    + "\t" + String.format("%.2f", cp.getEmptyDemurrageCost())
                    + "\t" + String.format("%.2f", cp.getEmptyDemurrageCost()+cp.getLadenDemurrageCost())
                    + "\t" + String.format("%.2f", cp.getRentalCost())
                    + "\t" + String.format("%.2f", cp.getPenaltyCost())
                    + "\t" + String.format("%.2f", cp.getOperationCost())
                    + "\t" + String.format("%.2f", cp.getTotalCost())+"\n");
            System.out.println("=================================");
        }
    }

    private void VaryInitialContainers(InputData in) throws IloException, IOException {
        System.out.println("=========Varying initialContainers (0~28) =========");

        int initialContainers;
        for (int i = 0; i <initialContainerSet.length; i++) {
            System.out.println("initialContainers = " + initialContainerSet[i]);

            Parameter p = new Parameter();
            new GenerateParameter(p, in, defaultTimeHorizon, uncertainDegree);

            // reset initial empty containers
            initialContainers = initialContainerSet[i];
            // calculate initial number of empty containers for each port at time 0
            // initial number of empty container in pp = total demands which origins in port pp * [0.8, 1.0]
            int[] initialEmptyContainer =new int [in.getPortSet().size()];
            int x=0;
            double alpha=0.8+0.2*random.nextDouble();
            int totalOwnedEmptyContainers = 0;
            for(Port pp:in.getPortSet())
            {
                for(int ii = 0; ii<in.getRequestSet().size(); ii++)
                {
                    if(pp.getPort().equals(in.getRequestSet().get(ii).getOriginPort())
                            &&in.getRequestSet().get(ii).getW_i_Earliest()<initialContainers)
                    {
                        initialEmptyContainer [x]=(int) (initialEmptyContainer [x]+alpha*p.getDemand()[i]);
                        totalOwnedEmptyContainers += initialEmptyContainer [x];
                    }
                }
                x=x+1;
            }
            System.out.println("Total Initial Owned Empty Containers = "+totalOwnedEmptyContainers);
            p.setInitialEmptyContainer(initialEmptyContainer);

            CCGwithPAP cp = new CCGwithPAP(in, p, (int) Math.sqrt(in.getRequestSet().size()));

            System.out.println("initialContainers"
                    + '\t' + "LPC"
                    + '\t' + "EPC"
                    + '\t' + "RC"
                    + '\t' + "PC"
                    + '\t' + "OC"
                    + '\t' + "TC");
            System.out.println(initialContainers
                    + "\t" + String.format("%.2f", cp.getLadenDemurrageCost())
                    + "\t" + String.format("%.2f", cp.getEmptyDemurrageCost())
                    + "\t" + String.format("%.2f", cp.getEmptyDemurrageCost()+cp.getLadenDemurrageCost())
                    + "\t" + String.format("%.2f", cp.getRentalCost())
                    + "\t" + String.format("%.2f", cp.getPenaltyCost())
                    + "\t" + String.format("%.2f", cp.getOperationCost())
                    + "\t" + String.format("%.2f", cp.getTotalCost())+"\n");
            System.out.println("=================================");
        }
    }
}
