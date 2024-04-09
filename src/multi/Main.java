package multi;

import java.io.*;
import java.util.Random;
import ilog.concert.IloException;


import multi.section.PerformanceExperiment;
import multi.section.SensitivityAnalysis;

public class Main extends DefaultSetting {

	 public static void main(String[] args) throws IloException, IOException {

		System.out.println("Instance:" + args[0]);
		CasePath = "data" + args[0] + "/";
		System.out.println("Experiment:" + args[1]);

		int Instance = Integer.parseInt(args[0]);
		int Experiment = Integer.parseInt(args[1]);

		if (args.length >= 3) {
			if(!args[2].equals("-")){
				RootPath = args[2]; // set the root path
				System.out.println("RootPath：" + RootPath);
			}
		}
		if(args.length >= 4){
			if(!args[3].equals("-")) {
				MIPGapLimit = Double.parseDouble(args[3]);
				System.out.println("MIPGapLimit：" + args[3]);
			}
		}
		if(args.length >= 5){
			randomSeed = Integer.parseInt(args[4]);
			System.out.println("Random Seed:" + args[4]);
		}
		if(args.length >= 6){
			budgetCoefficient = Double.parseDouble(args[5]);
			System.out.println("Budget Coefficient:" + args[5]);
		}
		if(args.length >= 7){
			defaultUncertainDegree = Double.parseDouble(args[6]);
			System.out.println("Uncertain Degree:" + args[6]);
		}
		int flag = 1;
		if(args.length >= 8){
			if(args[7].equals("P")){
				flag = 1;
				System.out.println("Numerical:" + "Performance Test");
			}
			else if(args[7].equals("S")){
				flag = 2;
				System.out.println("Numerical:" + "Sensitivity Analysis");
			}
		}

		// print the heap memory setting of JVM
		System.out.println("Max heap Memory = " + (Runtime.getRuntime().maxMemory() >> 20) + "M");
		System.out.println("Total heap Memory = " + (Runtime.getRuntime().totalMemory() >> 20) + "M");
		System.out.println("Max Available Cores = "+ Runtime.getRuntime().availableProcessors());

		random = new Random(randomSeed);
		System.out.println("=============== Seed = "+randomSeed+"===============");
		System.out.println("Fleet Type DefaultSetting: " + DefaultSetting.FleetType);

		if(flag == 1){
			// input parameters :
			// instance	experiment
			new PerformanceExperiment(Instance, Experiment);
		}else if(flag == 2){
			// input parameters :
			// Instance	Experiment	 Algo
			new SensitivityAnalysis(Instance, Experiment, "CCG&PAP");
		}
	 }
}

