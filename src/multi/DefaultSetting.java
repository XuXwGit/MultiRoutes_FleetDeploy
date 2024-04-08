package multi;

import java.io.FileWriter;
import java.io.IOException;
import java.util.Random;

public class DefaultSetting {
	//////////////////////////////////
	/* Numerical Experiment Test */
	// Vessel Capacity / Nums
	public static String VesselCapacityRange = "I";
	// use heterogeneous / homogeneous fleet
	// Homo / Hetero
	public static String FleetType = "Homo";
	//////////////////////////////////


	//////////////////////////////////
	/* Strategy DefaultSetting*/
	public static double reducePathPercentage = 0;
	public static int MaxLadenPathsNum = 5;
	public static int MaxEmptyPathsNum = 5;
	/* Algo enhancement strategy */
	public static boolean UseParetoOptimalCut = true;
	public static boolean UseLocalSearch = true;
	//////////////////////////////////


	//////////////////////////////////
	/* Default Data Parameter Setting */
	// default unit rental cost : 50
	public int DefaultUnitRentalCost = 50;
	// default unit demurrage cost : 175 / 100
	public int DefaultLadenDemurrageCost = 175;
	public int DefaultEmptyDemurrageCost = 100;
	// default unit loading/discharge/transshipment cost : 20/20/30
	public int DefaultUnitLoadingCost = 20;
	public int DefaultUnitDischargeCost = 20;
	public int DefaultUnitTransshipmentCost = 30;
	public int DefaultTurnOverTime = 14;
	//////////////////////////////////


	//////////////////////////////////
	// whether show DefaultSetting information
	public static boolean DebugEnable = false;
	// whether show DefaultSetting information in GenerateParameter
	public static boolean GenerateParamEnable = false;
	// whether show DefaultSetting information in subProblem
	public static boolean SubEnable = true;
	// whether show DefaultSetting information in DualProblem
	public static boolean DualEnable = false;
	// whether show DefaultSetting information in DualProblem
	public static boolean DualSubEnable = true;
	// whether show DefaultSetting information in MasterProblem
	public static boolean MasterEnable = false;
	//////////////////////////////////


	//////////////////////////////////
	/* Input Data Settings */
	public static double RequestIncludeRange = 0;
	public static boolean WhetherAllowSameRegionTrans = true;
	public static boolean WhetherCuttingOverCostPaths = true;
	//////////////////////////////////


	//////////////////////////////////
	/* Random Setting */
	// Log-Normal / Uniform / Normal
	public static String distributionType = "Uniform";
	public static Random random;
	public static int randomSeed = 0;
	public static boolean WhetherGenerateSamples = true;
	public static boolean WhetherCalculateMeanPerformance = false;
	public static boolean WhetherWriteSampleTests = true;
	public static boolean WhetherLoadSampleTests = false;
	public static int numSampleScenes = 1000;
	public static double log_normal_sigma_factor = 1.0;
	public static double budgetCoefficient = 1.0;
	public static double defaultUncertainDegree = 0.15;
	public static double penaltyCoefficient = 1.0;
	public static int initialEmptyContainers = 28  ;
	//////////////////////////////////


	//////////////////////////////////
	public static boolean WhetherWriteFileLog = false;
	public static boolean WhetherPrintFileLog = false;
	public static boolean WhetherPrintDataStatus = false;
	public static boolean WhetherPrintVesselDecision = false;
	public static boolean WhetherPrintRequestDecision = false;
	public static boolean WhetherPrintIteration = false;
	public static boolean WhetherPrintSolveTime = false;
	public static boolean WhetherPrintProcess = true;
	//////////////////////////////////


	//////////////////////////////////
	/* Cplex Solver Settings */
	// whether export model
	public static boolean WhetherExportModel = true;
	// whether print output log
	public static boolean WhetherCloseOutputLog = true;
	// MIP solve Gap limit
	public static double MIPGapLimit = 1e-3;
	// MIP solve Time limit
	public static double MIPTimeLimit = 36000; //s
	public static int MaxThreads = Runtime.getRuntime().availableProcessors();
	public static long MaxWorkMem = (Runtime.getRuntime().maxMemory() >> 20);
	//////////////////////////////////


	//////////////////////////////////
	/* Algo DefaultSetting*/
	public static int maxIterationNum = 100;
	public static int maxIterationTime = 3600; //s
	public static double boundGapLimit = 1.0;
	public static boolean WhetherSetInitialSolution = false;
	public static boolean WhetherAddInitializeSce = false;
	public static boolean CCG_PAP_Use_Sp = true;
	public static boolean UseHistorySolution = false;
	//////////////////////////////////


	//////////////////////////////////
	/* Java Programming DefaultSetting */
	public static boolean WhetherUseMultiThreads = false;
	public static int ProgressBarWidth = 50;
	//////////////////////////////////


	//////////////////////////////////
	/* Root path*/
	public static String RootPath = "D:/Multi-Routes Revised/Code/MultiRoute0504/";
	public static String DataPath = "data/";
	public static String CasePath = "data1/";
	public static String ExportModelPath = "model/";
	public static String AlgoLogPath = "log/";
	public static String SolutionPath = "solution/";
	public static String TestResultPath = "result/";
	//////////////////////////////////


	//////////////////////////////////
	// print progress bar
	protected static void drawProgressBar(int progress) {
		int completedBars = progress * ProgressBarWidth / 100;
		StringBuilder progressBar = new StringBuilder();
		progressBar.append("\r[");

		for (int i = 0; i < ProgressBarWidth; i++) {
			if (i < completedBars) {
				progressBar.append("=");
			} else if (i == completedBars) {
				progressBar.append(">");
			} else {
				progressBar.append("   ");
			}
		}
		progressBar.append("] ").append(progress).append("%");
		// 先清除整行再打印进度条
		progressBar.append("\r");
		System.out.print(progressBar.toString());
		System.out.flush();
	}

	// other print/record methods
	protected static void printSettings(){
		System.out.print("======================"+ "Settings" + "======================\n");
		System.out.print("FleetType = " + FleetType + "\n");
		System.out.print("Vessel Set = " + VesselCapacityRange + "\n");
		System.out.print("Random Distribution = " + distributionType + "\n");
		System.out.print("MIPGapLimit = " + MIPGapLimit + "\n");
		System.out.print("MIPTimeLimit = " + MIPTimeLimit + "s"+ "\n");
		System.out.print("MaxThreads = " + MaxThreads + "\n");
		System.out.print("MaxWorkMem = " + MaxWorkMem +"M"+ "\n");
		System.out.print("NumSampleScenes = " + numSampleScenes + "\n");
		System.out.print("maxIterationNum = " + maxIterationNum + "\n");
		System.out.print("maxIterationTime = " + maxIterationTime +"s"+ "\n");
		System.out.print("boundGapLimit = " + boundGapLimit + "\n");
		System.out.print("RandomSeed = "+randomSeed + "\n");
		System.out.print("WhetherLoadHistorySolution = " + UseHistorySolution + "\n");
		System.out.print("WhetherAddInitializeSce = " + WhetherAddInitializeSce + "\n");
	}
	protected static void writeSettings(FileWriter fileWriter) {
		try {
			fileWriter.write("======================"+ "Settings" + "======================\n");
			fileWriter.write("FleetType = " + FleetType + "\n");
			fileWriter.write("Vessel Set = " + VesselCapacityRange + "\n");
			fileWriter.write("Random Distribution = " + distributionType + "\n");
			fileWriter.write("MIPGapLimit = " + MIPGapLimit + "\n");
			fileWriter.write("MIPTimeLimit = " + MIPTimeLimit  + "s" + "\n");
			fileWriter.write("MaxThreads = " + MaxThreads + "\n");
			fileWriter.write("MaxWorkMem = " + MaxWorkMem + "M" + "\n");
			fileWriter.write("NumSampleScenes = " + numSampleScenes + "\n");
			fileWriter.write("maxIterationNum = " + maxIterationNum + "\n");
			fileWriter.write("maxIterationTime = " + maxIterationTime  + "s" + "\n");
			fileWriter.write("boundGapLimit = " + boundGapLimit + "\n");
			fileWriter.write("RandomSeed = "+randomSeed + "\n");
			fileWriter.write("WhetherLoadHistorySolution = " + UseHistorySolution + "\n");
			fileWriter.write("WhetherAddInitializeSce = " + WhetherAddInitializeSce + "\n");
			fileWriter.flush();
		} catch (IOException e) {
			throw new RuntimeException(e);
		}
	}
}
