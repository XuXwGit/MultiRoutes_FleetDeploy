#include "BD.h"
#include "DualSubProblem.h"
#include "MasterProblem.h"

namespace fleetdeployment
{

	void BD::frame() {
		std::ofstream fileWriter;
		fileWriter.open("BD" + std::to_string(p_.GetTimePointSet().size()) + ".txt", std::ios::app);

		time_t start = time(0);

		SetUpperBound(1e12);
		SetLowerBound(-1e12);

		std::vector<Scenario> sce;
		int max_iteration = 50;
		std::vector<std::vector<int>> vvv(p_.GetVesselSet().size(), std::vector<int>(p_.GetVesselRouteSet().size()));
		std::vector<double> upper(max_iteration + 1, GetUpperBound());
		std::vector<double> lower(max_iteration + 1, GetLowerBound());
		int iteration = 0;
		std::vector<int> worseCaseSet(p_.GetDemand().size());
		double gap = std::numeric_limits<double>::max();

		if (Setting::WhetherPrintProcess) {
			std::cout << "==========BD : ===========" << std::endl;
		}
		time_t time0 = time(0);
		DualSubProblem dsp(in_, p_, tau);
		MasterProblem mp(in_, p_);

		if (WhetherPrintProcess || WhetherPrintIteration) {
			double buildModelTime = difftime(time(0), time0);  // 转换为秒
			std::cout << "BuildModelTime = " << std::fixed << std::setprecision(2) << buildModelTime << std::endl;
			std::cout << "k" << "\t\t"
				<< "UB" << "\t\t"
				<< "LB" << "\t\t"
				<< "DSP-SolveTime" << "\t\t"
				<< "MP-SolveTime" << "\t\t"
				<< "Total Time" << "\t\t"
				<< "DSP-Status" << "\t\t"
				<< "MP-Status" << std::endl;
			std::cout << iteration << "\t\t"
				<< std::fixed << std::setprecision(2) << upper[iteration] << "\t\t"
				<< std::fixed << std::setprecision(2) << lower[iteration] << std::endl;
		}

		if (WhetherPrintFileLog) {
			fileWriter << "k" << "\t\t"
				<< "UB" << "\t\t"
				<< "LB" << "\t\t"
				<< "DSP-SolveTime" << "\t\t"
				<< "MP-SolveTime" << "\t\t"
				<< "Total Time" << "\t\t"
				<< "DSP-Status" << "\t\t"
				<< "MP-Status" << std::endl;
			fileWriter << iteration << "\t\t"
				<< std::fixed << std::setprecision(2) << upper[iteration] << "\t\t"
				<< std::fixed << std::setprecision(2) << lower[iteration] << std::endl;
		}

		// add the initial scene to make the MP feasible
		Initialize(sce);
		mp.addScene(sce[iteration]);
		int flag = 0;

		time_t start0 = time(0);
		while (upperBound - lowerBound > MIPGapLimit
			&& gap > MIPGapLimit
			&& flag == 0
			&& iteration < max_iteration)
		{
			time_t start1 = time(0);
			mp.SolveModel();
			time_t end1 = time(0);

			// check if the mp-solution changed
			double temp = 0;
			for (int r = 0; r < p_.GetVesselRouteSet().size(); r++)
			{
				for (int h = 0; h < p_.GetVesselSet().size(); h++)
				{
					temp = temp + abs(mp.GetVVarValue()[h][r] - vvv[h][r]);
					vvv[h][r] = mp.GetVVarValue()[h][r];
				}
			}
			// master solution has no change
			if (temp < 0.5)
			{
				flag = 1;
			}

			// LB = max{LB , MP.Objective}
			// LB = MP.Objective = MP.OperationCost + Eta
			if (mp.GetObjVal() > lowerBound) {
				SetLowerBound(mp.GetObjVal());
			}

			dsp.changeVVarCoefficients(mp.GetVVarValue());
			time_t start2 = time(0);
			dsp.SolveModel();
			time_t end2 = time(0);

			//  the SP is optimal :  add optimality cut
			if (dsp.GetSolveStatus() == IloCplex::Status::Optimal
				|| dsp.GetSolveStatus() == IloCplex::Status::Feasible)
			{
				//  std::cout<<"DSP is Optimal");

				//  update UB : UB = min{UB, MP.OperationCost + SP.Objective}
				if (dsp.GetObjVal() + mp.GetOperationCost() < upperBound)
				{
					SetUpperBound(dsp.GetObjVal() + mp.GetOperationCost());
				}

				// add optimality cut
				mp.addOptimalityCut(dsp.GetConstantItem(), dsp.GetBetaValue());

				// add the worst scene (extreme point) to scene Set
				Scenario scene = dsp.GetScene();
				sce.push_back(scene);
				std::vector<double> request = scene.GetRequest();
				for (int i = 0; i < p_.GetDemand().size(); i++) {
					request[i] = dsp.GetUValue()[i];
					if (request[i] != 0)
						worseCaseSet[iteration + 1] = i;
				}
			}
			// the SP is unbounded : add feasibility cut
			else if (dsp.GetSolveStatus() == IloCplex::Status::Unbounded)
			{
				std::cout << "DSP is Unbounded";
				// ! here beta is extreme ray !
				mp.addFeasibilityCut(dsp.GetConstantItem(), dsp.GetBetaValue());
			}

			else if (dsp.GetSolveStatus() == IloCplex::Status::Infeasible)
			{
				std::cout << "DSP is InFeasible";
			}

			else {

				std::cout << "DSP is error";

			}

			iteration = iteration + 1;
			upper[iteration] = upperBound;
			lower[iteration] = lowerBound;
			gap = upper[iteration] - lower[iteration];

			if (WhetherPrintProcess || WhetherPrintIteration) {
				time_t end1 = time(0);  // 结束时间 1
				time_t end2 = time(0);  // 结束时间 2

				std::cout << iteration << "\t\t"
					<< std::fixed << std::setprecision(2) << upper[iteration] << "\t\t"
					<< std::fixed << std::setprecision(2) << lower[iteration] << "\t\t"
					<< std::fixed << std::setprecision(2) << difftime(end2, start2) << "\t\t"
					<< std::fixed << std::setprecision(2) << difftime(end1, start1) << "\t\t"
					<< std::fixed << std::setprecision(2) << difftime(time(0), start0) << "\t\t"
					<< dsp.GetSolveStatusString() << "(" << std::fixed << std::setprecision(4) << dsp.GetMipGap() << ")\t\t"
					<< mp.GetSolveStatusString() << "(" << std::fixed << std::setprecision(4) << mp.GetMipGap() << ")\n";
			}

			if (WhetherOutputLog) {
				fileWriter << iteration << "\t\t"
					<< std::fixed << std::setprecision(2) << upper[iteration] << "\t\t"
					<< std::fixed << std::setprecision(2) << lower[iteration] << "\t\t"
					<< std::fixed << std::setprecision(2) << difftime(end2, start2) << "\t\t"
					<< std::fixed << std::setprecision(2) << difftime(end1, start1) << "\t\t"
					<< std::fixed << std::setprecision(2) << difftime(time(0), start0) << "\t\t"
					<< dsp.GetSolveStatusString() << "(" << std::fixed << std::setprecision(4) << dsp.GetMipGap() << ")\t\t"
					<< mp.GetSolveStatusString() << "(" << std::fixed << std::setprecision(4) << mp.GetMipGap() << ")\n";
			}

		}

		mp.end();
		dsp.end();

		SetVValue(mp.GetVVarValue());
		SetIter(iteration);
		SetObj(upperBound);

		SetGap((upperBound - lowerBound) / lowerBound);

		if (WhetherPrintProcess) {
			PrintSolution();
		}

		time_t end = time(0);
		SetSolveTime(difftime(end, start));

		if (WhetherPrintProcess || WhetherPrintSolveTime) {
			std::cout << "BD SolveTime = " << difftime(end, start) << " seconds\n";
			fileWriter << "BD SolveTime = " << difftime(end, start) << " seconds\n";
			std::cout << "=================================\n";
		}

		fileWriter.close();
	}

	void BD::Initialize(std::vector<Scenario>& sce)
	{
		std::vector<double> sss(p_.GetDemand().size(), 0.0);
		for (int i = 0; i < tau; i++) {
			sss[i] = 1.0;
		}

		Scenario sp;
		sp.SetRequest(sss);
		sce.push_back(sp);
	}

	void BD::PrintSolution() {
		std::cout << "Vessel Decision vVar : ";
		for (int r = 0; r < p_.GetVesselRouteSet().size(); r++)
		{
			std::cout << p_.GetVesselRouteSet()[r] << ":\t";
			for (int h = 0; h < p_.GetVesselSet().size(); h++)
			{
				if (vValue[h][r] != 0)
				{
					std::cout << "(" << p_.GetVesselSet()[h] << ")\t";
				}
			}
		}
		std::cout << std::endl;
	}


}