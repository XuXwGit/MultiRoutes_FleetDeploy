#include "CCG.h"

namespace fleetdeployment
{

	void CCG::frame() {
		std::string filename = "CCG" + std::to_string(p_.GetTimePointSet().size()) + ".txt";

		std::ofstream fileWriter(filename, std::ios::app);

		if (!fileWriter.is_open()) {
			std::cerr << "Error: Unable to open file " << filename << std::endl;
		}

		time_t start = time(0);

		SetUpperBound(1e12);
		SetLowerBound(-1e12);

		std::vector<Scenario> sce;

		int max_iteration = 50;
		std::vector<std::vector<int>> vvv(p_.GetVesselSet().size(), std::vector<int>(p_.GetVesselRouteSet().size()));
		std::vector<double> upper(max_iteration + 1);
		std::vector<double> lower(max_iteration + 1);
		upper[0] = upperBound;
		lower[0] = lowerBound;
		int iteration = 0;
		double gap = std::numeric_limits<double>::max();

		time_t time0 = time(0);
		DualSubProblem dsp(in_, p_, tau);
		MasterProblem mp(in_, p_);

		if (WhetherPrintProcess || WhetherPrintIteration) {
			std::cout << "=========C&CG : ========" << std::endl;
			std::cout << "BuildModelTime = "
				<< std::fixed << std::setprecision(2) << time(0) - time0 << std::endl;
			std::cout << "k" << "\t\t"
				<< "UB" << "\t\t"
				<< "LB" << "\t\t"
				<< "DSP-SolveTime" << "\t\t"
				<< "MP-SolveTime" << "\t\t"
				<< "Total Time" << "\t\t"
				<< "DSP-Status" << "\t\t"
				<< "MP-Status";
			std::cout << iteration << "\t\t"
				<< std::fixed << std::setprecision(2) << upper[iteration] << "\t\t"
				<< std::fixed << std::setprecision(2) << lower[iteration] << std::endl;
		}
		if (WhetherOutputLog) {
			fileWriter << "k" << "\t\t"
				<< "UB" << "\t\t"
				<< "LB" << "\t\t"
				<< "DSP-SolveTime" << "\t\t"
				<< "MP-SolveTime" << "\t\t"
				<< "Total Time" << "\t\t"
				<< "DSP-Status" << "\t\t"
				<< "MP-Status" << std::endl;
			fileWriter << iteration << "\t\t"
				<< std::fixed << std::setprecision(2) << upper[iteration] << "\t\t\t"
				<< std::fixed << std::setprecision(2) << lower[iteration] << std::endl;
		}

		Initialize(sce);
		int flag = 0;
		time_t start0 = time(0);
		while (upperBound - lowerBound > MIPGapLimit
			&& gap > MIPGapLimit
			&& flag == 0
			&& iteration < max_iteration)
		{
			// add new scene to Master Problem

			mp.addScene(sce[iteration]);

			time_t start1 = time(0);
			mp.SolveModel();
			time_t end1 = time(0);

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

			// MP >> the primal problem after relax some constraints
			// So : LP = MP - Obj
			if (mp.GetObjVal() > lowerBound)
			{
				SetLowerBound(mp.GetObjVal());
			}

			// solve dual sub problem
			dsp.changeVVarCoefficients(mp.GetVVarValue());
			time_t start2 = time(0);
			dsp.SolveModel();
			time_t end2 = time(0);

			//  update UB : UB = min{UB, MP.OperationCost + SP.Objective}
			if (dsp.GetObjVal() + mp.GetOperationCost() < upperBound)
			{
				SetUpperBound(dsp.GetObjVal() + mp.GetOperationCost());
			}

			// add worse case to scene Set
			sce.push_back(dsp.GetScene());

			iteration = iteration + 1;
			upper[iteration] = upperBound;
			lower[iteration] = lowerBound;
			gap = upper[iteration] - lower[iteration];

			if (WhetherPrintProcess || WhetherPrintIteration) {
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

		Gap = (upperBound - lowerBound) / lowerBound;

		mp.end();
		dsp.end();

		time_t end = time(0);
		SetSolveTime(difftime(end, start));

		if (WhetherPrintProcess || WhetherPrintSolveTime) {
			std::cout << "CCG SolveTime = " << (difftime(end, start)) << std::endl;
			std::cout << "=================================" << std::endl;
		}

		if (WhetherOutputLog) {
			fileWriter << "CCG SolveTime = " << difftime(end, start) << std::endl;
		}
		fileWriter.close();

		SetObj(upperBound);

		SetIter(iteration);
	}
	void CCG::Initialize(std::vector<Scenario>& sce)
	{
		std::vector<double> sss(in_.GetRequests().size());
		for (int i = 0; i < tau; i++)
		{
			sss[i] = 1;
		}

		Scenario sp;
		sp.SetRequest(sss);
		sce.push_back(sp);
	}


	void CCG::PrintSolution() {
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
