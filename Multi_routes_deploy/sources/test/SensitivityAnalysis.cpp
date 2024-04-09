#include "SensitivityAnalysis.h"

namespace fleetdeployment {

    void SensitivityAnalysis::VaryUncertainDegree(InputData in) {
        std::cout << "=========Varying UncertainDegree from 0 to 0.20==========" << std::endl;

        if (WhetherOutputLog) {
            fileWriter << "=========Varying UncertainDegree from 0 to 0.20==========" << std::endl;
            fileWriter << std::endl;
        }

        double UD;
        for (int i = 0; i < uncertainDegreeSet.size(); i++) {

            std::cout << "uncertainDegreeSet = " << uncertainDegreeSet[i];

            Parameter p;
            UD = uncertainDegreeSet[i];
            new GenerateParameter(p, in, T, UD);

            CCGwithPAP cp(in, p, (int)sqrt(in.GetRequests().size()));

            std::cout << "UD"
                << '\t' << "LPC"
                << '\t' << "EPC"
                << '\t' << "LC+EC"
                << '\t' << "RC"
                << '\t' << "PC"
                << '\t' << "OC"
                << '\t' << "TC" << std::endl;
            std::cout << UD
                << "\t" << std::fixed << std::setprecision(2) << cp.GetLadenCost()
                << "\t" << std::fixed << std::setprecision(2) << cp.GetEmptyCost()
                << "\t" << std::fixed << std::setprecision(2) << cp.GetEmptyCost() + cp.GetLadenCost()
                << "\t" << std::fixed << std::setprecision(2) << cp.GetRentalCost()
                << "\t" << std::fixed << std::setprecision(2) << cp.GetPenaltyCost()
                << "\t" << std::fixed << std::setprecision(2) << cp.GetOperationCost()
                << "\t" << std::fixed << std::setprecision(2) << cp.GetTotalCost() << std::endl;
            std::cout << "=================================" << std::endl;

            if (WhetherOutputLog) {
                fileWriter << "UncertainDegree"
                    << '\t' << "LadenPathCost"
                    << '\t' << "EmptyPathCost"
                    << '\t' << "LC+EC"
                    << '\t' << "RentalCost"
                    << '\t' << "PenaltyCost"
                    << '\t' << "OperationCost"
                    << '\t' << "TotalCost" << '\n';
                fileWriter << UD
                    << "\t" << std::fixed << std::setprecision(2) << cp.GetLadenCost()
                    << "\t" << std::fixed << std::setprecision(2) << cp.GetEmptyCost()
                    << "\t" << std::fixed << std::setprecision(2) << cp.GetEmptyCost() + cp.GetLadenCost()
                    << "\t" << std::fixed << std::setprecision(2) << cp.GetRentalCost()
                    << "\t" << std::fixed << std::setprecision(2) << cp.GetPenaltyCost()
                    << "\t" << std::fixed << std::setprecision(2) << cp.GetOperationCost()
                    << "\t" << std::fixed << std::setprecision(2) << cp.GetTotalCost() << std::endl;
            }
        }
    }

    void SensitivityAnalysis::VaryLoadAndDischargeCost(InputData in) {
        std::cout << "=========Varying Unit L&D&T Cost========" << std::endl;

        double LDTCoeff;
        for (int i = 0; i < containerPathCostSet.size(); i++) {

            std::cout << "Unit Path Cost = " << containerPathCostSet[i];

            Parameter p;
            GenerateParameter(p, in, T, uncertainDegree);
            LDTCoeff = containerPathCostSet[i];
            std::vector<double> ladenPathCost = p.GetLadenPathCost();
            std::vector<double> emptyPathCost = p.GetEmptyPathCost();
            for (int j = 0; j < p.GetPathSet().size(); j++) {
                ladenPathCost[j] = in.GetPathSet()[j].GetPathCost() * LDTCoeff + p.GetLadenPathDemurrageCost()[j];
                emptyPathCost[j] = in.GetPathSet()[j].GetPathCost() * 0.5 * LDTCoeff + p.GetEmptyPathDemurrageCost()[j];
            }
            p.SetLadenPathCost(ladenPathCost);
            p.SetEmptyPathCost(emptyPathCost);

            CCGwithPAP cp(in, p, (int)sqrt(in.GetRequests().size()));

            std::cout << "DemurrageCostCoeff"
                << '\t' << "LPC"
                << '\t' << "EPC"
                << '\t' << "LC+EC"
                << '\t' << "RC"
                << '\t' << "PC"
                << '\t' << "OC"
                << '\t' << "TC" << std::endl;
            std::cout << LDTCoeff
                << "\t" << std::fixed << std::setprecision(2) << cp.GetLadenCost()
                << "\t" << std::fixed << std::setprecision(2) << cp.GetEmptyCost()
                << "\t" << std::fixed << std::setprecision(2) << cp.GetEmptyCost() + cp.GetLadenCost()
                << "\t" << std::fixed << std::setprecision(2) << cp.GetRentalCost()
                << "\t" << std::fixed << std::setprecision(2) << cp.GetPenaltyCost()
                << "\t" << std::fixed << std::setprecision(2) << cp.GetOperationCost()
                << "\t" << std::fixed << std::setprecision(2) << cp.GetTotalCost() << std::endl;
            std::cout << "=================================" << std::endl;
        }
    }

    void SensitivityAnalysis::VaryRentalCost(InputData in) {
        std::cout << "=========Varying Unit Container Rental Cost (0.5~1.5)x20========" << std::endl;

        if (WhetherOutputLog) {
            fileWriter << "=========Varying Unit Container Rental Cost (0.5~1.5)x20========" << std::endl;
            fileWriter << std::endl;
        }


        double RentalCostCoeff;
        for (int i = 0; i < rentalContainerCostSet.size(); i++) {

            std::cout << "RentalCostCoeff = " << rentalContainerCostSet[i] << std::endl;

            Parameter p;
            new GenerateParameter(p, in, T, uncertainDegree);
            RentalCostCoeff = rentalContainerCostSet[i];
            p.changeRentalCost(RentalCostCoeff);

            CCGwithPAP cp(in, p, (int)sqrt(in.GetRequests().size()));

            std::cout << "RentalCostCoeff"
                << '\t' << "LPC"
                << '\t' << "EPC"
                << '\t' << "LC+EC"
                << '\t' << "RC"
                << '\t' << "PC"
                << '\t' << "OC"
                << '\t' << "TC" << std::endl;
            std::cout << RentalCostCoeff
                << "\t" << std::fixed << std::setprecision(2) << cp.GetLadenCost()
                << "\t" << std::fixed << std::setprecision(2) << cp.GetEmptyCost()
                << "\t" << std::fixed << std::setprecision(2) << cp.GetEmptyCost() + cp.GetLadenCost()
                << "\t" << std::fixed << std::setprecision(2) << cp.GetRentalCost()
                << "\t" << std::fixed << std::setprecision(2) << cp.GetPenaltyCost()
                << "\t" << std::fixed << std::setprecision(2) << cp.GetOperationCost()
                << "\t" << std::fixed << std::setprecision(2) << cp.GetTotalCost() << std::endl;
            std::cout << "=================================" << std::endl;


            if (WhetherOutputLog) {
                fileWriter << "RentalCostCoeff"
                    << '\t' << "LPC"
                    << '\t' << "EPC"
                    << '\t' << "LC+EC"
                    << '\t' << "RC"
                    << '\t' << "PC"
                    << '\t' << "OC"
                    << '\t' << "TC" << std::endl;
                fileWriter << RentalCostCoeff
                    << "\t" << std::fixed << std::setprecision(2) << cp.GetLadenCost()
                    << "\t" << std::fixed << std::setprecision(2) << cp.GetEmptyCost()
                    << "\t" << std::fixed << std::setprecision(2) << cp.GetEmptyCost() + cp.GetLadenCost()
                    << "\t" << std::fixed << std::setprecision(2) << cp.GetRentalCost()
                    << "\t" << std::fixed << std::setprecision(2) << cp.GetPenaltyCost()
                    << "\t" << std::fixed << std::setprecision(2) << cp.GetOperationCost()
                    << "\t" << std::fixed << std::setprecision(2) << cp.GetTotalCost() << std::endl;
            }

        }
    }

    void SensitivityAnalysis::VaryPenaltyCost(InputData in) {
        std::cout << "=========Varying Unit Demand Penalty Cost =========" << std::endl;

        if (WhetherOutputLog) {
            fileWriter << "=========Varying Unit Demand Penalty Cost =========" << std::endl;
        }

        double PenaltyCostCoeff;
        for (int i = 0; i < penaltyCostSet.size(); i++) {
            std::cout << "PenaltyCostCoeff = " << penaltyCostSet[i];

            Parameter p;
            new GenerateParameter(p, in, T, uncertainDegree);
            PenaltyCostCoeff = penaltyCostSet[i];
            p.changePenaltyCostForDemand(PenaltyCostCoeff);

            CCGwithPAP cp(in, p, (int)sqrt(in.GetRequests().size()));

            std::cout << "PenaltyCostCoeff"
                << '\t' << "LPC"
                << '\t' << "EPC"
                << '\t' << "RC"
                << '\t' << "PC"
                << '\t' << "OC"
                << '\t' << "TC";
            std::cout << PenaltyCostCoeff
                << "\t" << std::fixed << std::setprecision(2) << cp.GetLadenCost()
                << "\t" << std::fixed << std::setprecision(2) << cp.GetEmptyCost()
                << "\t" << std::fixed << std::setprecision(2) << cp.GetEmptyCost() + cp.GetLadenCost()
                << "\t" << std::fixed << std::setprecision(2) << cp.GetRentalCost()
                << "\t" << std::fixed << std::setprecision(2) << cp.GetPenaltyCost()
                << "\t" << std::fixed << std::setprecision(2) << cp.GetOperationCost()
                << "\t" << std::fixed << std::setprecision(2) << cp.GetTotalCost() << std::endl;
            std::cout << "=================================" << std::endl;
        }
    }

    void SensitivityAnalysis::VaryTurnOverTime(InputData in) {
        std::cout << "=========Varying TurnOverTime (0~28) =========" << std::endl;

        int turnOverTime;
        for (int i = 0; i < turnOverTimeSet.size(); i++) {
            std::cout << "********************turnOverTime = " << turnOverTimeSet[i] << "********************";

            Parameter para;
            new GenerateParameter(para, in, T, uncertainDegree);
            turnOverTime = turnOverTimeSet[i];
            para.SetTurnOverTime(turnOverTime);

            CCGwithPAP cp(in, para, (int)sqrt(in.GetRequests().size()));

            std::cout << "turnOverTime"
                << '\t' << "LPC"
                << '\t' << "EPC"
                << '\t' << "LC+EC"
                << '\t' << "RC"
                << '\t' << "PC"
                << '\t' << "OC"
                << '\t' << "TC" << std::endl;
            std::cout << turnOverTime
                << "\t" << std::fixed << std::setprecision(2) << cp.GetLadenCost()
                << "\t" << std::fixed << std::setprecision(2) << cp.GetEmptyCost()
                << "\t" << std::fixed << std::setprecision(2) << cp.GetEmptyCost() + cp.GetLadenCost()
                << "\t" << std::fixed << std::setprecision(2) << cp.GetRentalCost()
                << "\t" << std::fixed << std::setprecision(2) << cp.GetPenaltyCost()
                << "\t" << std::fixed << std::setprecision(2) << cp.GetOperationCost()
                << "\t" << std::fixed << std::setprecision(2) << cp.GetTotalCost() << std::endl;
            std::cout << "=================================" << std::endl;
        }
    }

    void SensitivityAnalysis::VaryInitialContainers(InputData& in) {
        std::cout << "=========Varying initialContainers (0~28) =========\n";

        int initialContainers;
        std::random_device rd;
        std::mt19937 gen(rd());
        std::uniform_real_distribution<> dis(0.8, 1.0);

        for (int i = 0; i < initialContainerSet.size(); i++) {
            std::cout << "initialContainers = " << initialContainerSet[i] << "\n";

            Parameter p;
            GenerateParameter(p, in, T, uncertainDegree);

            // Reset initial empty containers
            initialContainers = initialContainerSet[i];
            std::vector<int> initialEmptyContainer(in.GetPortSet().size(), 0);
            int totalOwnedEmptyContainers = 0;

            for (const Port& pp : in.GetPortSet()) {
                double alpha = dis(gen);  // Generate random alpha between 0.8 and 1.0
                for (int ii = 0; ii < in.GetRequests().size(); ii++) {
                    if (pp.GetPort() == in.GetRequests()[ii].GetOriginPort() &&
                        in.GetRequests()[ii].GetW_i_Earliest() < initialContainers) {
                        initialEmptyContainer[pp.GetID()] += static_cast<int>(alpha * p.GetDemand()[ii]);
                        totalOwnedEmptyContainers += initialEmptyContainer[pp.GetID()];
                    }
                }
            }

            std::cout << "Total Initial Owned Empty Containers = " << totalOwnedEmptyContainers << "\n";
            p.SetInitialEmptyContainer(initialEmptyContainer);

            CCGwithPAP cp(in, p, static_cast<int>(std::sqrt(in.GetRequests().size())));

            std::cout << "initialContainers\tLPC\tEPC\tRC\tPC\tOC\tTC\n";
            std::cout << initialContainers
                << "\t" << std::fixed << std::setprecision(2) << cp.GetLadenCost()
                << "\t" << std::fixed << std::setprecision(2) << cp.GetEmptyCost()
                << "\t" << std::fixed << std::setprecision(2) << (cp.GetEmptyCost() + cp.GetLadenCost())
                << "\t" << std::fixed << std::setprecision(2) << cp.GetRentalCost()
                << "\t" << std::fixed << std::setprecision(2) << cp.GetPenaltyCost()
                << "\t" << std::fixed << std::setprecision(2) << cp.GetOperationCost()
                << "\t" << std::fixed << std::setprecision(2) << cp.GetTotalCost() << "\n";
            std::cout << "=================================\n";
        }
    }


}