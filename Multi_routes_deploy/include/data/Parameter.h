#ifndef PARAMETER_H
#define PARAMETER_H

namespace fleetdeployment
{

    class  Parameter {
    private:
        double uncertainDegree;
        double rentalCost;
        int turnOverTime;
        std::vector<int> TravellingArcsSet;
        std::vector<int> transhipmentArcsSet;
        std::vector<int> timePointSet;
        std::vector<int> vesselRouteSet;
        std::vector<int> vesselSet;
        std::vector<int> vesselPathSet;
        std::vector<int> PathSet;
        std::vector<int> initialEmptyContainer;
        std::vector<int> demandRequestSet;
        std::vector<int> turnOverTimeSet;
        std::vector<int> vesselCapacity;
        std::vector<int> TravelTimeOnPath;
        std::vector<std::vector<int>> arcAndVesselPath;
        std::vector<std::vector<int>> arcAndPath;
        std::vector<std::vector<int>> shipRouteAndVesselPath;
        std::vector<std::vector<int>> vesselTypeAndShipRoute;

        std::vector<std::string> portSet;
        std::vector<std::string> originOfDemand;
        std::vector<std::string> destinationOfDemand;
        std::vector<double> demand;

        std::vector<double> vesselOperationCost;
        std::vector<double> penaltyCostForDemand;
        std::vector<double> ladenDemurrageCost;
        std::vector<double> emptyDemurrageCost;
        std::vector<double> ladenPathDemurrageCost;
        std::vector<double> emptyPathDemurrageCost;
        std::vector<double> ladenPathCost;
        std::vector<double> emptyPathCost;
        std::vector<double> maximumDemandVariation;

    public:
        // Constructor
        Parameter() {  };

        // Get uncertain degree
        inline const double GetUncertainDegree() const {
            return uncertainDegree;
        }
        // Set uncertain degree
        inline void SetUncertainDegree(double uncertainDegree) {
            this->uncertainDegree = uncertainDegree;
        }

        // Get rental cost
        inline const double GetRentalCost() const {
            return rentalCost;
        }
        // Set rental cost
        inline void SetRentalCost(double rentalCost) {
            this->rentalCost = rentalCost;
        }
        // Change rental cost
        inline void changeRentalCost(double rentalCostcoeff) {
            this->rentalCost = this->rentalCost * rentalCostcoeff;
        }

        // Get TravellingArcsSet
        inline const std::vector<int>& GetTravelArcsSet() const {
            return TravellingArcsSet;
        }
        // Set TravellingArcsSet
        inline void SetTravellingArcsSet(const std::vector<int>& TravellingArcsSet) {
            this->TravellingArcsSet = TravellingArcsSet;
        }

        // Get TranshipmentArcsSet
        inline const std::vector<int>& GetTranshipmentArcsSet() const {
            return transhipmentArcsSet;
        }
        // Set TranshipmentArcsSet
        inline void SetTranshipmentArcsSet(const std::vector<int>& transhipmentArcsSet) {
            this->transhipmentArcsSet = transhipmentArcsSet;
        }

        // Get TimePointSet
        inline const std::vector<int>& GetTimePointSet() const {
            return timePointSet;
        }
        // Set TimePointSet
        inline void SetTimePointSet(const std::vector<int>& timePointSet) {
            this->timePointSet = timePointSet;
        }

        // Get VesselRouteSet
        inline const std::vector<int>& GetVesselRouteSet()  const {
            return vesselRouteSet;
        }
        // Set VesselRouteSet
        inline void SetVesselRouteSet(const std::vector<int>& vesselRouteSet) {
            this->vesselRouteSet = vesselRouteSet;
        }

        // Get VesselSet
        inline const std::vector<int>& GetVesselSet() const {
            return vesselSet;
        }
        // Set VesselSet
        inline void SetVesselSet(const std::vector<int>& vesselSet) {
            this->vesselSet = vesselSet;
        }

        // Get VesselPathSet
        inline const std::vector<int>& GetVesselPathSet() const {
            return vesselPathSet;
        }
        // Set VesselPathSet
        inline void SetVesselPathSet(const std::vector<int>& vesselPathSet) {
            this->vesselPathSet = vesselPathSet;
        }

        // Get PathSet
        inline const std::vector<int>& GetPathSet() const {
            return PathSet;
        }
        // Set PathSet
        inline void SetPathSet(const std::vector<int>& PathSet) {
            this->PathSet = PathSet;
        }

        // Get InitialEmptyContainer
        inline const std::vector<int>& GetInitialEmptyContainer() const {
            return initialEmptyContainer;
        }
        // Set InitialEmptyContainer
        inline void SetInitialEmptyContainer(const std::vector<int>& initialEmptyContainer) {
            this->initialEmptyContainer = initialEmptyContainer;
        }

        // Get DemandRequestSet
        inline const std::vector<int>& GetDemandRequestSet() const {
            return demandRequestSet;
        }
        // Set DemandRequestSet
        inline void SetDemandRequestSet(const std::vector<int>& demandRequestSet) {
            this->demandRequestSet = demandRequestSet;
        }

        // Get TurnOverTime
        inline const std::vector<int>& GetTurnOverTimeSet() const {
            return turnOverTimeSet;
        }
        inline const int GetTurnoverTime() const {
            return turnOverTime;
        }
        // Set TurnOverTime
        inline void SetTurnOverTimeSet(const std::vector<int>& turnOverTimeSet) {
            this->turnOverTimeSet = turnOverTimeSet;
        }
        // Set TurnOverTime
        inline void SetTurnOverTime(int turnOverTime) {
            this->turnOverTimeSet = std::vector<int>(size(portSet), turnOverTime);
        }

        // Get VesselCapacity
        inline const std::vector<int>& GetVesselCapacity() const {
            return vesselCapacity;
        }
        // Set VesselCapacity
        inline void SetVesselCapacity(const std::vector<int>& vesselCapacity) {
            this->vesselCapacity = vesselCapacity;
        }

        // Get TravelTimeOnPath
        inline const std::vector<int>& GetTravelTimeOnPath() const {
            return TravelTimeOnPath;
        }
        // Set TravelTimeOnPath
        inline void SetTravelTimeOnPath(const std::vector<int>& TravelTimeOnPath) {
            this->TravelTimeOnPath = TravelTimeOnPath;
        }

        // Get ArcAndVesselPath
        inline const std::vector<std::vector<int>>& GetArcAndVesselPath() const {
            return arcAndVesselPath;
        }
        // Set ArcAndVesselPath
        inline void SetArcAndVesselPath(const std::vector<std::vector<int>>& arcAndVesselPath) {
            this->arcAndVesselPath = arcAndVesselPath;
        }

        // Get ArcAndPath
        inline const std::vector<std::vector<int>>& GetArcAndPath() const {
            return arcAndPath;
        }
        // Set ArcAndPath
        inline void SetArcAndPath(const std::vector<std::vector<int>>& arcAndPath) {
            this->arcAndPath = arcAndPath;
        }

        // Get ShipRouteAndVesselPath
        inline const std::vector<std::vector<int>>& GetShipRouteAndVesselPath() const {
            return shipRouteAndVesselPath;
        }
        // Set ShipRouteAndVesselPath
        inline void SetShipRouteAndVesselPath(const std::vector<std::vector<int>>& shipRouteAndVesselPath) {
            this->shipRouteAndVesselPath = shipRouteAndVesselPath;
        }

        // Get VesselTypeAndShipRoute
        inline const std::vector<std::vector<int>>& GetVesselTypeAndShipRoute() const {
            return vesselTypeAndShipRoute;
        }
        // Set VesselTypeAndShipRoute
        inline void SetVesselTypeAndShipRoute(const std::vector<std::vector<int>>& vesselTypeAndShipRoute) {
            this->vesselTypeAndShipRoute = vesselTypeAndShipRoute;
        }

        // Get PortSet
        inline const std::vector<std::string>& GetPortSet() const {
            return portSet;
        }
        // Set PortSet
        inline void SetPortSet(const std::vector<std::string>& portSet) {
            this->portSet = portSet;
        }

        // Get OriginOfDemand
        inline const std::vector<std::string>& GetOriginOfDemand() const {
            return originOfDemand;
        }
        // Set OriginOfDemand
        inline void SetOriginOfDemand(const std::vector<std::string>& originOfDemand) {
            this->originOfDemand = originOfDemand;
        }

        // Get DestinationOfDemand
        inline const std::vector<std::string>& GetDestinationOfDemand() const {
            return this->destinationOfDemand;
        }
        // Set DestinationOfDemand
        inline void SetDestinationOfDemand(const std::vector<std::string>& destinationOfDemand) {
            this->destinationOfDemand = destinationOfDemand;
        }

        inline const std::vector<double>& GetDemand() const {
            return demand;
        }

        // Inline member function to Set demand
        inline void SetDemand(const std::vector<double>& demand) {
            this->demand = demand;
        }

        // Inline member function to Set vessel operation cost
        inline void SetVesselOperationCost(const std::vector<double>& vesselOperationCost) {
            this->vesselOperationCost = vesselOperationCost;
        }
        inline const std::vector<double>& GetVesselOperationCost() const {
            return vesselOperationCost;
        }

        // Inline member function to Set penalty cost for demand
        inline void SetPenaltyCostForDemand(const std::vector<double>& penaltyCostForDemand) {
            this->penaltyCostForDemand = penaltyCostForDemand;
        }
        inline const std::vector<double>& GetPenaltyCostForDemand() const {
            return penaltyCostForDemand;
        }

        // Inline member function to change penalty cost for demand
        inline void changePenaltyCostForDemand(double penaltyCostCoeff) {
            // Adjust each element in the penaltyCostForDemand vector
            std::vector<double> newPenaltyCost = std::vector<double>(size(demand));
            for (int i = 0; i < size(demand); i++) {
                newPenaltyCost[i] = penaltyCostForDemand[i] * penaltyCostCoeff;
            }
            SetPenaltyCostForDemand(newPenaltyCost);
        }

        // Inline member function to Set laden demurrage cost
        inline void SetLadenDemurrageCost(const std::vector<double>& ladenDemurrageCost) {
            this->ladenDemurrageCost = ladenDemurrageCost;
        }
        inline const std::vector<double>& GetLadenDemurrageCost() const {
            return ladenDemurrageCost;
        }

        // Inline member function to Set empty demurrage cost
        inline void SetEmptyDemurrageCost(const std::vector<double>& emptyDemurrageCost) {
            this->emptyDemurrageCost = emptyDemurrageCost;
        }
        inline const std::vector<double>& GetEmptyDemurrageCost() const {
            return emptyDemurrageCost;
        }

        // Inline member function to Set laden path demurrage cost
        inline void SetLadenPathDemurrageCost(const std::vector<double>& ladenPathDemurrageCost) {
            this->ladenPathDemurrageCost = ladenPathDemurrageCost;
        }
        inline const std::vector<double>& GetLadenPathDemurrageCost() {
            return ladenPathDemurrageCost;
        }

        // Inline member function to Set empty path demurrage cost
        inline void SetEmptyPathDemurrageCost(const std::vector<double>& emptyPathDemurrageCost) {
            this->emptyPathDemurrageCost = emptyPathDemurrageCost;
        }
        inline const std::vector<double>& GetEmptyPathDemurrageCost() {
            return emptyPathDemurrageCost;
        }

        // Inline member function to Set laden path cost
        inline void SetLadenPathCost(const std::vector<double>& ladenPathCost) {
            this->ladenPathCost = ladenPathCost;
        }
        inline const std::vector<double>& GetLadenPathCost() const {
            return ladenPathCost;
        }

        // Inline member function to Set empty path cost
        inline void SetEmptyPathCost(const std::vector<double>& emptyPathCost) {
            this->emptyPathCost = emptyPathCost;
        }
        inline const std::vector<double>& GetEmptyPathCost() const {
            return emptyPathCost;
        }

        inline const std::vector<double>& GetMaximumDemandVariation() const {
            return maximumDemandVariation;
        }

        // Inline member function to Set maximum demand variation
        inline void SetMaximumDemandVariation(const std::vector<double>& maximumDemandVariation) {
            this->maximumDemandVariation = maximumDemandVariation;
        }

        // Inline member function to change maximum demand variation
        inline void changeMaximunDemandVariation(double coeff) {
            // Adjust each element in the maximumDemandVariation vector
            for (int i = 0; i < maximumDemandVariation.size(); ++i) {
                maximumDemandVariation[i] = maximumDemandVariation[i] * coeff;
            }
        }

        // Functions to calculate total capacity
        inline double GetTotalCapacityMax() {
            double total_capacity = 0;
            // r \in R
            for (int r = 0; r < size(GetVesselRouteSet()); r++)
            {
                // w \in \Omega
                // r(w) = r : p_.GetShipRouteAndVesselPath()[r][w] == 1
                for (int w = 0; w < size(GetVesselPathSet()); w++)
                {
                    double max_capacity = 0;
                    // h \in H_r
                    // r(h) = r : p_.GetVesselTypeAndShippingRoute()[h][r] == 1
                    for (int h = 0; h < size(GetVesselSet()); h++)
                    {
                        if (GetVesselTypeAndShipRoute()[h][r] * GetShipRouteAndVesselPath()[r][w] != 0) {
                            if (GetVesselCapacity()[h] > max_capacity) {
                                max_capacity = GetVesselCapacity()[h];
                            }
                        }
                    }
                    total_capacity += max_capacity;
                }
            }

            return total_capacity;
        }
        inline double GetTotalCapacityMin() const {
            double total_capacity = 0;
            // r \in R
            for (int r = 0; r < size(GetVesselRouteSet()); r++)
            {
                // w \in \Omega
                // r(w) = r : p_.GetShipRouteAndVesselPath()[r][w] == 1
                for (int w = 0; w < size(vesselPathSet); w++)
                {
                    double min_capacity = 100000000000;
                    // h \in H_r
                    // r(h) = r : p_.GetVesselTypeAndShippingRoute()[h][r] == 1
                    for (int h = 0; h < size(vesselSet); h++)
                    {
                        if (GetVesselTypeAndShipRoute()[h][r] * GetShipRouteAndVesselPath()[r][w] != 0) {
                            if (GetVesselCapacity()[h] < min_capacity) {
                                min_capacity = GetVesselCapacity()[h];
                            }
                        }
                    }
                    total_capacity += min_capacity;
                }
            }

            return total_capacity;
        }

        // Function to calculate total demand
        inline double GetTotalDemand() const {
            double total_demand = 0;
            for (int i = 0; i < size(demand); i++) {
                total_demand += demand[i] + maximumDemandVariation[i];
            }
            return total_demand;
        }
    };



}
#endif // PARAMETER_H

