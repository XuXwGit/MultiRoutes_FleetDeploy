#ifndef _VESSEL_H_
#define _VESSEL_H_

namespace fleetdeployment
{
    class  Vessel;
}

class  fleetdeployment::Vessel {
private:
    int vessel_ID;           // Unique IDentifier for the vessel
    int capacity;      // Maximum cargo capacity of the vessel
    double operating_cost;   // Cost associated with the vessel
    int current_route_ID;       // Current route of the vessel
    int max_vessels;         // Maximum number of vessels of this type

public:
    // Default constructor
    Vessel() {
        vessel_ID = 0;
        capacity = 0;
        operating_cost = 0.0;
        current_route_ID = 0;
        max_vessels = 0;
    }

    // Getter for vessel_ID
    int GetVesselID() const {
        return vessel_ID;
    }

    // Setter for vessel_ID
    void SetVesselID(int newVesselID) {
        vessel_ID = newVesselID;
    }

    // Getter for capacity
    int GetCapacity() const {
        return capacity;
    }

    // Setter for capacity
    void SetCapacity(int newCapacity) {
        capacity = newCapacity;
    }

    // Getter for operating_cost
    double GetOperatingCost() const {
        return operating_cost;
    }

    // Setter for operating_cost
    void SetOperatingCost(double newOperatingCost) {
        operating_cost = newOperatingCost;
    }

    // Getter for current_route
    int GetCurrentRouteID() const {
        return current_route_ID;
    }

    // Setter for current_route
    void SetCurrentRouteID(int newCurrentRoute) {
        current_route_ID = newCurrentRoute;
    }

    // Getter for max_vessels
    int GetMaxVessels() const {
        return max_vessels;
    }

    // Setter for max_vessels
    void SetMaxVessels(int newMaxVessels) {
        max_vessels = newMaxVessels;
    }
};

#endif // _VESSEL_H_
