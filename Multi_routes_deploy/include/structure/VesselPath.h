#ifndef _VESSELPATH_H_
#define _VESSELPATH_H_

#include <vector>
#include "arc.h"

namespace fleetdeployment
{
    class  VesselPath;

}
class  fleetdeployment::VesselPath : public Path {
private:
    int routeID;            // Route ID
    int rotation;

public:
    // Default constructor
    VesselPath() {
        PathID = 0;
        routeID = 0;
        rotation = 0;
        num_arcs = 0;
        origin_time = 0;
        destination_time = 0;
        path_time = 0;
        arcIDs = std::vector<int>();
        arcs = std::vector<Arc>();
    }


    // Getter for routes
    [[nodiscard]] inline const int GetRouteID() const {
        return routeID;
    }

    // Setter for routes
    inline void SetRouteID(int newRoutes) {
        routeID = newRoutes;
    }

    [[nodiscard]] inline  const int GetRotation() {
        return rotation;
    }
    inline void SetRotation(int _rotation) {
        rotation = _rotation;
    }

    inline void NumArcPlusOne() {
        num_arcs += 1;
    }

    inline void printPath()
    {
        std::cout << PathID << ": "
            << routeID << '\t'
            << rotation << '\t'
            << num_arcs << '\t';
        for (auto y : arcIDs)
        {
            std::cout << y;
            if (y != arcIDs.back())
                std::cout << ",";
        }
        std::cout << '\t';
        for (auto& y : arcs)
        {
            if (y == arcs.front())
                std::cout << y.GetTail().GetNodeID();
            std::cout << "¡ª" << y.GetHead().GetNodeID();
        }
        std::cout << '\t';
        std::cout << origin_time << "\t";
        std::cout << destination_time;
        std::cout << std::endl;
    }

};

#endif // _VESSELPATH_H_
