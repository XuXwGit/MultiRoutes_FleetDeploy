#ifndef _PATH_H_
#define _PATH_H_

#include <string>
#include <vector>
#include <fstream>
#include <numeric>

#include "Arc.h"


namespace fleetdeployment
{
    class Path;
}

class  fleetdeployment::Path {
    protected:
        int PathID;
        int origin_time;                               // Origin time
        int destination_time;               // Arrival time at the destination
        int path_time;                                  // Path time
        std::string origin_port;                       // Origin port name
        std::string destination_port;                  // Destination port name

        std::vector<std::string> Path_port;            // Port paths
        std::vector<int> arcIDs;                      // Arc IDs
        std::vector<Arc> arcs;

        int numberOfPath;                              // Number of paths
        int num_arcs;                              // Number of arcs

        double path_cost;                              // Path cost

    public:
        // Default constructor
        Path() : PathID(-1), origin_time(-1), destination_time(-1), path_time(-1), numberOfPath(-1), num_arcs(-1), path_cost(0.0) {
		}
        virtual ~Path() {};

        inline void SetPathID(const int newID) {
			PathID = newID;
		}
        [[nodiscard]] inline const int GetPathID() const{
            return PathID;
        }

        // Getter for origin_Port
        const std::string& GetOriginPort() const {
            return origin_port;
        }

        // Setter for origin_Port
        void SetOriginPort(const std::string& new_origin_Port) {
            origin_port = new_origin_Port;
        }

        // Getter for origin_Time
        const int GetOriginTime() const {
            return origin_time;
        }
        // Setter for origin_Time
        void SetOriginTime(const int new_origin_Time) {
            origin_time = new_origin_Time;
        }

        // Getter for destination_Port
        const std::string& GetDestinationPort() const {
            return destination_port;
        }

        // Setter for destination_Port
        void SetDestinationPort(const std::string& newDestination_Port) {
            destination_port = newDestination_Port;
        }

        // Getter for arrival_Time_to_destination
        const int GetDestinationTime() const {
            return destination_time;
        }

        // Setter for arrival_Time_to_destination
        void SetDestinationTime(const int newArrival_Time_to_destination) {
            destination_time = newArrival_Time_to_destination;
        }

        // Getter for pathTime
        const int GetPathTime() const {
            return path_time;
        }

        // Setter for pathTime
        void SetPathTime(const int newPathTime) {
            path_time = newPathTime;
        }

        inline const std::vector<Arc>& GetArcs() const{
			return arcs;
		}
        inline void SetArcs(const std::vector<Arc> _arcs) {
            arcs = _arcs;
        }

        inline const std::vector<std::string>& GetPathPorts() const{
			return Path_port;
		}

        inline void SetPathPorts(const std::vector<std::string>& _Path_port) {
			Path_port = _Path_port;
		}

        inline void AddPort2PathPorts(const std::string& _port) {
            Path_port.push_back(_port);
        }

        inline void AddArc2Arcs(const Arc _arc) {
			arcs.push_back(_arc);
		}

        inline void AddArcID2ArcIDs(const int _arcID) {
            arcIDs.push_back(_arcID);
        }

        inline const Node& GetDestinationNode() const{
            return arcs.back().GetHead();
        }

        inline const Node& GetOriginNode() const{
			return arcs.front().GetTail();
		}

        // Getter for numberOfPath
        inline int SetNumberofPath() const {
            return numberOfPath;
        }

        // Setter for numberOfPath
        inline void SetNumberofPath(const int newNumberOfPath) {
            numberOfPath = newNumberOfPath;
        }

        // Getter for numberOfArcs
        inline const int GetNumArcs() const {
            return num_arcs;
        }

        // Setter for numberOfArcs
        inline void SetNumArcs(const int newNumberOfArcs) {
            num_arcs = newNumberOfArcs;
        }

        // Getter for arcs_ID
        inline const std::vector<int>& GetArcIDs() const {
            return arcIDs;
        }

        // Setter for arcs_ID
        inline void SetArcIDs(const std::vector<int>& newArcsID) {
            arcIDs = newArcsID;
        }

        // Getter for path_cost
        [[nodiscard]] inline const double GetPathCost() const {
            return path_cost;
        }

        // Setter for path_cost
        void SetPathCost(double newPath_cost) {
            path_cost = newPath_cost;
        }

        bool operator == (Path& right)
        {
            if (GetOriginNode() != right.GetOriginNode()) {
                return false;
            }
            else {
                if (GetDestinationNode() != right.GetDestinationNode()) {
                    return false;
                }
                else {
                    for (size_t i = 0; i < arcIDs.size(); i++)
                    {
                        if (arcIDs[i] != right.arcIDs[i]) {
                            return false;
                        }
                    }
                    return true;
                }
            }
        }

        // copy assignment operator
        Path& operator = (const Path& right) {
			PathID = right.PathID;
			origin_port = right.origin_port;
			destination_port = right.destination_port;
			origin_time = right.origin_time;
			destination_time = right.destination_time;
			path_time = right.path_time;
			Path_port = right.Path_port;
			arcIDs = right.arcIDs;
			arcs = right.arcs;
			num_arcs = right.num_arcs;
			path_cost = right.path_cost;
			return *this;
		}

        const bool operator < (const Path& right) const
        {
            if (origin_port != right.origin_port)
            {
                return origin_port < right.origin_port;
            }
            else
            {
                if (destination_port != right.destination_port)
                {
                    return destination_port < right.destination_port;
                }
                else
                {
                    if (origin_time != right.origin_time)
                    {
                        return origin_time < right.origin_time;
                    }
                    else
                    {
                        if (destination_time != right.destination_time)
                        {
                            return destination_time < right.destination_time;
                        }
                        else
                        {
                            if (num_arcs != right.num_arcs)
                            {
                                return num_arcs < right.num_arcs;
                            }
                            else
                            {
                                return false;
                            }
                        }
                    }
                }
            }
        }

        void printPath() const
        {
            // print new path
            std::cout << PathID << '\t'
                << origin_port << "("
                << origin_time << ")" << '\t'
                << destination_port << "("
                << destination_time << ")" << '\t'
                << path_time << '\t';

            std::cout << Path_port.size() << '\t';

            for (size_t i = 0; i < Path_port.size(); i++)
            {
                if (i != 0) {
                    std::cout << ",";
                }
                std::cout << Path_port[i];
            }
            std::cout << '\t';

            std::cout << num_arcs << '\t';

            std::cout << std::endl;
            for (size_t i = 0; i < arcIDs.size(); i++)
            {
                if (i != 0) 
                {
                    std::cout << ",";
                }
                std::cout << arcIDs[i] << "(";
                arcs[i].printArc();
                std::cout << ")" << std::endl;                   
            }
            std::cout << std::endl;
        }

        const void outputPath(std::ofstream& fout) const {
            this->printPath();
            fout << PathID << '\t'
                << origin_port << '\t'
                << origin_time << '\t'
                << destination_port << '\t'
                << destination_time << '\t'
                << path_time << '\t';

            fout << GetPathPorts().size() << '\t';

            for (size_t j = 0; j < GetPathPorts().size(); j++)
            {
                if (j == 0)
                    fout << GetPathPorts()[j];
                else
                    fout << "," << GetPathPorts()[j];
            }
            fout << '\t';

            fout << GetNumArcs() << '\t';

            for (size_t j = 0; j < GetArcIDs().size(); j++)
            {
                if (j == 0)
                    fout << GetArcIDs()[j];
                else
                    fout << "," << GetArcIDs()[j];
            }
            fout << std::endl;
        }
};

#endif // _PATH_H_
