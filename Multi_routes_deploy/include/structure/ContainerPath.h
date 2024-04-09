#ifndef _CONTAINERPATH_H_
#define _CONTAINERPATH_H_

#include <string>
#include <vector>
#include <fstream>
#include <numeric>

#include "Arc.h"
#include "Path.h"

namespace fleetdeployment
{
    class ContainerPath;
}

class  fleetdeployment::ContainerPath : public Path{
    protected:
        int request_ID;                                         // Request IDentifier
        int totalTransshipTime;                         // Total transshipment time
        int totalDemurrageTime;						    // Total demurrage time
        int numberOfPath;                                   // Number of paths
        int minAllowTime = 0;					         // Minimum allowable time at transshipment port
        double demurrage_cost;
        std::vector<std::string> transship_port;   // Transshipment port names
        std::vector<int> transship_time;            // Transshipment times

    public:
        // Default constructor
        ContainerPath() 
            : Path(), 
            request_ID(-1), 
            transship_port(std::vector<std::string>()), 
            transship_time(std::vector<int>()), 
            totalTransshipTime(-1), 
            totalDemurrageTime(-1), 
            numberOfPath(-1), 
            minAllowTime(0),
            demurrage_cost(0.0) 
        {}

        // Getter for request_ID
        inline int GetRequestID() const {
            return request_ID;
        }

        // Setter for request_ID
        inline void SetRequestID(const int newRequestID) {
            request_ID = newRequestID;
        }

        // Getter for transshipment_port
        inline const std::vector<std::string>& GetTransshipPorts() const {
            return transship_port;
        }

        // Setter for transshipment_port
        inline void SetTransshipPorts(const std::vector<std::string>& newTransshipmentPort) {
            transship_port = newTransshipmentPort;
        }

        inline void AddPort2TransshipPorts(const std::string& _port) {
			        transship_port.push_back(_port);
		 }

        inline void AddTime2TransshipTime(const int _time) {
			transship_time.push_back(_time);
		}


        // Getter for transshipment_Time
        inline const std::vector<int>& GetTransshipTime() const {
            return transship_time;
        }

        // Setter for transshipment_Time
        inline void SetTransshipTime(const std::vector<int>& newTransshipment_Time) {
            transship_time = newTransshipment_Time;
            SetTotalTransshipmentTime(accumulate(newTransshipment_Time.begin(), newTransshipment_Time.end(), 0));
            int newTotalDemurrageTime = 0;
            for (int i = 0; i < transship_time.size(); i++) {
                if (transship_time[i] < minAllowTime) {
                    newTotalDemurrageTime += (transship_time[i] - minAllowTime > 0 ? transship_time[i] - minAllowTime : 0);
                }
            }
            SetTotalDemurrageTime(newTotalDemurrageTime);
        }

        // Getter for totalTransshipTime
        inline int GetTotalTransshipmentTime() const {
            return totalTransshipTime;
        }

        inline void SetTotalTransshipmentTime(const int newTotalTransshipTime) {
            totalTransshipTime = newTotalTransshipTime;
        }

        // Getter for totalTransshipTime
        inline int GetTotalDemurrageTime() const {
            return totalDemurrageTime;
        }

        inline void SetTotalDemurrageTime(const int newTotalDemurrageTime) {
            totalTransshipTime = newTotalDemurrageTime;
        }

        const bool operator < (const ContainerPath& right) const
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
                                if (transship_port.size() != right.transship_port.size())
                                    return transship_port.size() < right.transship_port.size();
                                else
                                    if(totalTransshipTime < right.totalTransshipTime)
                                        return totalTransshipTime < right.totalTransshipTime;
									else
										return false;
                            }
                        }
                    }
                }
            }
        }


        const void printPath() const
        {
            // print new path
            std::cout << PathID << '\t'
                << origin_port << "("
                << origin_time << ")" << '\t'
                << destination_port << "("
                << destination_time << ")" << '\t'
                << path_time << '\t';

            if (transship_port.empty())
                std::cout << "[]" << '\t';
            else
            {
                for (size_t i = 0; i < transship_port.size(); i++)
                {
                    if (i != 0)
                        std::cout << ",";
                    std::cout << transship_port[i] << "(" << transship_time[i] << ")";
                }
                std::cout << '\t';
            }

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

            if (GetTransshipPorts().empty())
                fout << 0 << '\t' << 0 << '\t';
            else
            {
                for (size_t j = 0; j < GetTransshipPorts().size(); j++)
                {
                    if (j == 0)
                        fout << GetTransshipPorts()[j];
                    else
                        fout << "," << GetTransshipPorts()[j];
                }
                fout << '\t';

                for (size_t j = 0; j < GetTransshipTime().size(); j++)
                {
                    if (j == 0)
                        fout << GetTransshipTime()[j];
                    else
                        fout << "," << GetTransshipTime()[j];
                }
                fout << '\t';
            }
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

#endif // _CONTAINERPATH_H_
