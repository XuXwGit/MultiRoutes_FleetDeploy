#ifndef _EMPTY_PATH_H_
#define _EMPTY_PATH_H_

#include <string>
#include <vector>

#include "ContainerPath.h"


namespace fleetdeployment
{
    class  EmptyPath;
}

class  fleetdeployment::EmptyPath : public ContainerPath {

private:
    int request_ID_;
    int origin_time_;
    int number_of_path_;
    std::vector<int> path_ID_;
    std::string origin_port_;

public:
    // Constructor (if needed)
    EmptyPath() {}

    // Getter and Setter for request_ID
    int GetRequestID() const { return request_ID_; }
    void SetRequestID(int request_ID) { request_ID_ = request_ID; }

    // Getter and Setter for origin_Port
    const std::string& GetOriginPort() const { return origin_port_; }
    void SetOriginPort(const std::string& origin_port) { origin_port_ = origin_port; }

    // Getter and Setter for origin_time
    int GetOriginTime() const { return origin_time_; }
    void SetOriginTime(int origin_time) { origin_time_ = origin_time; }

    // Getter and Setter for numberofPath
    int GetNumberOfPath() const { return number_of_path_; }
    void SetNumberOfPath(int number_of_path) { number_of_path_ = number_of_path; }

    // Getter and Setter for path_ID
    const std::vector<int>& GetPathID() const { return path_ID_; }
    void SetPathID(const std::vector<int>& path_ID) { path_ID_ = path_ID; }
};


#endif  // #define _EMPTY_PATH_H_
