#ifndef _SCENARIO_H_
#define _SCENARIO_H_

#include <vector>


namespace fleetdeployment
{



    class  Scenario {
    private:
        std::vector<double> request;  // Array of request values

    public:
        // Default constructor
        Scenario() {}

        // Getter for request
        std::vector<double> GetRequest() const {
            return request;
        }

        // Setter for request
        void SetRequest(const std::vector<double>& newRequest) {
            request = newRequest;
        }
    };


}

#endif // _SCENARIO_H_
