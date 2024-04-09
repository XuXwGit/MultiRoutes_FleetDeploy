#ifndef _REQUEST_H_
#define _REQUEST_H_

#include <string>
#include <vector>


namespace fleetdeployment
{
    class  Request {
    private:
        int requestID;                       // Unique IDentifier for the request
        std::string originPort;              // Origin port for the request
        std::string destinationPort;         // Destination port for the request
        int originGroup;                     // Origin group for the request
        int destinationGroup;                // Destination group for the request
        int w_i_Earliest;                    // Earliest start time for the request
        int latestDestinationTime;           // Latest arrival time at the destination
        std::vector<int> ladenPaths;         // Array of laden path IDs
        std::vector<int> ladenPathIndexes;   // Indexes of laden paths
        int numberOfLadenPath;               // Number of laden paths
        std::vector<int> emptyPaths;         // Array of empty path IDs
        std::vector<int> emptyPathIndexes;   // Indexes of empty paths
        int numberOfEmptyPath;               // Number of empty paths

    public:
        // Default constructor
        Request() {
            requestID = 0;
            originGroup = 0;
            destinationGroup = 0;
            w_i_Earliest = 0;
            latestDestinationTime = 0;
            numberOfLadenPath = 0;
            numberOfEmptyPath = 0;
        }

        // Getter for requestID
        int GetRequestID() const {
            return requestID;
        }

        // Setter for requestID
        void SetRequestID(int newRequestID) {
            requestID = newRequestID;
        }

        // Getter for originPort
        std::string GetOriginPort() const {
            return originPort;
        }

        // Setter for originPort
        void SetOriginPort(const std::string& newOriginPort) {
            originPort = newOriginPort;
        }

        // Getter for destinationPort
        std::string GetDestinationPort() const {
            return destinationPort;
        }

        // Setter for destinationPort
        void SetDestinationPort(const std::string& newDestinationPort) {
            destinationPort = newDestinationPort;
        }

        // Getter for originGroup
        int GetOriginGroup() const {
            return originGroup;
        }

        // Setter for originGroup
        void SetOriginGroup(int newOriginGroup) {
            originGroup = newOriginGroup;
        }

        // Getter for destinationGroup
        int GetDestinationGroup() const {
            return destinationGroup;
        }

        // Setter for destinationGroup
        void SetDestinationGroup(int newDestinationGroup) {
            destinationGroup = newDestinationGroup;
        }

        // Getter for w_i_Earliest
        int GetW_i_Earliest() const {
            return w_i_Earliest;
        }

        // Setter for w_i_Earliest
        void SetW_i_Earliest(int newW_i_Earliest) {
            w_i_Earliest = newW_i_Earliest;
        }

        // Getter for latestDestinationTime
        int GetLatestDestinationTime() const {
            return latestDestinationTime;
        }

        // Setter for latestDestinationTime
        void SetLatestDestinationTime(int newLatestDestinationTime) {
            latestDestinationTime = newLatestDestinationTime;
        }

        // Getter for ladenPaths
        std::vector<int> GetLadenPaths() const {
            return ladenPaths;
        }

        // Setter for ladenPaths
        void SetLadenPaths(const std::vector<int>& newLadenPaths) {
            ladenPaths = newLadenPaths;
        }

        // Getter for ladenPathIndexes
        std::vector<int> GetLadenPathIndexes() const {
            return ladenPathIndexes;
        }

        // Setter for ladenPathIndexes
        void SetLadenPathIndexes(const std::vector<int>& newLadenPathIndexes) {
            ladenPathIndexes = newLadenPathIndexes;
        }

        // Getter for numberOfLadenPath
        int GetNumberOfLadenPath() const {
            return numberOfLadenPath;
        }

        // Setter for numberOfLadenPath
        void SetNumberOfLadenPath(int newNumberOfLadenPath) {
            numberOfLadenPath = newNumberOfLadenPath;
        }

        // Getter for emptyPaths
        std::vector<int> GetEmptyPaths() const {
            return emptyPaths;
        }

        // Setter for emptyPaths
        void SetEmptyPaths(const std::vector<int>& newEmptyPaths) {
            emptyPaths = newEmptyPaths;
        }

        // Getter for emptyPathIndexes
        std::vector<int> GetEmptyPathIndexes() const {
            return emptyPathIndexes;
        }

        // Setter for emptyPathIndexes
        void SetEmptyPathIndexes(const std::vector<int>& newEmptyPathIndexes) {
            emptyPathIndexes = newEmptyPathIndexes;
        }

        // Getter for numberOfEmptyPath
        int GetNumberOfEmptyPath() const {
            return numberOfEmptyPath;
        }

        // Setter for numberOfEmptyPath
        void SetNumberOfEmptyPath(int newNumberOfEmptyPath) {
            numberOfEmptyPath = newNumberOfEmptyPath;
        }
    };



}
#endif // _REQUEST_H_
