#ifndef _DATA_PROCESS_H_
#define _DATA_PROCESS_H_

#include <fstream>
#include <iostream>
#include <sstream>
#include <ctime>
#include <algorithm>
#include <set>
#include <vector>
#include <string>

namespace fleetdeployment
{
	// int to string
	std::string i_to_s(size_t i);

	static std::string i_to_s(size_t i) {
		std::stringstream s;
		s << i;
		return s.str();
	};

    static std::vector<std::string> split(const std::string& str, const std::string& delimiter) {
        std::vector<std::string> tokens;
        size_t start = 0, end = 0;
        while ((end = str.find(delimiter, start)) != std::string::npos) {
            tokens.push_back(str.substr(start, (end - start)));
            start = end + delimiter.size();
        }
        tokens.push_back(str.substr(start));
        return tokens;
    };
}

#endif // !_DATA_PROCESS_H_