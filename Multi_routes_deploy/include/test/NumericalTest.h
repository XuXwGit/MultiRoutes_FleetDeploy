#ifndef _NUMERICAL_TEST_H_
#define _NUMERICAL_TEST_H_

#ifdef _WIN32
#include <windows.h>
#elif defined(__unix__) || defined(__APPLE__)
#include <sys/resource.h>
#include <unistd.h>
#endif

#include <iostream>
#include <vector>
#include <string>
#include <thread>

#include "lca_od_model.h"
#include "lca_origin_model.h"
#include "PerformanceExperiment.h"
#include "CompareWithReactive.h"
#include "SensitivityAnalysis.h"
#include "output_data.h"
#include "generate_path.h"

using namespace std;
using namespace fleetdeployment;

void test_lca();
void print_memory();
void generate_paths();
void numerical_test();
std::vector<string> load_instances(std::string& path);


#endif  // !_NUMERICAL_TEST_H_
