#include "NumericalTest.h"


void print_memory() {
#ifdef _WIN32
  MEMORYSTATUSEX memoryStatus;
  memoryStatus.dwLength = sizeof(MEMORYSTATUSEX);
  GlobalMemoryStatusEx(&memoryStatus);

  std::cout << "Max heap Memory = " << memoryStatus.ullTotalPhys / (1024 * 1024)
            << "M\n";
  std::cout << "Total heap Memory = "
            << memoryStatus.ullTotalVirtual / (1024 * 1024) << "M\n";
#elif defined(__unix__) || defined(__APPLE__)
  struct rusage r_usage;
  getrusage(RUSAGE_SELF, &r_usage);
  // Note: This is a rough estimate and may not be entirely accurate
  std::cout << "Max heap Memory = " << r_usage.ru_maxrss / 1024
            << "M\n";  // Maximum resident set size
#endif
  std::cout << "Max Available Cores = " << std::thread::hardware_concurrency()
            << "\n";
}

void test_lca()
{
	lca_od_model();

	lca_origin_model();
}

	void numerical_test() {
        for (int i = 5; i < 6; i++) {
          std::cout << "=============== Seed = " << i * 100
                    << "===============" << std::endl;

          // input parameters :
          // instance	experiment
          // 1					true
          // 2					true
          // 1					false
          // 2					false
          new PerformanceExperiment(1, true);

          // input:  instance ID and method
          // instance :
          // 1 ！！ R=2
          // 2 ！！ R=8
          // method :
          //	"CCG"
          // "BD"
          new CompareWithReactive(2, "CCG");
          new CompareWithReactive(2, "BD");

          // use method
          // 1: BD & PAP
          // 2: CCG & PAP
          new SensitivityAnalysis(1);
        }
}

std::vector<std::string> load_instances(std::string& path) {
        std::vector<std::string> instances;
        int totalLine = 0;
        try {
          std::ifstream file(path + "instances.txt");

          if (file.is_open()) {
            std::cout << "Success to Load Instance File" << std::endl;

            std::string line;
            bool firstTime = true;
            while (std::getline(file, line)) {
              std::cout << line << std::endl;
              std::vector<std::string> ss;
              size_t pos = 0;
              while ((pos = line.find('\t')) != std::string::npos) {
                ss.push_back(line.substr(0, pos));
                line.erase(0, pos + 1);
              }
              ss.push_back(line);

              if (firstTime) {
                firstTime = false;
              } else {
                instances.push_back(ss[1]);
              }
              totalLine++;
            }
            file.close();
          } else {
            std::cout << "Can not find the instances file" << std::endl;
          }
        } catch (const std::exception& e) {
          std::cout << "Error in read data" << std::endl;
          std::cerr << e.what() << std::endl;
        }

        return instances;
}

	void generate_paths() {
        // Input data of liner shipping network (R, P, W)
        // R : the Set of ship routes / ship rotations
        // P : the Set of ports
        // W: the Set of O-D pairs
        // datafile: data2-10; data3-6; data8-29; data11-46; data13-28
        // "ASIA - AFRICA"
        // "ASIA - NORTH AMERICA"		14
        // "ASIA - SOUTH AMERICA"		3-21
        // "ASIA - OCEANIA￣		7-24
        // "INTRA-ASIA"		14-32
        // ['NORTH AMERICA - SOUTH AMERICA',
        // 'NORTH AMERICA - CARIBBEAN',
        // 'NORTH AMERICA - MIDDLE EAST-IND',
        // 'ASIA - AFRICA',
        // 'ASIA - NORTH AMERICA',
        // 'ASIA - SOUTH AMERICA',
        // 'ASIA - EUROPE',
        // 'ASIA - MEDITERRANEAN',
        // 'ASIA - MIDDLE EAST INDIA',
        // 'ASIA - OCEANIA', 'EUROPE - AFRICA',
        // 'EUROPE - NORTH AMERICA',
        // 'EUROPE - SOUTH AMERICA',
        // 'EUROPE - MIDDLE EAST INDIA',
        // 'EUROPE - OCEANIA',
        // 'INTRA-AFRICA',
        // 'INTRA - CARIBBEAN',
        // 'INTRA-SOUTH AMERICA',
        // 'INTRA-ASIA',
        // 'INTRA-EUROPE',
        // 'INTRA-MEDITERRANEAN',
        // 'INTRA-MIDDLE EAST INDIA',
        // 'MEDITERRANEAN - NORTH EUROPE',
        // 'MIDDLE EAST INDIA - AFRICA',
        // 'NORTH EUROPE - MEDITERRANEAN']
        std::string path = "/data/";

        std::vector<std::string> instances = load_instances(path);

        for (std::string instance : instances) {
          if (instance == "ASIA - SOUTH AMERICA") {
            continue;
          }

          std::string input_filepath = path + instance + "/input/";
          std::string output_filepath = path + instance + "/output/";

          Data D(input_filepath);

          ST_network ST(D);

          output_STdata(output_filepath, ST);

          GeneratePath GP(ST, output_filepath);

          output_Path_data(output_filepath, GP);
        }
}