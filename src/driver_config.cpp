///
/// Created by yinxu on 17-4-9.
///

#include "driver_config.h"


driver_config::driver_config() {

}

driver_config::~driver_config() {

}

int driver_config::argments_parser(int argc, char **argv) {

//    /// Declare the supported options.
//    po::options_description desc("Allowed options");
//    desc.add_options()
//            ("help", "produce help message")
//            ("compression", po::value<int>(), "set compression level");
//
//    po::variables_map vm;
//    po::store(po::parse_command_line(argc, *argv, desc), vm);
//    po::notify(vm);
//
//    if (vm.count("help")) {
//        std::cout << desc << "\n";
//        return 1;
//    }
//
//    if (vm.count("compression")) {
//        std::cout << "Compression level was set to "
//                  << vm["compression"].as<int>() << ".\n";
//    } else {
//        std::cout << "Compression level was not set.\n";
//    }
}