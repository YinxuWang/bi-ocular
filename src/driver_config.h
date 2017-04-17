///
/// Created by yinxu on 17-4-9.
///

#ifndef MAKER_BINOCULAR_DRIVER_CONFIG_H
#define MAKER_BINOCULAR_DRIVER_CONFIG_H

#include <boost/program_options.hpp>
#include <stdio.h>
#include <string>
#include <vector>
#include <fstream>
#include <iostream>
#include <sstream>
#include <math.h>
namespace po = boost::program_options;


class driver_config {
public:

    driver_config();

    ~driver_config();

    /// the function that handles main function arguments,
    /// return 0 for succeed and non-zero for different errors
    int argments_parser(int argc, char **argv);

private:

    /// range: 0 ~ 60hz
    int frame_rate;

    /// range: 0 ~ 500hz
    int imu_sampling_frequency;

    /// false for recording and true for calibration
    bool calibration_mode;

    /// num of shown images, 1 for left, 2 for right, 3 for two at the same time
    int images_num;

};


#endif //MAKER_BINOCULAR_DRIVER_CONFIG_H
