///
/// Created by yinxu on 17-4-9.
///

#include <iostream>
#include <stdio.h>
#include <libusb-1.0/libusb.h>
#include <boost/concept_check.hpp>

#include <opencv2/opencv.hpp>
#include "maker_binocular.h"
#include "driver_config.h"


int main(int argc, char **argv) {

//    while(argc--){
//        switch (argv[argc])
//        {
//        }
//    }

//    driver_config m_driver_config;
//    m_driver_config.argments_parser(argc, argv);
    makerbinocular m_makerbinocular(25, 100);

    cv::Mat left_image(480, 640, CV_8UC1, cv::Scalar(0));
    cv::Mat right_image(480, 640, CV_8UC1, cv::Scalar(0));


    {
        while (1) {
            m_makerbinocular.get_frame(left_image, right_image);
            m_makerbinocular.read_USB_data();
        }
    }

    return 0;
}
