///
/// Created by yinxu on 17-4-9.
///

#include "maker_binocular.h"
#include <iomanip>
#include <netinet/in.h>


struct dispalyArg {
    unsigned char *leftImage;
    unsigned char *rightImage;
};

/*
static void *display_thread_func(void *arg){
    /// raw data to two images conversion to show image via openCV ///
    int cnt_y, cnt_x;
    struct displayArg* p = (struct dispalyArg *) arg;

    unsigned char *pcL =  ((struct dispalyArg *) arg->leftImage);
    unsigned char *pcR =  ((dispalyArg *) arg->rightImage);


    cv::Mat left_image(480, 640, CV_8UC1, cv::Scalar(0));
    cv::Mat right_image(480, 640, CV_8UC1, cv::Scalar(0));

    for (cnt_y = 0; cnt_y < 480; cnt_y++) {
        for (cnt_x = 0; cnt_x < 640; cnt_x++) {
            // left image
            left_image.at<uchar>(479 - cnt_y, cnt_x) = *(pcL + cnt_y * 640 + cnt_x);
            // right image
            right_image.at<uchar>(479 - cnt_y, cnt_x) = *(pcR + cnt_y  * 640 + cnt_x);
        }
    }

    cv::imshow("left image", left_image);
    cv::waitKey(1);
    cv::imshow("right image" ,  right_image);
    cv::waitKey(1);
}
 */

makerbinocular::makerbinocular(unsigned char frame_rate, unsigned char imu_frequency) {
    init();
}

makerbinocular::~makerbinocular() {

    libusb_free_device_list(devs, 1);
    libusb_close(dev_handle);
}


/// do initilization work ///
void makerbinocular::init() {
    initialized = false;
    imu_initialized = false;

    calibration_mode = true;

    // init required data
    has_new_frame = false;
    imu_init_state = 0;


    current_image_time = 0;
    current_imu_time = 0;
    time_elapsed = 0;

    image_frame_time_stamp = 0;
    frame_time_stamp = 0;


    /// the negative value is num of frames discarded
    current_frame_index = -10;
    current_expose_time = -3;
    dataRem = 0;
    dataLeft = 0;

    read_size = 76800*8;
    IMU_sample_count = 0;

    processState = Process_Header;

    dataCopied = 0;
    isFirstXfer = true;


    time_reset_value = 0;


    /// print header or column names to .csv file ///
    imu_data_check_csv.open("imu0.csv", std::ios::out | std::ios::app);
    imu_data_check_csv.precision(16);
    imu_data_check_csv << std::setw(16) << "timestamp" << "," << "omega_x" << "," << "omega_y" << "," << "omega_z"
                       << ","
                       << "alpha_x" << "," << "alpha_y" << "," << "alpha_z" << "\n";


    int r;
    int err;
    ssize_t cnt;

    /// initialize a library session
    r = libusb_init(&contex);

    if (r < 0) {
        std::cout << "Init error!" << std::endl;
    }

    /// set verbosity level to 3
    libusb_set_debug(contex, 3);

    cnt = libusb_get_device_list(contex, &devs);

    if (cnt < 0) {
        std::cout << "Get devices error" << std::endl;
    } else {
        std::cout << "Get " << cnt << " devices in total" << std::endl;
    }

    // found cypress usb device
    bool idVendorAndProductfound = false;
    for (int i = 0; i < cnt; i++) {
        device = devs[i];
        err = libusb_get_device_descriptor(device, &desc);

        if (err < 0) {
            std::cout << "failed to get desc" << std::endl;
            return;
        }

        if (desc.idVendor == 0x2014 && desc.idProduct == 0x0117) {
            std::cout << "============================================" << std::endl;
            printf("Found cypress usb device: idVendor 0x%04x idProduct: 0x%04x\r\n", desc.idVendor, desc.idProduct);
            std::cout << "============================================" << std::endl;
            idVendorAndProductfound = true;
            break;
        }
    }

    if (idVendorAndProductfound == false) {
        std::cout << "============================================" << std::endl;
        std::cout << "Error: Can not found the device, please check the idVendor and idProduct!" << std::endl;
        std::cout << "============================================" << std::endl;
        return;
    }

    /// get cypress usb config desc
    libusb_get_config_descriptor(device, 0, &config);

    /// check the USB interface num ///
    if ((int) config->bNumInterfaces > 1)
        std::cout << "too many interfaces, please check the USB firmware." << std::endl;

    const struct libusb_interface *inter;
    const struct libusb_interface_descriptor *interdesc;
    const struct libusb_endpoint_descriptor *epdesc;

    inter = &config->interface[0];
    interdesc = &inter->altsetting[0];

    if ((int) interdesc->bNumEndpoints > 2)
        std::cout << "too many endpoints,  please check the USB firmware." << std::endl;

    for (int j = 0; j < interdesc->bNumEndpoints; j++) {
        epdesc = &interdesc->endpoint[j];

        if ((epdesc->bEndpointAddress) & 0x80) {
            bulk_ep_in = epdesc->bEndpointAddress;
            printf("Hints: Get Built in endpoint: 0x%02x\n", epdesc->bEndpointAddress);
            printf("Max packetsize is %d \n", epdesc->wMaxPacketSize);
        }
    }

    err = libusb_open(device, &dev_handle);

    if (err < 0) {
        printf("open device failed\n");
        libusb_free_device_list(devs, 1);
        libusb_close(dev_handle);
        return;
    }

    if (libusb_kernel_driver_active(dev_handle, 0) == 1) {
        printf("Kernel Driver Active\n");
        if (libusb_detach_kernel_driver(dev_handle, 0) == 0)
            printf("Kernal Driver Detached\n");
    }

    err = libusb_claim_interface(dev_handle, 0);
    if (err < 0) {
        printf("can not claim interface");
        libusb_free_device_list(devs, 1);
        libusb_close(dev_handle);
        return;
    }

    /// USB commands data ///
    unsigned char data[4];


    ///*****************************************************************************///
    /// The following part is used to set the imu-cam parameters via USB command ///
    ///*****************************************************************************///

    /// set the frame rate, data[1] is the rate ///
    data[0] = 0x02; // mode
    data[1] = 20; // hz
    data[2] = 0x00;
    data[3] = 0x00;

    /// bmRequest type:0b01000000 : host to device, class, device
    /// bRequest :    0xD0 : defined by product vendors
    /// bRequest type is defined in USB firmware, that is on board. Check the Excel file for details
    if (!libusb_control_transfer(dev_handle, 0x40, 0xD0, 0, 0, data, 4, 100))
        return;




    /// set the exposure and gain mode ///
    data[0] = 0x00;
    data[1] = 0x00;
    data[2] = 0x00;
    data[3] = 0x00;

    if (!libusb_control_transfer(dev_handle, 0x40, 0xD2, 0, 0, data, 4, 100))
        return;




    /// set the gain when AGC(auto gain control) is disabled ///
    data[0] = 0x2f; /// max value can be set
    data[1] = 0x00;
    data[2] = 0x00;
    data[3] = 0x00;

    if (!libusb_control_transfer(dev_handle, 0x40, 0xD3, 0, 0, data, 4, 100))
        return;




    /// set the exposure time, data[0]<<8|data[1] * 27.185 = ....us ///
    data[0] = 0x01;
    data[1] = 0x72;
    data[2] = 0x00;
    data[3] = 0x00;

    if (!libusb_control_transfer(dev_handle, 0x40, 0xD4, 0, 0, data, 4, 100))
        return;



    /// set the IMU sampling frequency, set the value = imu sampling frequency / cam rate ///
    data[0] = 0x0a; /// 4 times sampling frequency
    data[1] = 0x00;
    data[2] = 0x00;
    data[3] = 0x00;

    if (!libusb_control_transfer(dev_handle, 0x40, 0xDE, 0, 0, data, 4, 100))
        return;




    /// initialization done, set the flag ///
    initialized = true;
}

int makerbinocular::read_USB_data() {
    int error;
    int transferd;

    error = libusb_bulk_transfer(dev_handle, bulk_ep_in, dataBuffer + dataRem, read_size, &transferd, 1000);

    dataLeft = dataRem + read_size;
    //std::cout << "Current system time is " << time(&timev) <<std::endl;
    //std::cout << "Total bytes transfered in bulk transfer is " << transferd << std::endl;

    return error;
}


/// imuheaderIndex should be the index of header of IMU data, 0x66(102) and 0xdd(221) ///
int makerbinocular::process_imu_data(int imuHeaderIndex) {
    if ((dataBuffer[imuHeaderIndex] == 0x66) & (dataBuffer[imuHeaderIndex + 1]) == 0xdd) {
        int imu_timestamp_raw;
        double imu_timestamp;
        short accX_raw, accY_raw, accZ_raw, gyroX_raw, gyroY_raw, gyroZ_raw, temp_raw;
        double accX, accY, accZ, temp, gyroX, gyroY, gyroZ;

        imu_data_txt.open("imu_interpolated.txt", std::ios::out | std::ios::app);
        imu_data_txt.precision(16);
        imu_data_check_csv.open("imu0.csv", std::ios::out | std::ios::app);
        imu_data_check_csv.precision(16);
        // std::cout << "processing IMU data.\n";

        for (int i = imuHeaderIndex; i < IMU_sample_count * 18 + imuHeaderIndex; i++) {

            /// unit: 100us ///
            imu_timestamp_raw = (dataBuffer[i + 2] << 24) | (dataBuffer[i + 3] << 16) |
                                (dataBuffer[i + 4] << 8) + dataBuffer[i + 5];
            accX_raw = (dataBuffer[i + 6] << 8) | (dataBuffer[i + 7]);
            accY_raw = (dataBuffer[i + 8] << 8) | (dataBuffer[i + 9]);
            accZ_raw = (dataBuffer[i + 10] << 8) | (dataBuffer[i + 11]);

            temp_raw = (dataBuffer[i + 12] << 8) | (dataBuffer[i + 13]);

            gyroX_raw = (dataBuffer[i + 14] << 8) | (dataBuffer[i + 15]);
            gyroY_raw = (dataBuffer[i + 16] << 8) | (dataBuffer[i + 17]);
            gyroZ_raw = (dataBuffer[i + 18] << 8) | (dataBuffer[i + 19]);

            /// unit: second ///
            imu_timestamp = imu_timestamp_raw * 1.0 / 10000;

            /// unit: m/(s*s) for +-4g ///
            accX = accX_raw * 8.0 / 65536 * 9.8;
            accY = accY_raw * 8.0 / 65536 * 9.8;
            accZ = accZ_raw * 8.0 / 65536 * 9.8;

            /// unit: degree centigrade, the formula is got from datasheet ///
            temp = temp_raw / 325.8 + 25;

            /// unit: degree per second for +-1000g ///
            gyroX = gyroX_raw * 2.0 * M_PI / (65536 * 360) * 2000;
            gyroY = gyroY_raw * 2.0 * M_PI / (65536 * 360) * 2000;
            gyroZ = gyroZ_raw * 2.0 * M_PI / (65536 * 360) * 2000;

            /// increase the index ///
            i += 17;

            ///*****************************************************************************///
            /// The following part is used to print the imu data to txt or csv file ///
            ///*****************************************************************************///

            /// do driver-side timestamp reset ///
            double calibrated_timestamp = imu_timestamp - time_reset_value;
            if (current_frame_index == 0) {
                if (!calibration_mode) {
                    /// trick just for first row of imu data, set the imu_timestamp as 0///
                    if (calibrated_timestamp < 0.0001)
                        imu_data_txt << std::setw(16) << 0 << " " << 0 << " " << gyroX
                                     << " " << gyroY << " "
                                     << gyroZ << " " << accX
                                     << " " << accY << " " << accZ << " " << 0 << " " << 0 << " " << 0 << " " << temp
                                     << "\n";

                        /// otherwise, just set as it is ///
                    else
                        imu_data_txt << std::setw(16) << calibrated_timestamp << " " << 0 << " " << gyroX
                                     << " " << gyroY << " "
                                     << gyroZ << " " << accX
                                     << " " << accY << " " << accZ << " " << 0 << " " << 0 << " " << 0 << " " << temp
                                     << "\n";

                } else {
                    /// trick just for first row of imu data, set the imu_timestamp as 0///
                    if (calibrated_timestamp < 0.0001)
                        imu_data_check_csv << std::setw(16)
                                           << 1000000000000000001 << ","
                                           << gyroX << ","
                                           << gyroY << ","
                                           << gyroZ << ","
                                           << accX << ","
                                           << accY << ","
                                           << accZ << "\n";
                        /// otherwise, just set as it is ///
                    else
                        imu_data_check_csv << std::setw(16)
                                           << (long) (calibrated_timestamp * 1000000000 + 1000000000000000001) << ","
                                           << gyroX << ","
                                           << gyroY << ","
                                           << gyroZ << ","
                                           << accX << ","
                                           << accY << ","
                                           << accZ << "\n";
                }
            } else
                /// print for current_frame_index > 0 ///
            if (current_frame_index > 0) {
                if (!calibration_mode) {
                    imu_data_txt << std::setw(16)
                                 << calibrated_timestamp << " "
                                 << 0 << " "
                                 << gyroX << " "
                                 << gyroY << " "
                                 << gyroZ << " "
                                 << accX << " "
                                 << accY << " "
                                 << accZ << " "
                                 << 0 << " "
                                 << 0 << " "
                                 << 0 << " "
                                 << temp
                                 << "\n";
                } else {
                    imu_data_check_csv << std::setw(16)
                                       << (long) (calibrated_timestamp * 1000000000 + 1000000000000000001) << ","
                                       << gyroX << ","
                                       << gyroY << ","
                                       << gyroZ << ","
                                       << accX << ","
                                       << accY << ","
                                       << accZ << "\n";
                }
            }
        }
        imu_data_txt.close();
        imu_data_check_csv.close();
    } else {
        std::cout << "imu header error";
        return 0;
    }
}

/// function that processes the data read into dataBuffer. ///
/// Once we get into the function, return only when all data in dataBuffer has been processed. ///
/// processed the data like a state-machine, and we may processed more than one frames every time we call this function///
bool makerbinocular::get_frame(cv::Mat &left_image, cv::Mat &right_image) {

    //std::cout << "======================get frame======================" << time(&timev) <<std::endl;

    time_elapsed = 0;
    int imuLen = 0;
    int frame_time_stamp_raw = 0;

    /*
    if (left_image.rows != 480 | left_image.cols != 640 | right_image.rows != 480 | right_image.cols != 640) {
        std::cout << left_image.rows << left_image.cols << std::endl;
        std::cout << "Error: the image size should be: 640 x 480" << std::endl;
    }
    */

    if (processState == Process_Header) {
        if (dataLeft < 1024) {
            memcpy(dataBuffer, dataBuffer, dataLeft);
            dataRem = dataLeft;
            return false;
        } else {

        }
    }

    int totalBytes = read_size + dataRem;

    for (int i = 0; i < totalBytes; i++) {
        /// processing frame header ///
        if (processState == Process_Header) {
            if ((i + 15 + 1000) > totalBytes) {
                dataRem = totalBytes - i;
                memcpy(dataBuffer, dataBuffer + i, dataRem);
                return false;
            }

            /// frame header check ///
            if (((*(dataBuffer + i) == 51) && (*(dataBuffer + i + 1) == 204) && (*(dataBuffer + i + 14) == 34) &&
                 (*(dataBuffer + i + 15) == 221))) {


                //std::cout << "frame Header found, the index is " << i << std::endl;

                processState = Process_IMU;
                IMU_sample_count = dataBuffer[i + 5];
                imuLen = IMU_sample_count * 18;
                int current_expose_time_raw = (dataBuffer[i + 2] << 8) | dataBuffer[i + 3];
                current_expose_time = current_expose_time_raw * 1.0 * 27.185 / 1000;


                /// unit: 100us ///
                frame_time_stamp_raw = (dataBuffer[i + 10] << 24) | (dataBuffer[i + 11] << 16) |
                                       (dataBuffer[i + 12] << 8) + dataBuffer[i + 13];

                /// unit: second ///
                frame_time_stamp = frame_time_stamp_raw * 1.0 / 10000;
                if (current_frame_index == 0) time_reset_value = frame_time_stamp;


                //std::cout << "IMU sample count is " << IMU_sample_count << std::endl;
                //std::cout << "current exposure time is " << current_expose_time << std::endl;
                //std::cout << "time stamp::" << frame_time_stamp << std::endl;


                /// if the frame header is i, imuheader should be i + 16 ///
                /// process IMU data by calling makerbinocular public method process_imu_data ///
                makerbinocular::process_imu_data(i + 16);

                i += 20 + imuLen;
                dataLeft = totalBytes - i;
                processState = Process_Left_Image;


            } else {
                //std::cout << " Error" << i << std::endl;
                continue;
            }
        }

        /// now we start to process image data ///
        if (processState == Process_Left_Image) {
            /// processing left image ///
            int dataNeedCopy = 307200 - dataCopied;
            if (dataLeft >= (dataNeedCopy)) {
                memcpy(leftImageBuffer + dataCopied, dataBuffer + i, dataNeedCopy);
                dataLeft -= dataNeedCopy;
                i += dataNeedCopy;
                dataRem = dataLeft;
                processState = Process_Right_Image;
                dataCopied = 0;
            } else {
                memcpy(leftImageBuffer + dataCopied, dataBuffer + i, dataLeft);
                dataCopied += dataLeft;
                dataLeft = 0;
                dataRem = 0;

                return false;
            }
        }
        if (processState == Process_Right_Image) {
            /// processing right image ///
            int dataNeedCopy = 307200 - dataCopied;
            if (dataLeft >= (dataNeedCopy)) {
                memcpy(rightImageBuffer + dataCopied, dataBuffer + i, dataNeedCopy);
                dataLeft -= dataNeedCopy;
                i += dataNeedCopy;
                dataRem = dataLeft;
                processState = Process_Frame_Done;
                dataCopied = 0;
            } else {
                memcpy(rightImageBuffer + dataCopied, dataBuffer + i, dataLeft);
                dataCopied += dataLeft;
                dataLeft = 0;
                dataRem = 0;
                return false;
            }
        }

        /// if we have processed imu data and two images, we show the images, write to file and update the timestamp.txt///
        if (processState == Process_Frame_Done) {

            double calibrated_time = frame_time_stamp - time_reset_value;
            if (calibrated_time < 0)
                calibrated_time = 0;

            cv::Mat my_mat1(480, 640, CV_8UC1, &leftImageBuffer);
            cv::Mat my_mat2(480, 640, CV_8UC1, &rightImageBuffer);


            /// write the raw data to image ///
            char imageFileName[24];
            if (!calibration_mode) {
                int temp = 10000000 + current_frame_index;

                sprintf(imageFileName, "%d.pgm", temp);
                imageFileName[0] = 'm';
            } else {
                long temp = (long) (calibrated_time * 1000000000 + 1000000000000000001);
                sprintf(imageFileName, "%ld.png", temp);
            }

            //std::cout << "image file name is " << imageFileName << std::endl;

            std::vector<int> compression_params;
            compression_params.push_back(CV_IMWRITE_PXM_BINARY);
            //compression_params.push_back(1);
            if (current_frame_index >= 0) {
                /// save images ///
                cv::imwrite(imageFileName, my_mat1, compression_params);
                /// record the images's timestamp ///
                image_data_timestamp_txt.open("timestamps.txt", std::ios::out | std::ios::app);
                image_data_timestamp_txt.precision(16);
                image_data_timestamp_txt << imageFileName << " " << calibrated_time << std::endl;
                image_data_timestamp_txt.close();
            }
            current_frame_index++;

            /// show images in windows ///
            cv::imshow("left image", my_mat1);
            cv::waitKey(1);
            //cv::imshow("right image", my_mat2);
            //cv::waitKey(1);

            /// raw data to two images conversion to show image via openCV ///
            //dispalyArg *displayArg1;
            //displayArg1->leftImage = leftImageBuffer;
            //displayArg1->rightImage = rightImageBuffer;
            //pthread_create(&displayThread,NULL,display_thread_func,displayArg1);

            processState = Process_Header;
            /// i will auto increase by 1 in the for loop, so decrease 1 to ensure no data skipped
            i -= 1;
        }
    }

    return has_new_frame;
}
/*
static void * display_thread_func_helper(void *arg){
   return makerbinocular::display_thread_func(arg);
}
 */


