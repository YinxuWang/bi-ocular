///
/// Created by yinxu on 17-4-9.
///

#ifndef MAKER_BINOCULAR_H
#define MAKER_BINOCULAR_H

#include <opencv2/opencv.hpp>
#include <libusb-1.0/libusb.h>
#include <boost/concept_check.hpp>
#include <stdio.h>
#include <string>
#include <vector>
#include <fstream>
#include <iostream>
#include <sstream>
#include <math.h>



typedef unsigned char u8;
enum Process_State_t {Process_Header, Process_IMU, Process_Left_Image, Process_Right_Image, Process_Frame_Done};

/**
 * @brief Driver for maker binocular 
 * Stereo images and 3 axis accelemeters and 3 axis gyroscope data are transfered
 * 
 */
class makerbinocular {
public:
    /**
     * @brief Constructor of class makerbinocular
     * 
     */
    makerbinocular(unsigned char frame_rate=25, unsigned char imu_frequency=200);

    /**
     * @brief Deconstructor of the makerbinocular
     * 
     */
    ~makerbinocular();

    /**
     * @brief Init the maker binocular driver
     * 
     * @return void
     */
    void init();

    /**
     * @brief read USB data
     *
     * @return void
     */

    int read_USB_data();

    /**
   * @brief process IMU data
   * @param imuHeaderIndex the index of imu header in the dataBuffer
   *
   * @return succeed or not
   */
    int process_imu_data(int imuHeaderIndex);

    /**
     * @brief Get the transfered image
     * 
     * @param left_image left image of the stereo camera
     * @param right_image right image of the stereo camera
     * @return bool True: get new full image,  false: doesn't get new full image
     */
    bool get_frame(cv::Mat &left_image, cv::Mat &right_image);

    /**
     * @brief Get the flag weather the driver has been initialized
     * 
     * @return bool
     */
    bool is_initialized() { return initialized; }

    /**
     * @brief Get the flag weather a new frame has been received succesfully
     * 
     * @return bool
     */
    bool new_frame_arrived() { return has_new_frame; }


    void reset_frame_flag() {has_new_frame = false; }

private:

    ///*****************************************************************************///
    ///***************************** USB part **************************************///
    ///*****************************************************************************///
    /// libusb device pointer
    libusb_device **devs;
    /// libusb context
    libusb_context *contex = NULL;

    /// libusb device descriptor
    struct libusb_device_descriptor desc;
    /// libusb device configuration
    struct libusb_config_descriptor *config;

    libusb_device *device;
    libusb_device_handle *dev_handle;

    ///*****************************************************************************///
    ///***************************** USB part **************************************///
    ///*****************************************************************************///

    /// time elapsed since last camera frame,  unit is us
    long time_elapsed;
    /// time elapsed since last imu frame,  unit is us
    long imu_time_elpased;
    /// the time elapsed since start of camera,  unit is us
    long current_image_time;
    ///  the time elapsed since the start of imu,  unit is us
    long current_imu_time;

    int current_frame_index;

    double current_expose_time;


    ///*****************************************************************************///
    ///***************************** File Stream  **********************************///
    ///*****************************************************************************///
    /// imu_data.txt filestream
    std::ofstream imu_data_txt;
    /// imu_data_check.csv filestream
    std::ofstream imu_data_check_csv;
    /// image data timestamp filestream
    std:: ofstream image_data_timestamp_txt;


    /// process frame data like a state machine ///
    Process_State_t processState;



    /// the buffer stores the data transfered in a USB bulk transfer event
    unsigned char dataBuffer[4028800];
    /// the value that indicates how many bytes of data still left in dataBuffer
    int dataRem;
    /// might be replaced by dataRem
    int dataLeft;
    /// the value that indicates how many data has been copied into leftImageBuffer or rightImageBuffer
    int dataCopied;
    int read_size;


    /// image raw data buffer
    unsigned char leftImageBuffer[307200];
    unsigned char rightImageBuffer[307200];

    /// IMU part data
    int IMU_sample_count;

    /// frame time stamp
    double frame_time_stamp;

    // flag that indicates if it is the first USB bulk transfer
    bool isFirstXfer;
    unsigned int image_frame_time_stamp;

    /// builk in end point address
    u8 bulk_ep_in;

    // sample 1000 imu datas to estimate tht zero bias
    int imu_init_state;
    double acc_zero_bias[3];
    double gyro_zero_bias[3];

    ///
    bool imu_initialized;
    ///
    float gyro_raw[3];
    float acc_raw[3];

    /// left and right image
    cv::Mat left_image_;
    cv::Mat right_image_;

    pthread_t displayThread;

    bool calibration_mode;
    bool time_start_from_0;

    double time_reset_value;

    int buffer_size;

    bool initialized;
    bool has_new_frame;
    time_t  timev;

    long system_in_ns;
};

#endif
