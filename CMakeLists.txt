cmake_minimum_required(VERSION 2.8)

#name the project as maker_binocular
project(maker_binocular)

#find OpenCV and load its settings and variables
#ex. ${OpenCV_LIBS}
find_package(OpenCV REQUIRED)

#add include files 
include_directories(
        ${PROJECT_SOURCE_DIR}/include
        ${OpenCV_INCLUDE_DIRS}
)

#add an executable called binocular_driver from source files main.cpp and maker_binocular.cpp
#that is, compile main.cpp and maker_binocular.cpp to an executable file binocular_driver
add_executable(binocular_driver
        src/main.cpp
        src/maker_binocular.cpp
        src/driver_config.cpp src/driver_config.h)

#link libraries libusb-1.0 and opencv to the execuatble binocular_driver
target_link_libraries(binocular_driver
        pthread
        -lusb-1.0
        ${OpenCV_LIBS}
        )