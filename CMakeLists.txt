add_library(CalServo CalServo.cpp)

target_link_libraries(CalServo PCA9685)
target_include_directories(CalServo PUBLIC <path_to_PCA9685_headers>)
