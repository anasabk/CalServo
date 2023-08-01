if(ESP_PLATFORM)
    set(SOURCES CalServo.cpp)

    idf_component_register(
        SRCS ${SOURCES}
        INCLUDE_DIRS . 
        REQUIRES PCA9685
    )

else(ESP_PLATFORM)
    add_library(CalServo CalServo.cpp CalServo.h)

    target_link_libraries(CalServo PCA9685)
    target_include_directories(CalServo PUBLIC .)
endif(ESP_PLATFORM)
