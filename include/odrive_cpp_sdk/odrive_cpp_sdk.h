#ifndef ODRIVE_SDK_INCLUDE_ODRIVE_SDK_ODRIVESDK_H_
#define ODRIVE_SDK_INCLUDE_ODRIVE_SDK_ODRIVESDK_H_

#include <string>
#include <cstring>
#include <stdio.h>
#include <iostream>
#include <dirent.h>
#include <stdint.h>
#include <fcntl.h>
#include <unistd.h>
#include <algorithm>

#define ODRIVE_SDK_SCAN_SUCCESS 0
#define ODRIVE_SDK_ODRIVE_WITH_SERIAL_NUMBER_NOT_FOUND 1
#define ODRIVE_SDK_SERIAL_NUMBER_MAP_INVALID 2
#define ODRIVE_SDK_UNEXPECTED_RESPONSE 3
#define ODRIVE_SDK_NOT_INITIALIZED 4
#define ODRIVE_SDK_COMM_SUCCESS 0

#define ODRIVE_SDK_SET_GOAL_CMD_PREFIX "p "
#define ODRIVE_SDK_GET_GOAL_0_CMD_PART "0 "
#define ODRIVE_SDK_GET_GOAL_1_CMD_PART "1 "
#define ODRIVE_SDK_GET_GOAL_POSTFIX " 0 0"

#define ODRIVE_SDK_GET_ENCODER_STATE_CMD_PREFIX "g 1 "
#define ODRIVE_SDK_GET_ENCODER_0_STATE "2"
#define ODRIVE_SDK_GET_ENCODER_1_STATE "6"

#define ODRIVE_SDK_GET_MOTOR_ERROR_CMD_PREFIX "g 1 "
#define ODRIVE_SDK_GET_MOTOR_0_ERROR "3"
#define ODRIVE_SDK_GET_MOTOR_1_ERROR "7"
#define ODRIVE_SDK_MOTOR_NO_ERROR_STATUS 0

#define ODRIVE_SDK_GET_SERIALNO_CMD "i"
#define ODRIVE_SDK_GET_SERIALNO_RESULT_LINE_INDEX 3
#define ODRIVE_SDK_GET_SERIALNO_TOTAL_RESULT_LINES 4

#define ODRIVE_SDK_SERIAL_FILE_DIR "/dev"
#define ODRIVE_SDK_LINUX_SERIAL_FILE_PREFIX "ttyACM"
#define ODRIVE_SDK_MAC_SERIAL_FILE_PREFIX "cu.usbmodem"

#define ODRIVE_SDK_GET_CMD_MAX_RESULT_LENGTH 500

namespace odrive
{

    class CppSdk {

    public:
        CppSdk(
               const std::string* odrive_serial_numbers,
               const uint8_t num_odrives,
               const std::string* motor_to_odrive_serial_number_map,
               const bool* motor_position_map, // false = slot 0, true = slot 1
               const float* encoder_ticks_per_radian,
               const uint8_t num_motors
               );
        ~CppSdk();

        int init(); // start communication
        int runCalibration();
        void setZeroethRadianInEncoderTicks(const int16_t* zeroeth_radian_in_encoder_ticks); // assumed to match num_motors
        int setGoalMotorPositions(const double* axes_positions_in_radians_array); // assumed to match num_motors
        int readCurrentMotorPositions(double* axes_positions_in_radians_array); // assumed to match num_motors
        int checkErrors(uint8_t* error_codes_array); // assumed to match num_motors

    private:

        // read settings
        uint8_t num_odrives_;
        uint8_t num_motors_;
        float* encoder_ticks_per_radian_;
        int16_t* zeroeth_radian_in_encoder_ticks_;
        bool* motor_position_map_;

        // saved for use between creation and init
        std::string* odrive_serial_numbers_;
        std::string* motor_to_odrive_serial_number_map_;

        // for serial
        int* odrive_files_;
        std::string* odrive_serial_filenames_;
        uint8_t* motor_to_odrive_file_index_;
        std::string findSerialFileWithSN(std::string& serial_number, const uint8_t odrive_file_index);
    };
}

#endif /* ODRIVE_SDK_INCLUDE_ODRIVE_SDK_ODRIVESDK_H_ */
