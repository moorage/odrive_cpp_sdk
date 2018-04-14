#include "odrive_cpp_sdk/odrive_cpp_sdk.h"

using namespace odrive;

CppSdk::CppSdk(const std::string* odrive_serial_numbers,
               const uint8_t num_odrives,
               const std::string* motor_to_odrive_serial_number_map,
               const bool* motor_position_map, // false = slot 0, true = slot 1
               const float* encoder_ticks_per_radian,
               const uint8_t num_motors) {
    // read settings
    num_odrives_ = num_odrives;
    num_motors_ = num_motors;
    encoder_ticks_per_radian_ = new float[num_motors];
    for (uint8_t i = 0; i < num_motors; ++i) {
        encoder_ticks_per_radian_[i] = encoder_ticks_per_radian[i];
    }
    zeroeth_radian_in_encoder_ticks_ = new int16_t[num_motors];
    for (uint8_t i = 0; i < num_motors; ++i) {
        zeroeth_radian_in_encoder_ticks_[i] = 0;
    }
    motor_position_map_ = new bool[num_motors];
    for (uint8_t i = 0; i < num_motors; ++i) {
        motor_position_map_[i] = motor_position_map[i];
    }

    // saved for use between creation and init
    odrive_serial_numbers_ = new std::string[num_odrives];
    for (uint8_t i = 0; i < num_odrives; ++i) {
        odrive_serial_numbers_[i].assign(odrive_serial_numbers[i]);
    }
    motor_to_odrive_serial_number_map_ = new std::string[num_motors];
    for (uint8_t i = 0; i < num_motors; ++i) {
        motor_to_odrive_serial_number_map_[i].assign(motor_to_odrive_serial_number_map[i]);
    }

    odrive_serial_filenames_ = NULL;
    odrive_files_ = NULL;
    motor_to_odrive_file_index_ = NULL;
}

CppSdk::~CppSdk() {
    if (motor_to_odrive_file_index_) {
        for (uint8_t i = 0; i < num_odrives_; ++i) {
            if (odrive_files_[i] && odrive_files_[i] > 0) {
                if (close(odrive_files_[i]) < 0) {
                    std::cerr << "Couldn't close `" << odrive_serial_filenames_[i] << "': " << strerror(errno) << "` (" << errno << ")"  << std::endl;
                }
                odrive_files_[i] = -1;
            }
        }
    }

    if (encoder_ticks_per_radian_) delete [] encoder_ticks_per_radian_;
    if (zeroeth_radian_in_encoder_ticks_) delete [] zeroeth_radian_in_encoder_ticks_;
    if (motor_position_map_) delete [] motor_position_map_;
    if (odrive_serial_numbers_) delete [] odrive_serial_numbers_;
    if (motor_to_odrive_serial_number_map_) delete [] motor_to_odrive_serial_number_map_;
    if (odrive_serial_filenames_) delete [] odrive_serial_filenames_;
    if (odrive_files_) delete [] odrive_files_;
    if (motor_to_odrive_file_index_) delete [] motor_to_odrive_file_index_;
}

int CppSdk::init() {
    // for serial
    odrive_serial_filenames_ = new std::string[num_odrives_];

    odrive_files_ = new int[num_odrives_];
    for (uint8_t i = 0; i < num_odrives_; ++i) {
        std::string file_name = findSerialFileWithSN(odrive_serial_numbers_[i], i);

        if (file_name.empty()) {
            return ODRIVE_SDK_ODRIVE_WITH_SERIAL_NUMBER_NOT_FOUND;
        }
        odrive_files_[i] = open(file_name.c_str(), O_RDWR | O_NOCTTY);
        if (odrive_files_[i] < 0) {
            std::cerr << "Could not open '" << file_name.c_str() << "': `" << strerror(errno) << "` (" << errno << ")" << std::endl;
            continue;
        }
    }

    motor_to_odrive_file_index_ = new uint8_t[num_motors_];
    for (uint8_t i = 0; i < num_motors_; ++i) {
        long sn_index = std::distance(odrive_serial_numbers_, std::find(odrive_serial_numbers_, odrive_serial_numbers_ + num_odrives_, motor_to_odrive_serial_number_map_[i]));
        if (sn_index < 0 || sn_index >= num_motors_) {
            return ODRIVE_SDK_SERIAL_NUMBER_MAP_INVALID;
        }
        motor_to_odrive_file_index_[i] = (uint8_t) sn_index;
    }

    return ODRIVE_SDK_COMM_SUCCESS;
}

void CppSdk::setZeroethRadianInEncoderTicks(const int16_t* zeroeth_radian_in_encoder_ticks) {
    for (uint8_t i = 0; i < num_motors_; ++i) {
        zeroeth_radian_in_encoder_ticks_[i] = zeroeth_radian_in_encoder_ticks[i];
    }
}

int CppSdk::runCalibration(){ return -1; }

int CppSdk::setGoalMotorPositions(const double* axes_positions_in_radians_array) {
    // FIXME TEMP what radians is "home"?
    std::string cmd;
    uint8_t file_index;
    ssize_t io_result;
    for (uint8_t i = 0; i < num_motors_; ++i) {
        cmd.clear();
        int position_in_ticks = zeroeth_radian_in_encoder_ticks_[i] + (axes_positions_in_radians_array[i] * encoder_ticks_per_radian_[i]);

        if (! motor_to_odrive_file_index_) {
            return ODRIVE_SDK_NOT_INITIALIZED;
        }

        file_index = motor_to_odrive_file_index_[i];
        cmd = ODRIVE_SDK_SET_GOAL_CMD_PREFIX;
        cmd.append(motor_position_map_[i] ? ODRIVE_SDK_GET_GOAL_1_CMD_PART : ODRIVE_SDK_GET_GOAL_0_CMD_PART).append(std::to_string(position_in_ticks)).append(ODRIVE_SDK_GET_GOAL_POSTFIX); // TODO use last two commands

        io_result = write(odrive_files_[file_index], cmd.c_str(), cmd.size());
        if (io_result != cmd.size()) {
            std::cerr << "Couldn't send `" << cmd << "` to '" << odrive_serial_filenames_[i] << "': `" << strerror(errno) << "` (" << errno << ")" << std::endl;
            return ODRIVE_SDK_UNEXPECTED_RESPONSE;
        }
    }

    return ODRIVE_SDK_COMM_SUCCESS;
}

int CppSdk::readCurrentMotorPositions(double* axes_positions_in_radians_array) {
    std::string cmd;
    uint8_t file_index;
    ssize_t io_result;
    char read_serial_buf[ODRIVE_SDK_GET_CMD_MAX_RESULT_LENGTH];
    for (uint8_t i = 0; i < num_motors_; ++i) {
        cmd.clear();
        cmd = "";
        cmd.append(ODRIVE_SDK_GET_ENCODER_STATE_CMD_PREFIX).append(motor_position_map_[i] ? ODRIVE_SDK_GET_ENCODER_1_STATE : ODRIVE_SDK_GET_ENCODER_0_STATE);

        if (! motor_to_odrive_file_index_) {
            return ODRIVE_SDK_NOT_INITIALIZED;
        }

        file_index = motor_to_odrive_file_index_[i];

        io_result = write(odrive_files_[file_index], cmd.c_str(), cmd.size());
        if (io_result != cmd.size()) {
            std::cerr << "Couldn't send `" << cmd << "` to '" << odrive_serial_filenames_[i] << "': `" << strerror(errno) << "` (" << errno << ")" << std::endl;
            return ODRIVE_SDK_UNEXPECTED_RESPONSE;
        }

        io_result = read(odrive_files_[file_index], read_serial_buf, ODRIVE_SDK_GET_CMD_MAX_RESULT_LENGTH);
        if (io_result < 0) {
            std::cerr << "Couldn't read response from '" << odrive_serial_filenames_[i] << "': " << strerror(errno) << "` (" << errno << ")"  << std::endl;
            return ODRIVE_SDK_UNEXPECTED_RESPONSE;
        }

        try {
            axes_positions_in_radians_array[i] =  (std::stoi(read_serial_buf) / (double)encoder_ticks_per_radian_[i]) - (zeroeth_radian_in_encoder_ticks_[i] / (double)encoder_ticks_per_radian_[i]); // TODO Check math
        } catch(std::invalid_argument e1 ) {
            return ODRIVE_SDK_UNEXPECTED_RESPONSE;
        } catch( std::out_of_range e2 ) {
            return ODRIVE_SDK_UNEXPECTED_RESPONSE;
        }
    }
    return ODRIVE_SDK_COMM_SUCCESS;
}

int CppSdk::checkErrors(uint8_t* error_codes_array) {
    std::string cmd;
    uint8_t file_index;
    ssize_t io_result;
    char read_serial_buf[ODRIVE_SDK_GET_CMD_MAX_RESULT_LENGTH];
    for (uint8_t i = 0; i < num_motors_; ++i) {
        cmd.clear();
        cmd = "";
        cmd.append(ODRIVE_SDK_GET_MOTOR_ERROR_CMD_PREFIX).append(motor_position_map_[i] ? ODRIVE_SDK_GET_MOTOR_1_ERROR : ODRIVE_SDK_GET_MOTOR_0_ERROR);

        if (! motor_to_odrive_file_index_) {
            return ODRIVE_SDK_NOT_INITIALIZED;
        }

        file_index = motor_to_odrive_file_index_[i];

        io_result = write(odrive_files_[file_index], cmd.c_str(), cmd.size());
        if (io_result != cmd.size()) {
            std::cerr << "Couldn't send `" << cmd << "` to '" << odrive_serial_filenames_[i] << "': `" << strerror(errno) << "` (" << errno << ")" << std::endl;
            return ODRIVE_SDK_UNEXPECTED_RESPONSE;
        }

        io_result = read(odrive_files_[file_index], read_serial_buf, ODRIVE_SDK_GET_CMD_MAX_RESULT_LENGTH);
        if (io_result < 0) {
            std::cerr << "Couldn't read response from '" << odrive_serial_filenames_[i] << "': " << strerror(errno) << "` (" << errno << ")"  << std::endl;
            return ODRIVE_SDK_UNEXPECTED_RESPONSE;
        }

        try {
            error_codes_array[i] = (uint8_t) std::stoi(read_serial_buf);
        } catch(std::invalid_argument e1 ) {
            return ODRIVE_SDK_UNEXPECTED_RESPONSE;
        } catch( std::out_of_range e2 ) {
            return ODRIVE_SDK_UNEXPECTED_RESPONSE;
        }
    }
    return ODRIVE_SDK_COMM_SUCCESS;
}

// side effect: writes to odrive_serial_filenames_ if match
std::string CppSdk::findSerialFileWithSN(std::string& serial_number, const uint8_t odrive_file_index) {
    if (!odrive_serial_filenames_[odrive_file_index].empty()) {
        return odrive_serial_filenames_[odrive_file_index];
    }

    std::string end_of_sn_string = " " + serial_number + "\n";
    char read_serial_buf[ODRIVE_SDK_GET_CMD_MAX_RESULT_LENGTH];
    ssize_t io_result;
    int serial_file;
    long filename_index;

    DIR *dp;
    struct dirent *ep;
    std::string dname_str;
    std::string serial_filename;
    std::string current_serial_number;
    std::string serial_cmd("");
    serial_cmd.append(ODRIVE_SDK_GET_SERIALNO_CMD);
    std::string linux_prefix(ODRIVE_SDK_LINUX_SERIAL_FILE_PREFIX);
    std::string mac_prefix(ODRIVE_SDK_MAC_SERIAL_FILE_PREFIX);
    dp = opendir(ODRIVE_SDK_SERIAL_FILE_DIR);
    if (dp != NULL) {
        while ( (ep = readdir(dp)) ) {
            if (ep->d_name[0] == '.') continue;

            dname_str.assign(ep->d_name);
            if (dname_str.compare(0, linux_prefix.length(), linux_prefix) == 0 || dname_str.compare(0, mac_prefix.length(), mac_prefix) == 0) {
                // Ignore if we've already opened this
                filename_index = std::distance(odrive_serial_filenames_, std::find(odrive_serial_filenames_, odrive_serial_filenames_ + num_odrives_, dname_str));
                if (filename_index >= 0 && filename_index < num_odrives_) {
                    continue; // already found & checked this odrive
                }

                // open and check serial number
                serial_filename.clear();
                serial_filename = ODRIVE_SDK_SERIAL_FILE_DIR;
                serial_filename.append("/").append(dname_str);

                // this uses the C-based fopen instead of C++ std::fstream because of hanging on macOS
                serial_file = open(serial_filename.c_str(), O_RDWR | O_NOCTTY);
                if (serial_file < 0) {
                    std::cerr << "Could not open '" << serial_filename.c_str() << "': `" << strerror(errno) << "` (" << errno << ")" << std::endl;
                    continue;
                }

                io_result = write(serial_file, serial_cmd.c_str(), serial_cmd.size());
                if (io_result != serial_cmd.size()) {
                    std::cerr << "Couldn't send `" << serial_cmd << "` to '" << serial_filename.c_str() << "': `" << strerror(errno) << "` (" << errno << ")" << std::endl;
                    continue;
                }

                current_serial_number.clear();
                for (int i = 0; i < ODRIVE_SDK_GET_SERIALNO_TOTAL_RESULT_LINES; ++i) {
                    io_result = read(serial_file, read_serial_buf, ODRIVE_SDK_GET_CMD_MAX_RESULT_LENGTH);
                    if (io_result < 0) {
                        std::cerr << "Couldn't read response from '" << serial_filename.c_str() << "': " << strerror(errno) << "` (" << errno << ")"  << std::endl;
                        continue;
                    }

                    if (i == ODRIVE_SDK_GET_SERIALNO_RESULT_LINE_INDEX) {
                        current_serial_number.assign(read_serial_buf);
                    }
                }

                if (close(serial_file) < 0) {
                    std::cerr << "Couldn't close '" << serial_filename.c_str() << "': " << strerror(errno) << "` (" << errno << ")"  << std::endl;
                }
                serial_file = -1;

                // if this device is the serial number, save to odrive_serial_filenames_ and return full path
                if (0 == current_serial_number.compare(current_serial_number.length() - end_of_sn_string.length(), end_of_sn_string.length(), end_of_sn_string)) {
                    odrive_serial_filenames_[odrive_file_index] = dname_str;
                    return serial_filename;
                }
            }
        }

    }


    return std::string(); // not found
}
