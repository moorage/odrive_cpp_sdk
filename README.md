# Odrive CPP SDK

This is the simplest C++ implementation of ODrive communication methods possible.

It requires `ascii` and the older `ascii` api flashed on the odrive in your `tup.config`, like such:

```
CONFIG_BOARD_VERSION=v3.4-24V
CONFIG_USB_PROTOCOL=ascii
CONFIG_UART_PROTOCOL=ascii
CONFIG_STEP_DIR=n
```

The simplest possible c++ program to run it, as an example (assuming 2 motors), is:

```cpp
#include <iostream>
#include "odrive_cpp_sdk.h"

int main(int argc, const char * argv[]) {
    std::string odrive_serial_numbers[1] = {"0x303339373235510a330045"};
    std::string odrive_serial_numbers_map[2] = {"0x303339373235510a330045","0x303339373235510a330045"};
    int16_t zeroeth_radian_in_encoder_ticks_[2] = { -2, 0 };

    bool odrive_position_per_motor[2] = {false, true};
    float odrive_encoder_ticks_per_radian_per_motor[2] = { 57.2958 * (2048 * 4) / 360.0, 57.2958 * (2048 * 4) / 360.0 };
    odrive::CppSdk odrive_cpp_sdk(
        odrive_serial_numbers,
        1,
        odrive_serial_numbers_map,
        odrive_position_per_motor,
        odrive_encoder_ticks_per_radian_per_motor,
        2
    );
    std::cout << "odrive_cpp_sdk constructed" << std::endl;

    odrive_cpp_sdk.setZeroethRadianInEncoderTicks(zeroeth_radian_in_encoder_ticks_);

    int result = odrive_cpp_sdk.init();
    std::cout << "odrive_cpp_sdk.init got: " << result << std::endl;

    if (result == ODRIVE_SDK_ODRIVE_WITH_SERIAL_NUMBER_NOT_FOUND) {
        std::cout << "odrive_cpp_sdk.init :: ODRIVE_SDK_ODRIVE_WITH_SERIAL_NUMBER_NOT_FOUND" << std::endl;
        return EXIT_FAILURE;
    }

    uint8_t odrive_motor_current_errors[2] = {0};
    result = odrive_cpp_sdk.checkErrors(odrive_motor_current_errors);
    std::cout << "odrive_cpp_sdk.checkErrors got result:" << result << " and value: [" << std::to_string(odrive_motor_current_errors[0]) << "," << std::to_string(odrive_motor_current_errors[1]) << "]" << std::endl;

    double odrive_motor_current_positions[2] = {0};
    result = odrive_cpp_sdk.readCurrentMotorPositions(odrive_motor_current_positions);
    std::cout << "odrive_cpp_sdk.readCurrentMotorPositions got result:" << result << " and value: [" << std::to_string(odrive_motor_current_positions[0]) << "," << std::to_string(odrive_motor_current_positions[1]) << "]" << std::endl;

    double odrive_motor_cmd_positions[2] = { 3.14159 / 2.0, 3.14159 / 2.0 };
    result = odrive_cpp_sdk.setGoalMotorPositions(odrive_motor_cmd_positions);
    std::cout << "odrive_cpp_sdk.setGoalMotorPositions got result:" << result << std::endl;

    odrive_motor_current_errors[0] = 0;
    odrive_motor_current_errors[1] = 0;
    result = odrive_cpp_sdk.checkErrors(odrive_motor_current_errors);
    std::cout << "odrive_cpp_sdk.checkErrors got result:" << result << " and value: [" << std::to_string(odrive_motor_current_errors[0]) << "," << std::to_string(odrive_motor_current_errors[1]) << "]" << std::endl;

    odrive_motor_current_positions[0] = 0;
    odrive_motor_current_positions[1] = 0;
    result = odrive_cpp_sdk.readCurrentMotorPositions(odrive_motor_current_positions);
    std::cout << "odrive_cpp_sdk.readCurrentMotorPositions got result:" << result << " and value: [" << std::to_string(odrive_motor_current_positions[0]) << "," << std::to_string(odrive_motor_current_positions[1]) << "]" << std::endl;
}
```
