# Yet Another AHRS Implementation for STM32

[![License](https://img.shields.io/badge/License-MIT-blue.svg)](https://opensource.org/licenses/MIT)

## Description

This project provides an implementation of an Attitude and Heading Reference System (AHRS) for the STM32 microcontroller. 
The AHRS algorithm fuses data from various sensors such as accelerometers, gyroscopes, and magnetometers to estimate the orientation (attitude) and heading of the device.

## Features

- Easy scalability
- I2C and SPI IO
- MPU9250/6050 and co. and BMI280 out of the box
- Estimation of device orientation and heading
- Madgwick and Mahony algorithms implemented.
- 6 or 9 dimension of freedom
- Intended to use with a STM32 microcontrollers
- Runtime selectable precision vs performance mode

## Requirements

- STM32 microcontroller based device
- GCC toolchain tested
- I2C or SPI sensors (accelerometer, gyroscope, magnetometer)

## Getting Started

Follow these steps to get started with the AHRS implementation:

1. Clone the repository
2. Configure the project for your STM32 board, add the library and headers
3. Init a library, see examples folder
4. Build the project using the chosen toolchain
5. Flash the generated binary onto your STM32 board
6. Connect the sensors to the appropriate pins on the board
7. Run the AHRS implementation

For detailed instructions on building, configuring, and running the project, please refer to the example folder

## Usage

To use the AHRS implementation in your STM32 project, follow these steps:

Refer to the provided [examples](examples/) for sample code and usage demonstrations.

## Contributing

Contributions to this project are welcome! If you would like to contribute, please follow these guidelines:

1. Fork the repository
2. Create a new branch: `git checkout -b feature/your-feature`
3. Make your changes and commit them: `git commit -am 'Add your feature'`
4. Push to the branch: `git push origin feature/your-feature`
5. Submit a pull request

## License

This project is licensed under the [MIT License](LICENSE).

## Acknowledgements

- [Sebastian Madgwick](https://www.samba.org/tridge/UAV/madgwick_internal_report.pdf) - Thanks to Sebastian Madgwick for his science works.
- [Kris Winer](https://github.com/kriswiner/MPU9250/tree/master) - Thanks to Kris Winer for his inspiration and all information he shared.

