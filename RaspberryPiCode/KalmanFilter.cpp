#include <iostream>
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <linux/i2c-dev.h>
#include <cstdint>
#include <termios.h>
#include <string.h>
#include <sstream>
#include <cmath>
#include <iomanip>
#include <errno.h>

#define LSM6DS3_ADDRESS 0x6A
#define LSM6DS3_CHIP_ID 106
#define WHO_AM_I_REG    0x0F
#define MASTER_CONFIG   0x1A
#define CTRL1_XL        0x10
#define CTRL2_G         0x11

// Wiring for the IMU Black = ground; red = 3.3v; Blue = SDA; Yellow = SCL;

struct IMUData {
    float accel_x, accel_y, accel_z;
    float gyro_x, gyro_y, gyro_z;
};

class LSM6DS3 {
public:
    LSM6DS3(const char* i2c_device = "/dev/i2c-1");
    bool begin();
    void enableI2CMasterPullups(bool enable);
    bool readAccel(IMUData& data);
    bool readGyro(IMUData& data);

private:
    int i2c_fd;
    const char* device;
    bool writeRegister(uint8_t reg, uint8_t value);
    bool readRegister(uint8_t reg, uint8_t& value);
};

LSM6DS3::LSM6DS3(const char* i2c_device) : device(i2c_device), i2c_fd(-1) {}

bool LSM6DS3::begin() {
    i2c_fd = open(device, O_RDWR);
    if (i2c_fd < 0) {
        std::cerr << "Failed to open I2C device\n";
        return false;
    }

    if (ioctl(i2c_fd, I2C_SLAVE, LSM6DS3_ADDRESS) < 0) {
        std::cerr << "Failed to set I2C address\n";
        return false;
    }

    uint8_t chip_id = 0;
    if (!readRegister(WHO_AM_I_REG, chip_id)) {
        std::cerr << "Failed to read chip ID\n";
        return false;
    }

    if (chip_id != LSM6DS3_CHIP_ID) {
        std::cerr << "Unexpected chip ID: " << static_cast<int>(chip_id) << "\n";
        return false;
    }

    // Configure accelerometer (1.66 kHz, ±2g)
    writeRegister(CTRL1_XL, 0x60);

    // Configure gyroscope (1.66 kHz, ±245 dps)
    writeRegister(CTRL2_G, 0x60);

    std::cout << "LSM6DS3 initialized successfully!\n";
    return true;
}

void LSM6DS3::enableI2CMasterPullups(bool enable) {
    uint8_t reg = 0;
    if (readRegister(MASTER_CONFIG, reg)) {
        if (enable)
            reg |= (1 << 3);
        else
            reg &= ~(1 << 3);

        writeRegister(MASTER_CONFIG, reg);
    }
}

bool LSM6DS3::readAccel(IMUData& data) {
    uint8_t buffer[6];
    uint8_t start_reg = 0x28;

    if (write(i2c_fd, &start_reg, 1) != 1) return false;
    if (read(i2c_fd, buffer, 6) != 6) return false;

    int16_t raw_x = (int16_t)(buffer[1] << 8 | buffer[0]);
    int16_t raw_y = (int16_t)(buffer[3] << 8 | buffer[2]);
    int16_t raw_z = (int16_t)(buffer[5] << 8 | buffer[4]);

    const float accel_sensitivity = 0.061f; // mg/LSB for ±2g
    data.accel_x = raw_x * accel_sensitivity / 1000.0f;
    data.accel_y = raw_y * accel_sensitivity / 1000.0f;
    data.accel_z = raw_z * accel_sensitivity / 1000.0f;

    return true;
}

bool LSM6DS3::readGyro(IMUData& data) {
    uint8_t buffer[6];
    uint8_t start_reg = 0x22;

    if (write(i2c_fd, &start_reg, 1) != 1) return false;
    if (read(i2c_fd, buffer, 6) != 6) return false;

    int16_t raw_x = (int16_t)(buffer[1] << 8 | buffer[0]);
    int16_t raw_y = (int16_t)(buffer[3] << 8 | buffer[2]);
    int16_t raw_z = (int16_t)(buffer[5] << 8 | buffer[4]);

    const float gyro_sensitivity = 8.75f; // mdps/LSB for ±245 dps
    data.gyro_x = raw_x * gyro_sensitivity / 1000.0f;
    data.gyro_y = raw_y * gyro_sensitivity / 1000.0f;
    data.gyro_z = raw_z * gyro_sensitivity / 1000.0f;

    return true;
}

bool LSM6DS3::readRegister(uint8_t reg, uint8_t& value) {
    if (write(i2c_fd, &reg, 1) != 1) return false;
    if (read(i2c_fd, &value, 1) != 1) return false;
    return true;
}

bool LSM6DS3::writeRegister(uint8_t reg, uint8_t value) {
    uint8_t buffer[2] = { reg, value };
    return (write(i2c_fd, buffer, 2) == 2);
}

// === MAIN TEST ===

int main() {
    LSM6DS3 imu;
    if (!imu.begin()) return 1;

    imu.enableI2CMasterPullups(true);

    IMUData data;
    
    const char* portname = "/dev/serial0";
    int serial_port = open(portname, O_RDWR | O_NOCTTY);
    if (serial_port < 0) {
        std::cerr << "Error opening " << portname << ": " << strerror(errno) << std::endl;
        return 1;
    }

    struct termios tty;
    if (tcgetattr(serial_port, &tty) != 0) {
        std::cerr << "Error getting terminal attributes: " << strerror(errno) << std::endl;
        return 1;
    }

    cfsetispeed(&tty, B115200);
    cfsetospeed(&tty, B115200);
    tty.c_cflag &= ~PARENB;
    tty.c_cflag &= ~CSTOPB;
    tty.c_cflag &= ~CSIZE;
    tty.c_cflag |= CS8;
    tty.c_cflag &= ~CRTSCTS;
    tty.c_cflag |= CREAD | CLOCAL;
    tty.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
    tty.c_iflag &= ~(IXON | IXOFF | IXANY);
    tty.c_oflag &= ~OPOST;

    tcsetattr(serial_port, TCSANOW, &tty);
    tcflush(serial_port, TCIFLUSH);

    std::string message;
        for (;;) {
            char ch;
            int n = read(serial_port, &ch, 1);
            if (n > 0) {
                if (ch == '\n') {
                    float control;
                    std::stringstream ss(message);
                
                    if (!(ss >> control )) {
                        std::cout << "Parse failed on: " << message << std::endl;
                    } else {
                        if (imu.readGyro(data)) {

                            std::ostringstream tt;
                            tt << std::fixed << std::setprecision(4);
            
                            tt << data.gyro_x << " " << data.gyro_y << " " << data.gyro_z << "\n";

                            std::string transfer = tt.str();
                            //std::cout << transfer;
                            if (write(serial_port, transfer.c_str(), transfer.length()) < 0) {
                                std::cout << "Failed to write data back.\n";
                            }
                            usleep(1000);
                        } else {
                            std::cout << "Failed to read IMU data.\n";
                        }
                    }   

                    message.clear();
                } else {
                    message += ch;
                }
            }
        }

    return 0;
}
