#include <iostream>
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <string.h>
#include <sstream>
#include <cmath>
#include <iomanip>
#include <errno.h>

// Detumble now takes output array by reference
void Detumble(const float state0[10], float state[10]) {
    float omega[3] = {state0[0], state0[1], state0[2]};
    float phi = state0[3], theta = state0[4], psi = state0[5];
    float epsilon[3] = {state0[6], state0[7], state0[8]};
    float eta = state0[9];
    
    float T[3] = {-0.2f*omega[0], -0.2f*omega[1], -0.2f*omega[2]};
        
    float J[3][3] = {
        {426.6667, 0, 0}, 
        {0, 426.6667, 0}, 
        {0, 0, 426.6667}
        };
        
    float invJ[3][3] = {
        {0.002343749816895, 0, 0}, 
        {0, 0.002343749816895, 0}, 
        {0, 0, 0.002343749816895}
        };
        
    float wx[3][3] = {
        {0, -omega[2], omega[1]}, 
        {omega[2], 0, -omega[0]}, 
        {-omega[1], omega[0], 0}
        };
    
    float wx_J_w[3] = {};
    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 3; ++j) {
            wx_J_w[i] += wx[i][j]*J[j][i]*omega[j];
        }
    }
    
    
    float T_wxJw[3] = {T[0] - wx_J_w[0], T[1] - wx_J_w[1], T[2] - wx_J_w[2]};
    
    float omega_dot[3] = {};
    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 3; ++j) {
            omega_dot[i] += invJ[i][j]*T_wxJw[j];
        }
    }
    
    float eta_epx[3][3] = {
        {eta, -epsilon[2], epsilon[1]},
        {epsilon[2], eta, -epsilon[0]},
        {-epsilon[1], epsilon[0], eta}
        };
    
    float epsilon_dot[3] = {};
    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 3; ++j) {
            epsilon_dot[i] += 0.5*eta_epx[i][j]*omega[j];
        }
    }
    
    float eta_dot = 0;
    for (int i = 0; i < 3; ++i) {
        eta_dot += epsilon[i]*omega[i];
    }
    
    eta_dot = -0.5*eta_dot;
    
    float E[3][3] = {
        {1.0f, sinf(phi)*tanf(theta), cosf(phi)*tanf(theta)},
        {0.0f, cosf(phi), -sinf(phi)},
        {0.0f, sinf(phi)*(1/cosf(theta)), cosf(phi)*(1/cosf(theta))}
        };
        
    float E_dot[3] = {};
    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 3; ++j) {
            E_dot[i] += E[i][j]*omega[j];
        }
    }
    
    for (int i = 0; i < 3; ++i) {
        state[i] = omega_dot[i];
    }

    for (int i = 0; i < 3; ++i) {
        state[3 + i] = E_dot[i];
    }
    for (int i = 0; i < 3; ++i) {
        state[6 + i] = epsilon_dot[i];
    }
        state[9] = eta_dot;
}

int main() {
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
                float w1, w2, w3, phi, theta, psi, q1, q2, q3, eta;
                std::stringstream ss(message);
                if (!(ss >> w1 >> w2 >> w3 >> phi >> theta >> psi >> q1 >> q2 >> q3 >> eta)) {
                    std::cout << "Parse failed on: " << message << std::endl;
                } else {
                    float state0[10] = {w1, w2, w3, phi, theta, psi, q1, q2, q3, eta};
                    float state[10];
                    Detumble(state0, state);
                    
                    std::ostringstream tt;
                    tt << std::fixed << std::setprecision(6);
                    for (int i = 0; i < 10; ++i) {
                        tt << state[i];
                        if (i < 9) tt << " ";
                    }
                    tt << "\n";

                    std::string data = tt.str();
                    std::cout << data;
                    if (write(serial_port, data.c_str(), data.length()) < 0) {
                        std::cout << "Failed to write data back.\n";
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
