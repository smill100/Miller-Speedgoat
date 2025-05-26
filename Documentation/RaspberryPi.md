# Steps for Using the Raspberry Pi for Testing
1. Turn on Pi
2. Open command terminal
3. In the command terminal ype g++ filename.cpp -o placeholder 
4. In the command terminal type sudo ./placeholder

In order to use serial communication throught the UART pins, the "sudo ./" command has to be used instead of "./" when starting the file. 
The code is designed to run infinetly and will only perform the necessary tasks once an entire string is read. The code is designed to recieve a string of ASCII characters. With UART communication each ASCII character is sent one at a time. The code reads each character at a time and then stores the characters together. Once the newline character, \n, is read, the code begins executing the tasks to calculate and send new data. This code does not have to be stopped after one test. 

# Detumble and Disturbance
The detumble and disturbance code is configured to receive a 10 space seperated floats as a string of ASCII characters. With the entire string, the code configures the values to floats and stores them as 10 seperate values, which are then put into a function to caluclate the derivatives of the angular velocity, Euler angles, and quaternions. The code then formats the new values as a space seperated string of ASCII characters, with a \n delimeter. Once all of the characters are sent, the code clears the stored variables received and waits for the next string to be read. 

# Kalman Filter
For the purpose of not changing the code too much, the Speedgoat sends a string with the value of "1\n". This is so the Raspberry Pi knows when it is clear to gather and send IMU data. With the entire string received, the code gathers the gyro data from the IMU and then sends it to the Speedgoat for the Kalman filtering. 
