# Testing
The Speedgoat and MATLAB need to have matching versions. For this thesis, the Speedgoat was updated to use MATLAB 2020a. There are tutorial videos on the Speedgoat website: https://www.speedgoat.com/knowledge-center?tag=how%20to 
In order to access the videos for updating the Speedgoat, an account is required and you need the speedgoat serial number to create an account.

To begin testing, the target PC needs MATLAB/Simulink and the support packages below. These support packages can be downloaded in Matlab.
1. MATLAB coder
2. Simulink coder
3. Simulink real-time
4. Simulink real-time target support

In addition to these packages, a c++ compiler is required. This compiler needs to match the version of MATLAB being used on the host PC and Speedgoat. Here is information on it: https://www.mathworks.com/support/requirements/supported-compilers.html you just have to make sure the specific MATLAB year works with the specific compiler you're using.
I used MATLAB 2020a and microsoft visual C++ 2017, as those two are compatible. IMPORTANT: if you are using MATLAB with a computer that has a compiler that isn't compatible with that year, then there will be errors with Simulink real-time. Make sure you uninstall non-compatible versions.

Speedgoat has a blockset library that needs to be downloaded and installed in Matlab. The process involves downloading the correct version and placing it in a Matlab folder. The download is a .zip file, once it is in matlab you can unzip it and find the file speedgoat_setup.p. Then type speedgoat_setup in the MATLAB command window.

The connection and setup can be tested by using the command slrttest in the MATLAB command window. 8 tests will be run and a status of pass or fail will be displayed for each test based on if it passed or failed.

Steps to Testing:
1. Turn on Raspberry Pi
2. Open MATLAB
3. Run the .m code if doing the disturbance or detumble test
4. Open the Simulink models on the host pc
5. Make sure host PC is connected to the Speedgoat via ethernet
6. Turn on Speedgoat
7. Open terminal on Raspberry Pi and run the .cpp code
8. In the Simulink model go to the real-time tab and make sure the target PC is connected.
   If it says disconnected, click the disconnected button and wait for it to say connected.
9. Once connected you can hit the run on target button and the test will run

# Explination of Real-Time Simulink
To make a simulink model that can be run on the Speedgoat, the real-time app in Simulink can be used and will configure everything so that a Speedgoat can be used. When first connecting to the Speedgoat through Simulink, you may need to add the IP address. This can be found when you turn on the Speedgoat and have a monitor plugged in. This information and other information will be shown and are important for making sure the connection between the host PC and Speedgoat is working correctly.

# Tips
The monitor attached to the Speedgoat will display statuses when a test is being run. If there is a error in compiling the simulink model to C, the Simulink will show an error on the host PC. If its an error while the test is running, then an error will display on the Speedgoat monitor. When using the RS232 serial send receive block, if there is an error in sending/receiving, then there will be a specfic error. It will display an error saying receive fifo full or transmit fifo full. This means more data is going in than coming out. Try changing the step size of the model to be higher.

Do not use scope blocks in the simulink model. This can cause issues with the host PC and Speedgoat and cause them to crash. Instead use data inspector by logging the signals.

You can compare the data of past runs, as long as you don't close MATLAB, in data inspector. This helps with troubleshooting and seeing the differences between different solvers. In data inspector you can use a data cursor to see the exact value at a given time step. You can use the data inspector to export the data from a run to a .mat or .csv file in your current folder path. 
