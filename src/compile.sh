g++ -std=gnu++11 main.cpp KinectAPI.hpp -I/home/migly/OpenNI-Linux-x64-2.2/Include -I/home/migly/NiTE-Linux-x64-2.2/Include -I/opt/ros/kinetic/include -L/home/migly/OpenNI-Linux-x64-2.2/Redist -L/home/migly/NiTE-Linux-x64-2.2/Redist -lOpenNI2 -lNiTE2 -lglut -lGL `pkg-config --cflags opencv` `pkg-config --libs opencv` -Wl,-rpath ./
cd ./Bin/x64-Release/
./a.out
