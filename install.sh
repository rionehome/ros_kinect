sudo apt-get install git g++ cmake libxi-dev libxmu-dev libusb-1.0-0-dev pkg-config freeglut3-dev build-essential

cd

#install libfreenect
git clone https://github.com/OpenKinect/libfreenect.git
cd libfreenect
mkdir build; cd build
cmake .. -DBUILD_OPENNI2_DRIVER=ON
make
sudo make install

sudo mkdir -p /etc/udev/rules.d/
sudo cp ../platform/linux/udev/51-kinect.rules /etc/udev/rules.d/

#install Openni2
cd
wget https://s3.amazonaws.com/com.occipital.openni/OpenNI-Linux-x64-2.2.0.33.tar.bz2

tar -xjvf OpenNI-Linux-x64-2.2.0.33.tar.bz2 && rm OpenNI-Linux-x64-2.2.0.33.tar.bz2
cd OpenNI-Linux-x64-2.2
sudo ./install.sh

#Test installation of Openni2
cd

cp libfreenect/build/lib/OpenNI2-FreenectDriver/libFreenectDriver.so OpenNI-Linux-x64-2.2/Redist/OpenNI2/Drivers/
cp libfreenect/build/lib/OpenNI2-FreenectDriver/libFreenectDriver.so OpenNI-Linux-x64-2.2/Tools/OpenNI2/Drivers/

sudo apt-get install usbutils
#lsusb #You should see kinect devices

#Install NiTE2
cd

wget http://jaist.dl.sourceforge.net/project/roboticslab/External/nite/NiTE-Linux-x64-2.2.tar.bz2

tar xjvf NiTE-Linux-x64-2.2.tar.bz2 && rm NiTE-Linux-x64-2.2.tar.bz2

cd NiTE-Linux-x64-2.2
sudo ./install.sh

cd  ..
cp libfreenect/build/lib/OpenNI2-FreenectDriver/libFreenectDriver.so NiTE-Linux-x64-2.2/Samples/Bin/OpenNI2/Drivers/
cp OpenNI-Linux-x64-2.2/Redist/libOpenNI2.so NiTE-Linux-x64-2.2/Samples/Bin
