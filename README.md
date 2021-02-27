## WiFi settings on Jetson Nano (Debian way)

nmcli d wifi list
nmcli device wifi connect <SSID> password <PSK>
nmcli c s <SSID> | grep autoconnect
sudo nmcli c mod <SSID> connection.autoconnect-priority 10
nmcli c s <SSID> | grep priority

## Wiring commands

i2cdetect -r -y 1
sudo /opt/nvidia/jetson-io/jetson-io.py

## Connecting MacOS to Jetson Nano throw UART

1. Connect GND RX TX pins
2. ls /dev/cu.*
3. screen /dev/cu.usbserial-1410 9600
4. screen -S 9465.ttys002.laptop -X quit


cmake -D CMAKE_BUILD_TYPE=RELEASE \
        -D CMAKE_INSTALL_PREFIX=/usr \
        -D OPENCV_EXTRA_MODULES_PATH=~/opencv_contrib/modules \
        -D EIGEN_INCLUDE_PATH=/usr/include/eigen3 \
        -D WITH_CUDA=ON \
        -D CUDA_ARCH_BIN=5.3 \
        -D CUDA_ARCH_PTX="" \
        -D WITH_CUDNN=ON \
        -D WITH_CUBLAS=ON \
        -D ENABLE_FAST_MATH=ON \
        -D CUDA_FAST_MATH=ON \
        -D OPENCV_DNN_CUDA=ON \
        -D ENABLE_NEON=ON \
        -D WITH_QT=OFF \
        -D WITH_OPENMP=ON \
        -D WITH_OPENGL=ON \
        -D BUILD_TIFF=ON \
        -D WITH_FFMPEG=ON \
        -D WITH_GSTREAMER=ON \
        -D WITH_TBB=ON \
        -D BUILD_TBB=ON \
        -D BUILD_TESTS=OFF \
        -D WITH_EIGEN=ON \
        -D WITH_V4L=ON \
        -D WITH_LIBV4L=ON \
        -D OPENCV_ENABLE_NONFREE=ON \
        -D INSTALL_C_EXAMPLES=OFF \
        -D INSTALL_PYTHON_EXAMPLES=OFF \
        -D BUILD_NEW_PYTHON_SUPPORT=ON \
        -D BUILD_opencv_python3=ON \
        -D HAVE_opencv_python3=ON \
        -D PYTHON_DEFAULT_EXECUTABLE=$(which python3) \
        -D OPENCV_GENERATE_PKGCONFIG=ON \
        -D BUILD_EXAMPLES=OFF ..