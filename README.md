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

## Setup SSH keychain

ssh-copy-id -i ~/.ssh/personal.rsa.pub droid

## Disable GUI and enable 15W power mode

sudo systemctl set-default multi-user.target
sudo systemctl set-default graphical.target
sudo systemctl start gdm3.service

## Sync source code to edge device

git clone https://github.com/akhmedov/ground-droid
rsync -a --exclude-from={'.*'} ../droid/ droid:ground-droid