
![white background](https://github.com/user-attachments/assets/b1138b20-78b6-4019-8555-94abfe7d106e)
![RP0](https://github.com/user-attachments/assets/c0970898-2d92-4a27-a1f0-996b5a072682)

Download Raspberry Pi Imager and inslall OS on Raspberry Pi

(Attention):
Install legacy Raspbian Bullseye 64-bit
https://www.raspberrypi.com/software/

Connect to Raspberry Pi, open Terminal and type all commands

Update Raspbian:
```
sudo apt update
sudo apt -y upgrade
sudo apt install -y python3-dev python3-opencv
```
Install apps:
```
sudo rm /usr/lib/python3.11/EXTERNALLY-MANAGED
pip3 install Cython
pip3 install pyserial
pip3 install dronekit
pip3 install MAVProxy
```
To work dronekit in python from 3.10 you need to
```
nano +2689 ~/.local/lib/python3.11/site-packages/dronekit/__init__.py
change collections.MutableMapping on collections.abc.MuttableMapping
```
Create starter file:
```
touch ~/starter.sh
echo '#!/bin/bash' >> ~/starter.sh
echo  >> ~/starter.sh
echo 'python ~/dronekit-python/tracker_betaflight.py' >> ~/starter.sh
chmod 755 ~/starter.sh
```
Change window manager (mutter to openbox-lxde):
```
sudo sed -i "s/mutter/openbox-lxde/g" /etc/xdg/lxsession/LXDE-pi/desktop.conf
cp -rf /etc/xdg/openbox/ ~/.config/
```
Autostart:
```
mkdir -p ~/.config/lxsession/LXDE-pi/
cp /etc/xdg/lxsession/LXDE-pi/* ~/.config/lxsession/LXDE-pi/
echo "/home/`whoami`/starter.sh" >> ~/.config/lxsession/LXDE-pi/autostart
```
Screensaver:
```
sed -i /xscreensaver/d ~/.config/lxsession/LXDE-pi/autostart
echo 'xset s noblank' >> ~/.config/lxsession/LXDE-pi/autostart
echo 'xset -dpms' >> ~/.config/lxsession/LXDE-pi/autostart
echo 'xset -s off' >> ~/.config/lxsession/LXDE-pi/autostart
```
Enable UART1 on Raspberry pi (add to the end of the file):
```
sudo nano /boot/config.txt
#Enable UART1
enable_uart=1
dtoverlay=disable-bt
dtoverlay=uart0
dtoverlay=uart1
dtoverlay=uart2
#Enable PAL on video
sdtv_mode = 2
```
Increase Swap to RAM size:
```
sudo dphys-swapfile swapoff
sudo nano /etc/dphys-swapfile
sudo dphys-swapfile setup
sudo dphys-swapfile swapon
```
Enable Composite Video:
```
sudo raspi-config
```
Display Options -> Composite

Enable SSH and Serial Port on Raspberry pi:
```
sudo raspi-config
```
-	Select option 3 - Interface Options
-	Select option P6 - Serial Port

(Attention):
At the prompt “Would you like a login shell to be accessible over serial?”, answer 'No'
At the prompt “Would you like the serial port hardware to be enabled?”, answer 'Yes'
Exit raspi-config and reboot the Raspberry Pi for changes to take effect

Get dronekit scripts from GitHub:
```
git clone https://github.com/dronekit/dronekit-python.git
```

After installation we can check mavlink connection with FC:
```
mavproxy.py --master=/dev/ttyAMA0 --baudrate 57600
```
# INAV
1. Install INAV Configurator 8
2. Download zip https://github.com/iNavFlight/inav/archive/refs/tags/8.0.1-RC1.zip
3. Unzip 8.0.1-RC1.zip and upload on Raspberry pi
4. Create a build directory and run the cmake and make commands from within the build directory
```
cd 8.0.1-RC1/ 
mkdir build
cd build
cmake ..
```
5. Buld the firmware. Find FC name "make help | less"
```
make SPEEDYBEEF405V3
```
6. Download inav_8.0.1_SPEEDYBEEF405V3.hex on your PC
7. Connect to flight conntroller and backup your settings (CLI -> diff all -> Save to File)
9. Flash inav_8.0.1_SPEEDYBEEF405V3.hex on FC
10. Connect to fight controller and read setting (CLI -> Load from back up file)
11. You should now have a “MSP RC OVERRIDE” mode available in “Modes” -> “Misc Modes”. Assign an RC channel from  your “real” controller to toggle this mode on and off.
12. Configure which channels we allow the computer to actually override
- 0000000000000000 = 0 -> don’t override any channels
- 1111111111111111 = 65535 -> override channels 1-16
- 0000000000001111 = 15 -> override channels 1-4
- 0000000011110000 = 240 -> override channels 5-8
- 1010101010101010 = 43690 -> override all even channels
```
set msp_override_channels=15
save
```
