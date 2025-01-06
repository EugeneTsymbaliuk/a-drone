Download Raspberry Pi Imager and inslall OS on Raspberry Pi
(Attention):
Install legacy Raspbian Bullseye 64-bit
https://www.raspberrypi.com/software/

# Connect to Raspberry Pi, open Terminal and type all commands

# Update Raspbian
sudo apt update
sudo apt -y upgrade
sudo apt install -y python3-dev python3-opencv

# Install apps
#sudo rm /usr/lib/python3.11/EXTERNALLY-MANAGED
pip3 install pyserial
pip3 install dronekit
pip3 install MAVProxy

# To work dronekit in python from 3.10 you need to
#nano +2689 ~/.local/lib/python3.11/site-packages/dronekit/__init__.py
#change collections.MutableMapping on collections.abc.MuttableMapping

# Create starter file
touch ~/starter.sh
echo '#!/bin/bash' >> ~/starter.sh
echo  >> ~/starter.sh
echo 'python ~/dronekit-python/tracker_ardu.py' >> ~/starter.sh
chmod 755 ~/starter.sh

# Change window manager (mutter to openbox-lxde)
sudo sed -i "s/mutter/openbox-lxde/g" /etc/xdg/lxsession/LXDE-pi/desktop.conf
cp -rf /etc/xdg/openbox/ ~/.config/

# Autostart
mkdir -p ~/.config/lxsession/LXDE-pi/
cp /etc/xdg/lxsession/LXDE-pi/* ~/.config/lxsession/LXDE-pi/
echo "/home/`whoami`/starter.sh" >> ~/.config/lxsession/LXDE-pi/autostart

# Screensaver
sed -i /xscreensaver/d ~/.config/lxsession/LXDE-pi/autostart
echo 'xset s noblank' >> ~/.config/lxsession/LXDE-pi/autostart
echo 'xset -dpms' >> ~/.config/lxsession/LXDE-pi/autostart
echo 'xset -s off' >> ~/.config/lxsession/LXDE-pi/autostart

# Enable UART1 on Raspberry pi (add to the end of the file)
sudo nano /boot/config.txt

# Enable UART1
enable_uart=1
dtoverlay=disable-bt
dtoverlay=uart0
dtoverlay=uart1
dtoverlay=uart2

# Enable PAL on video
sdtv_mode = 2

# Increase Swap to RAM size
sudo dphys-swapfile swapoff
sudo nano /etc/dphys-swapfile
sudo dphys-swapfile setup
sudo dphys-swapfile swapon

#Enable Composite Video
sudo raspi-config
Display Options -> Composite

# Enable SSH and Serial Port on Raspberry pi
sudo raspi-config
-	Select option 3 - Interface Options
-	Select option P6 - Serial Port

(Attention):
At the prompt “Would you like a login shell to be accessible over serial?”, answer 'No'
At the prompt “Would you like the serial port hardware to be enabled?”, answer 'Yes'
Exit raspi-config and reboot the Raspberry Pi for changes to take effect

# Get dronekit scripts from GitHub  
git clone https://github.com/dronekit/dronekit-python.git

# After installation we can check mavlink connection with FC
mavproxy.py --master=/dev/ttyAMA0 --baudrate 57600

