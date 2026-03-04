#!/bin/bash

set +e

CURRENT_HOSTNAME=`cat /etc/hostname | tr -d " \t\n\r"`
if [ -f /usr/lib/raspberrypi-sys-mods/imager_custom ]; then
   /usr/lib/raspberrypi-sys-mods/imager_custom set_hostname bach-rpi5
else
   echo bach-rpi5 >/etc/hostname
   sed -i "s/127.0.1.1.*$CURRENT_HOSTNAME/127.0.1.1\tbach-rpi5/g" /etc/hosts
fi
FIRSTUSER=`getent passwd 1000 | cut -d: -f1`
FIRSTUSERHOME=`getent passwd 1000 | cut -d: -f6`
if [ -f /usr/lib/raspberrypi-sys-mods/imager_custom ]; then
   /usr/lib/raspberrypi-sys-mods/imager_custom enable_ssh
else
   systemctl enable ssh
fi
if [ -f /usr/lib/userconf-pi/userconf ]; then
   /usr/lib/userconf-pi/userconf 'bachrpi5' '$5$AuaA0Rf8p7$s.6NyBgOIN0YetcfoUhSDLWG2CyCJmZ0GhZr7RGmucD'
else
   echo "$FIRSTUSER:"'$5$AuaA0Rf8p7$s.6NyBgOIN0YetcfoUhSDLWG2CyCJmZ0GhZr7RGmucD' | chpasswd -e
   if [ "$FIRSTUSER" != "bachrpi5" ]; then
      usermod -l "bachrpi5" "$FIRSTUSER"
      usermod -m -d "/home/bachrpi5" "bachrpi5"
      groupmod -n "bachrpi5" "$FIRSTUSER"
      if grep -q "^autologin-user=" /etc/lightdm/lightdm.conf ; then
         sed /etc/lightdm/lightdm.conf -i -e "s/^autologin-user=.*/autologin-user=bachrpi5/"
      fi
      if [ -f /etc/systemd/system/getty@tty1.service.d/autologin.conf ]; then
         sed /etc/systemd/system/getty@tty1.service.d/autologin.conf -i -e "s/$FIRSTUSER/bachrpi5/"
      fi
      if [ -f /etc/sudoers.d/010_pi-nopasswd ]; then
         sed -i "s/^$FIRSTUSER /bachrpi5 /" /etc/sudoers.d/010_pi-nopasswd
      fi
   fi
fi
if [ -f /usr/lib/raspberrypi-sys-mods/imager_custom ]; then
   /usr/lib/raspberrypi-sys-mods/imager_custom set_wlan  -h 'Duy Bach' 'a48f67aaddb850528a5a7940f7e20a8cd0bfcac77d7c6a88a86168f047d0de1b' 'VN'
else
cat >/etc/wpa_supplicant/wpa_supplicant.conf <<'WPAEOF'
country=VN
ctrl_interface=DIR=/var/run/wpa_supplicant GROUP=netdev
ap_scan=1

update_config=1
network={
	scan_ssid=1
	ssid="Duy Bach"
	psk=a48f67aaddb850528a5a7940f7e20a8cd0bfcac77d7c6a88a86168f047d0de1b
}

WPAEOF
   chmod 600 /etc/wpa_supplicant/wpa_supplicant.conf
   rfkill unblock wifi
   for filename in /var/lib/systemd/rfkill/*:wlan ; do
       echo 0 > $filename
   done
fi
rm -f /boot/firstrun.sh
sed -i 's| systemd.run.*||g' /boot/cmdline.txt
exit 0
