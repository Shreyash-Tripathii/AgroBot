#!/usr/bin/env bash
# ==============================================================================
# setup_rpi_i2c.sh
# Configures Software I2C-3 @ 100 kHz on Raspberry Pi running Ubuntu 24.04 Server
# for the BNO085 IMU (bypasses Raspberry Pi hardware I2C clock-stretching bug).
# ==============================================================================

set -e

CONFIG_TXT="/boot/firmware/config.txt"
if [ ! -f "$CONFIG_TXT" ] && [ -f "/boot/config.txt" ]; then
    CONFIG_TXT="/boot/config.txt"
fi

echo "==> Configuring Raspberry Pi Software I2C-3 on $CONFIG_TXT..."

# 1. Add i2c-gpio device tree overlay for bus 3 (SDA=GPIO 23, SCL=GPIO 24, 100 kHz)
OVERLAY_LINE="dtoverlay=i2c-gpio,bus=3,i2c_gpio_sda=23,i2c_gpio_scl=24,i2c_gpio_delay_us=5"

if grep -Fxq "$OVERLAY_LINE" "$CONFIG_TXT"; then
    echo "    [OK] Overlay already present in $CONFIG_TXT."
else
    echo "    [+] Adding $OVERLAY_LINE to $CONFIG_TXT"
    echo "" | sudo tee -a "$CONFIG_TXT" > /dev/null
    echo "# Software I2C-3 for BNO085 (100 kHz bitbang, avoids clock-stretching bugs)" | sudo tee -a "$CONFIG_TXT" > /dev/null
    echo "$OVERLAY_LINE" | sudo tee -a "$CONFIG_TXT" > /dev/null
fi

# 2. Ensure kernel modules load on boot
echo "==> Enabling i2c kernel modules in /etc/modules..."
for mod in i2c-dev i2c-gpio; do
    if ! grep -q "^$mod" /etc/modules; then
        echo "    [+] Adding $mod to /etc/modules"
        echo "$mod" | sudo tee -a /etc/modules > /dev/null
    fi
done

# 3. Install i2c-tools and required utilities
echo "==> Installing i2c-tools..."
sudo apt-get update -qq
sudo apt-get install -y -qq i2c-tools python3-pip gpiod

# 4. Add user to required groups (i2c, dialout, gpio)
echo "==> Adding user '$USER' to groups i2c, dialout, gpio..."
sudo groupadd -f i2c
sudo groupadd -f gpio
sudo usermod -aG i2c,dialout,gpio "$USER"

# 5. Setup udev rule for /dev/i2c-* access
UDEV_RULE="/etc/udev/rules.d/99-i2c.rules"
if [ ! -f "$UDEV_RULE" ]; then
    echo 'KERNEL=="i2c-[0-9]*", GROUP="i2c", MODE="0660"' | sudo tee "$UDEV_RULE" > /dev/null
    sudo udevadm control --reload-rules && sudo udevadm trigger
fi

# 6. Install Adafruit BNO085 and ExtendedI2C Python packages
echo "==> Installing Adafruit Python packages for BNO085..."
pip3 install --break-system-packages \
    Adafruit-Blinka \
    adafruit-circuitpython-bno08x \
    adafruit-extended-bus

echo ""
echo "=========================================================================="
echo " Setup complete! Please REBOOT your Raspberry Pi for changes to take effect:"
echo "   sudo reboot"
echo ""
echo " After rebooting, verify that BNO085 is detected on bus 3 (address 0x4A):"
echo "   i2cdetect -y 3"
echo "=========================================================================="
