#!/usr/bin/env bash
set -e

echo "--------------------------------------"
echo "      WMR Headtracker Launcher        "
echo "--------------------------------------"

# Ensure python3 is available
if ! command -v python3 &> /dev/null; then
    echo "Python 3 is not installed."
    echo "Install it with:"
    echo "  Arch: sudo pacman -S python"
    echo "  Ubuntu/Debian: sudo apt install python3"
    exit 1
fi

# Detect the WMR / HoloLens USB device
DEVICE=$(ls /dev/bus/usb/*/* 2>/dev/null | while read d; do
    if udevadm info -a -n "$d" | grep -q 'idVendor.*045e' && \
       udevadm info -a -n "$d" | grep -q 'idProduct.*0659'; then
        echo "$d"
        break
    fi
done)

if [ -z "$DEVICE" ]; then
    echo "WMR / HoloLens Sensors not detected."
    echo "Please plug in the headset USB and try again."
    exit 1
fi

echo "Detected device: $DEVICE"

# Create udev rule if missing
RULE_FILE="/etc/udev/rules.d/99-hololens.rules"
if [ ! -f "$RULE_FILE" ]; then
    echo "Creating udev rule for WMR device (requires sudo)..."

    # Detect best group automatically
    if getent group wheel >/dev/null; then
        GROUP="wheel"
    elif getent group sudo >/dev/null; then
        GROUP="sudo"
    elif getent group users >/dev/null; then
        GROUP="users"
    else
        GROUP=$(id -gn)
    fi

    sudo bash -c "echo 'SUBSYSTEM==\"usb\", ATTR{idVendor}==\"045e\", ATTR{idProduct}==\"0659\", MODE=\"0660\", GROUP=\"$GROUP\", TAG+=\"uaccess\"' > $RULE_FILE"
    sudo udevadm control --reload-rules
    sudo udevadm trigger

    echo "Udev rule installed for group: $GROUP"
    echo "Please unplug and replug the headset USB, then press ENTER"
    read
fi

# Set up Python virtual environment
if [ ! -d "venv" ]; then
    echo "Creating Python virtual environment..."
    python3 -m venv venv
fi

source venv/bin/activate

echo "Installing/updating requirements..."
pip install --upgrade pip
pip install -r requirements.txt

echo ""
echo "Starting WMR Tracker..."
echo "--------------------------------------"
python wmr_tracker.py
