#!/bin/bash
# ADAS System Setup Script for Termux
# Run this once to install all dependencies

echo "=================================="
echo "  ADAS System Setup for Termux"
echo "=================================="
echo ""

# Update package list
echo "[1/6] Updating package list..."
pkg update -y

# Install Python
echo "[2/6] Installing Python..."
pkg install python -y

# Install termux-api
echo "[3/6] Installing termux-api..."
pkg install termux-api -y

# Install numpy (for Kalman filter)
echo "[4/6] Installing NumPy..."
pip install numpy

# Grant necessary permissions
echo "[5/6] Requesting permissions..."
echo "      Please grant ALL permissions when prompted!"
termux-setup-storage
sleep 2

# Test sensor access
echo "[6/6] Testing sensor access..."
echo ""
echo "Testing GPS..."
termux-location -p gps -r last > /dev/null 2>&1
if [ $? -eq 0 ]; then
    echo "  ✓ GPS working"
else
    echo "  ✗ GPS failed - check Termux:API app is installed"
fi

echo ""
echo "Testing Accelerometer..."
termux-sensor -s 'BMI160 Accelerometer' -n 1 > /dev/null 2>&1
if [ $? -eq 0 ]; then
    echo "  ✓ Accelerometer working"
else
    echo "  ✗ Accelerometer failed"
fi

echo ""
echo "=================================="
echo "  Setup Complete!"
echo "=================================="
echo ""
echo "IMPORTANT: Make sure you have installed:"
echo "  - Termux:API app from Google Play or F-Droid"
echo "  - Grant all permissions to Termux:API"
echo ""
echo "To run the ADAS system:"
echo "  python adas_system.py"
echo ""
