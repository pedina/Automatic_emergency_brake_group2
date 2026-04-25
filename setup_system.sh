#!/bin/bash
# Kilépés hiba esetén
set -e

echo "=== AEB Rendszer (Main Ág) Frissítés és Indítás ==="

# 1. Navigálás és frissítés
cd ~/ros2_ws/src/Automatic_emergency_brake_group2
echo "--> GitHub frissítések lekérése a main ágra..."
git checkout main
git pull origin main

# 2. Workspace újraépítése
echo "--> Fordítás indítása..."
cd ~/ros2_ws
colcon build --symlink-install

# 3. Környezet aktiválása
echo "--> Környezet forrásolása..."
source install/setup.bash

echo "=== SIKER! Indul a Launch! ==="
# 4. Automatikus indítás!
ros2 launch plan_long_emergency system_launch.py
