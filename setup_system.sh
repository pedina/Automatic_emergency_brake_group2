#!/bin/bash
# Kilépés hiba esetén
set -e

echo "--- AEB Rendszer Integráció Indítása ---"

# 1. Mappa ellenőrzése
REPO_DIR="$HOME/ros2_ws/src/Automatic_emergency_brake_group2"
if [ ! -d "$REPO_DIR" ]; then
    echo "Hiba: A projekt mappa nem található itt: $REPO_DIR"
    exit 1
fi
cd "$REPO_DIR"

# 2. Git műveletek - Biztonsági mentés és frissítés
echo "--> GitHub frissítések lekérése..."
git fetch --all

# Létrehozunk egy friss 'integration' ágat a main-ből
git checkout main
git pull origin main
git checkout -B integration_all_nodes

# 3. Branchek összefűzése
echo "--> Node ágak összefűzése..."
git merge origin/behavior-planner --no-edit || echo "Figyelem: behavior-planner merge hiba!"
git merge origin/plan-long-emergency --no-edit || echo "Figyelem: plan-long-emergency merge hiba!"
git merge origin/control-long-emergency --no-edit || echo "Figyelem: control-long-emergency merge hiba!"

# 4. Buildelés (kifejezetten a te 3 node-odra)
echo "--> Fordítás indítása..."
cd ~/ros2_ws
colcon build --symlink-install --packages-select behavior_planner plan_long_emergency ctrl_long_emergency

echo "--- SIKER! Most add ki: source install/setup.bash ---"
