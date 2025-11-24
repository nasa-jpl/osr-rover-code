#!/bin/bash
# Calibrate all servo motors (0-3) with two positions each

for m in {0..3}; do
    echo "Calibrating motor $m to position 200..."
    python3 calibrate_servos.py $m 200
    sleep 1
    
    echo "Calibrating motor $m to position 150..."
    python3 calibrate_servos.py $m 150
    sleep 1
done

echo "All servos calibrated!"
