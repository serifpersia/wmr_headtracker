#!/bin/bash

set -e

if [ ! -d "venv" ]; then
    echo "Creating virtual environment..."
    python3 -m venv venv
fi

echo "Activating venv..."
source venv/bin/activate

echo "Installing requirements..."
pip install -r requirements.txt

echo "Running tracker..."
python3 wmr_tracker.py
