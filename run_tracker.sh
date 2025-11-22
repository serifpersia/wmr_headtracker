#!/bin/bash

if ! command -v python &> /dev/null
then
    echo "Python is not installed. Please install Python and try again."
    exit 1
fi

if [ ! -d "venv" ]; then
    echo "Creating virtual environment..."
    python -m venv venv
fi

source venv/bin/activate

echo "Installing requirements..."
pip install -r requirements.txt

echo "Running tracker..."
python wmr_tracker.py
