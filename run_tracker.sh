#!/bin/bash

ORIENTATIONS=("horizontal_right" "horizontal_left" "vertical_right" "vertical_left")
ORIENTATION="horizontal_right"

show_help() {
    echo "Usage: ./run_tracker.sh [--orientation <orientation>]"
    echo "Available orientations: ${ORIENTATIONS[*]}"
}

while [[ $# -gt 0 ]]; do
    case $1 in
        -h|--help)
            show_help
            exit 0
            ;;
        --orientation)
            shift
            ORIENTATION="$1"
            shift
            ;;
        *)
            echo "Unknown argument: $1"
            show_help
            exit 1
            ;;
    esac
done

if ! command -v python3 &> /dev/null; then
    echo "Python3 is not installed. Please install Python3 and try again."
    exit 1
fi

if [ ! -d "venv" ]; then
    echo "Creating virtual environment..."
    python3 -m venv venv
fi

echo "Activating venv..."
source venv/bin/activate

echo "Installing requirements..."
pip install -r requirements.txt

echo "Running tracker with orientation: $ORIENTATION"
python3 wmr_tracker.py --orientation "$ORIENTATION"
