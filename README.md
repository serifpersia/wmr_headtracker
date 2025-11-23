# WMR Head Tracker

A simple 3DOF head tracker for Windows Mixed Reality sensors. Tracks rotation (yaw, pitch, roll) and sends data via UDP.

## Features

- 3DOF rotation tracking
- Works on Windows and Linux
- Configurable sensor orientation

## Requirements

- Python

## Usage

### Linux

```bash
./run_tracker.sh [--orientation <orientation>]
```

### Windows

```bash
sudo chmod +x run_tracker.sh
sudo ./run_tracker.bat [--orientation <orientation>]
```

*Default orientation is horizontal_right where the board its horizontally and the oculink connector is on the right hand side.

## License

This project is licensed under the [MIT License](LICENSE)
