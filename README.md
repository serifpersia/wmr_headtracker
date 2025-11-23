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
sudo chmod +x run_tracker.sh
sudo ./run_tracker.sh [--orientation <orientation>]
```

### Windows

```bash
./run_tracker.bat [--orientation <orientation>]
*Selecting correct 0 indexed sensor might fail on fresh system boot, selecting 1 then restarting the program & selecting 0 to get IMU data streaming.
```

*Default orientation is horizontal_right where the board its horizontally and the oculink connector is on the right hand side.

## License

This project is licensed under the [MIT License](LICENSE)
