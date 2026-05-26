# Dependencies and Requirements

This document lists the software and hardware dependencies required to reproduce the 2D LiDAR to 3D point cloud reconstruction pipeline.

## 1. Hardware Requirements

| Component | Purpose | Notes |
|---|---|---|
| 2D RPLiDAR / SLAMTEC LiDAR sensor | Provides 2D polar scans: angle and distance | Connected to the computer through a serial/USB interface |
| Arduino-compatible board | Controls the vertical translation mechanism | Streams the current Z-axis position to the C++ acquisition program |
| Stepper motor + driver | Moves the LiDAR along the Z axis | Used for vertical actuation |
| Lead-screw or linear guide mechanism | Converts motor rotation into vertical displacement | Produces stacked 2D scans for 3D reconstruction |
| Windows PC | Runs the acquisition, filtering, and visualization pipeline | The current C++ acquisition code uses Windows serial APIs |

## 2. Operating System

The acquisition program is currently written for **Windows**, because it uses the Windows serial communication API:

```cpp
#include <Windows.h>
```

Recommended environment:

- Windows 10 or Windows 11
- USB drivers installed for both the RPLiDAR sensor and Arduino board
- Correct COM ports identified in Device Manager

Linux/macOS support would require replacing the Windows serial interface with a cross-platform serial library or POSIX serial communication.

## 3. Python Dependencies

Python is used for point-cloud outlier filtering.

Recommended version:

```text
Python 3.9 or newer
```

Install dependencies with:

```bash
pip install -r requirements.txt
```

Required Python packages:

| Package | Version | Purpose |
|---|---:|---|
| numpy | >= 1.23 | Numerical array processing |
| scikit-learn | >= 1.2 | Local Outlier Factor filtering |

Optional packages for future evaluation and visualization:

| Package | Purpose |
|---|---|
| pandas | Reading tabular measurement/evaluation files |
| matplotlib | Plotting evaluation curves and error distributions |
| open3d | Point-cloud visualization and advanced geometric processing |

## 4. Java Dependencies

The graphical interface and point-cloud viewer are implemented in Java.

Recommended version:

```text
JDK 8 or newer
```

Required external Java library:

```text
src/java/viewer/lib/jSerialComm-2.11.2.jar
```

Purpose:

- COM-port detection and serial-port handling from the Java GUI.

Compile example:

```bat
cd src\java\viewer
javac -cp lib\jSerialComm-2.11.2.jar Surface.java
```

Run example:

```bat
java -cp .;lib\jSerialComm-2.11.2.jar Surface
```

On Linux/macOS, replace `;` in the classpath with `:`.

## 5. C++ Dependencies

The acquisition and reconstruction module is implemented in C++.

Required tools:

| Tool | Purpose |
|---|---|
| Visual Studio 2019 or newer | Compiling the Windows C++ acquisition program |
| C++17-compatible compiler | Building the acquisition source code |
| SLAMTEC/RPLiDAR SDK | Communicating with the RPLiDAR sensor |
| Windows SDK | Serial communication through `Windows.h` |

The RPLiDAR SDK is included in:

```text
third_party/rplidar_sdk/
```

The main acquisition and reconstruction source file is:

```text
src/cpp/lidar_acquisition/main.cpp
```

The program converts each 2D LiDAR scan sample into a 3D point using:

```text
x = r cos(theta)
y = r sin(theta)
z = current vertical displacement
```

The expected output format is:

```text
x:<value>,y:<value>,z:<value>
```

## 6. Arduino Dependencies

The firmware is provided in:

```text
firmware/arduino_z_axis_controller/arduino_z_axis_controller.ino
```

Required software:

| Tool | Purpose |
|---|---|
| Arduino IDE 1.8.x or 2.x | Uploading firmware to the Arduino board |
| USB serial driver | Communication between Arduino and PC |

The firmware controls the Z-axis actuation and transmits the current vertical position to the host computer.

## 7. Runtime Data Flow

The full pipeline uses the following dependency chain:

```text
RPLiDAR sensor + Arduino controller
        ↓
C++ acquisition and reconstruction module
        ↓
Raw 3D point file: x,y,z
        ↓
Python LOF outlier filtering
        ↓
Filtered 3D point cloud
        ↓
Java visualization interface
```

## 8. Minimal Installation Checklist

Before running the system, verify that:

1. Python is installed and `pip install -r requirements.txt` completes successfully.
2. Java JDK is installed and `javac` is available from the command line.
3. The Arduino firmware is uploaded to the Arduino board.
4. The RPLiDAR SDK is available in `third_party/rplidar_sdk/`.
5. The C++ acquisition executable is built and placed where the Java GUI expects it.
6. The correct LiDAR and Arduino COM ports are selected.
7. The output folders `data/raw/` and `data/processed/` exist.

## 9. Notes 

These dependencies are intentionally lightweight. The reconstruction pipeline relies mainly on standard C++ acquisition, Arduino-based actuation, Python-based filtering, and Java-based visualization. This structure allows the experimental system to remain reproducible without requiring expensive commercial 3D LiDAR software or proprietary processing frameworks.
