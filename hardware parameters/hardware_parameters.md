# Hardware Parameters and Reproducibility Support

This document summarizes the hardware parameters required to reproduce the proposed 2D LiDAR-based 3D point cloud reconstruction framework. Parameters explicitly reported in the manuscript are listed directly. Parameters that must be confirmed from the final prototype or source-code configuration are marked as **TBD**.

## System-level configuration

| Parameter | Value / Setting | Role in reconstruction |
|---|---|---|
| Acquisition principle | Sequential 2D LiDAR scans stacked along the vertical axis | Generates 3D point clouds by assigning a vertical coordinate to each completed 360-degree scan layer. |
| Scanning geometry | 360-degree horizontal scan with controlled vertical translation | Preserves the native 2D LiDAR scan plane while extending acquisition into 3D. |
| Primary output | Raw and filtered 3D point cloud coordinates (X, Y, Z) | Used for visualization, filtering, and quantitative evaluation. |
| Operating scene assumption | Mostly static scene during acquisition | Required because the complete 3D point cloud is accumulated over multiple scan layers. |

## LiDAR sensor parameters

| Parameter | Value / Setting | Notes |
|---|---|---|
| Sensor model | SLAMTEC RPLIDAR A1 | Low-cost 2D spinning LiDAR. |
| Measurement principle | Laser triangulation | Produces distance and angle measurements. |
| Angular coverage | 0-360 degrees | Full planar scan used for each reconstructed layer. |
| Typical scan frequency | 1-10 Hz; representative value around 5.5 Hz | Used for synchronization. |
| Measurement rate | More than 8000 measurements/s | Reported sensor capability. |
| Approximate points per revolution | Around 1450 points/revolution | Depends on motor speed and acquisition mode. |
| Distance unit | Millimeters | Used in reconstruction and evaluation. |
| Angular unit | Degrees | Converted to radians in software. |
| Minimum measurable distance | Approximately 150 mm | Points below this range should be filtered. |
| Distance resolution | Below 0.5 mm for targets closer than 1.5 m | Manufacturer-reported behavior. |
| Relative ranging error | Below 1% over operating range | Reported sensor behavior. |
| Laser wavelength | Approximately 785 nm | Infrared laser. |
| Laser power | Typically around 3 mW; up to 5 mW | Low-power operation. |
| Laser safety class | Class 1 | Eye-safe classification. |
| Supply voltage | 5 V | Stable power supply required. |
| Current consumption | Around 80 mA standby; 350 mA normal operation | Startup current may be higher. |

## Vertical translation and motion parameters

| Parameter | Value / Setting | Notes |
|---|---|---|
| Vertical motion type | Controlled linear translation along Z-axis | Assigns scan layers to vertical positions. |
| Guiding mechanism | Linear sliding rails | Reduces lateral play and improves layer consistency. |
| Transmission mechanism | Trapezoidal lead screw with nut carriage | Converts motor rotation into vertical displacement. |
| Lead screw start type | Single-start screw | Pitch equals linear advance per full revolution. |
| Lead screw pitch | TBD mm/revolution | Fill using exact prototype screw specification. |
| Usable vertical stroke | TBD mm | Fill from measured prototype travel range. |
| Vertical step size | TBD mm/layer | Defines spacing between successive scan layers. |
| Number of vertical layers | TBD | Depends on stroke and selected step size. |
| Vertical speed | TBD mm/s | Required for synchronization-error analysis. |
| Backlash estimate | TBD mm | Measure or bound for error propagation analysis. |

## Motor, driver, and control parameters

| Parameter | Value / Setting | Notes |
|---|---|---|
| Motor type | NEMA 17 stepper motor | Repeatable positioning and holding capability. |
| Motor wiring | Bipolar 4-wire | Compatible with TB6600-type driver. |
| Nominal step angle | TBD degrees/step | Confirm exact motor specification. |
| Steps per revolution | TBD steps/revolution | Derived from step angle and microstepping. |
| Driver | TB6600 stepper motor driver | Adjustable current and microstep settings. |
| Drive mode | Full-step emphasized; microstepping setting TBD | Full-step prioritizes torque and holding stability. |
| Microcontroller | Arduino UNO | Generates step and direction signals. |
| Control signals | STEP and DIR | One pulse advances the motor by the configured increment. |
| Human interface | Push-button start/stop | Starts or interrupts acquisition sequences. |

## Coordinate reconstruction model

```text
x = r cos(theta)
y = r sin(theta)
z = vertical_position
P = (x, y, z)
```


