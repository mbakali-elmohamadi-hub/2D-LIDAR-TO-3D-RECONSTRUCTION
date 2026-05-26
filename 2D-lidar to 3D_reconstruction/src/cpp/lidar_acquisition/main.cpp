/*
 * RPLiDAR 2D Scan Acquisition and 3D Point Cloud Reconstruction
 *
 * This program connects to a SLAMTEC/RPLiDAR sensor and an Arduino-based
 * Z-axis controller. The LiDAR provides polar 2D measurements (angle, distance),
 * while the Arduino streams the current Z displacement. Each 2D point is then
 * converted into a 3D Cartesian point:
 *
 *     x = distance_mm * cos(angle_rad)
 *     y = distance_mm * sin(angle_rad)
 *     z = current_z_mm
 *
 * Output format:
 *     x:<value>,y:<value>,z:<value>
 *
 * Example on Windows:
 *     ultra_simple.exe --channel --serial COM7 115200 COM5 200 data/raw/raw_points.txt
 */

#include <stdio.h>
#include <stdlib.h>
#include <signal.h>
#include <string.h>
#include <Windows.h>
#include <fstream>
#include <cmath>
#include <string>
#include <cstdlib>

#include "sl_lidar.h"
#include "sl_lidar_driver.h"

#define M_PI 3.14159265358979323846f
#define _countof(_Array) (int)(sizeof(_Array) / sizeof(_Array[0]))

#ifdef _WIN32
#define delay(x) ::Sleep(x)
#endif

using namespace sl;

static bool ctrl_c_pressed = false;

void ctrlc(int) {
    ctrl_c_pressed = true;
}

bool checkLidarHealth(ILidarDriver* driver) {
    sl_lidar_response_device_health_t health_info;
    sl_result result = driver->getHealth(health_info);

    if (SL_IS_OK(result)) {
        printf("SLAMTEC LiDAR health status: %d\n", health_info.status);
        if (health_info.status == SL_LIDAR_STATUS_ERROR) {
            printf("Error: internal LiDAR error detected. Please reboot the device and try again.\n");
            return false;
        }
        return true;
    }

    printf("Error: failed to retrieve LiDAR health code: %x\n", result);
    return false;
}

/*
 * Read a complete serial line from the Arduino.
 * Numeric lines are interpreted as Z positions in millimetres.
 * The line "off" marks the end of the scan.
 */
bool readArduinoLine(HANDLE serialHandle, int& zPositionMm, bool& stopRequested) {
    static std::string line;
    char character;
    DWORD bytesRead;
    stopRequested = false;

    while (ReadFile(serialHandle, &character, 1, &bytesRead, NULL) && bytesRead > 0) {
        if (character == '\n' || character == '\r') {
            if (line.empty()) {
                continue;
            }

            if (line.find("off") != std::string::npos) {
                stopRequested = true;
                line.clear();
                return true;
            }

            zPositionMm = atoi(line.c_str());
            line.clear();
            return true;
        }
        line += character;
    }
    return false;
}

int main(int argc, const char* argv[]) {
    if (argc < 6 || strcmp(argv[1], "--channel") != 0 || strcmp(argv[2], "--serial") != 0) {
        printf("Usage: %s --channel --serial <lidar_port> [baudrate] <arduino_port> [scan_height_mm] [output_file]\n", argv[0]);
        return -1;
    }

    const char* lidarPort = argv[3];
    sl_u32 lidarBaudrate = 115200;
    const char* arduinoPort = NULL;
    int scanHeightMm = 0;
    const char* outputPath = "data/raw/raw_points.txt";

    if (argc >= 5 && argv[4][0] != 'C') {
        lidarBaudrate = strtoul(argv[4], NULL, 10);
    }

    if (argc == 6) {
        arduinoPort = (lidarBaudrate == 115200) ? argv[4] : argv[5];
    } else if (argc >= 7) {
        arduinoPort = argv[5];
        scanHeightMm = atoi(argv[6]);
    }

    if (argc >= 8) {
        outputPath = argv[7];
    }

    if (arduinoPort == NULL) {
        printf("Error: Arduino port was not specified.\n");
        return -1;
    }

    printf("Configuration: LiDAR=%s, Arduino=%s, Baudrate=%u, Height=%d mm, Output=%s\n",
           lidarPort, arduinoPort, lidarBaudrate, scanHeightMm, outputPath);

    std::ofstream outputFile(outputPath);
    if (!outputFile.is_open()) {
        printf("Error: cannot open output file: %s\n", outputPath);
        return -1;
    }

    ILidarDriver* driver = *createLidarDriver();
    if (!driver) {
        printf("Error: insufficient memory.\n");
        return -2;
    }

    IChannel* channel = (*createSerialPortChannel(lidarPort, lidarBaudrate));
    sl_lidar_response_device_info_t deviceInfo;
    bool connected = false;

    if (SL_IS_OK(driver->connect(channel))) {
        connected = SL_IS_OK(driver->getDeviceInfo(deviceInfo));
    }

    if (!connected) {
        printf("Error: cannot bind to LiDAR serial port %s.\n", lidarPort);
        delete driver;
        return -1;
    }

    if (!checkLidarHealth(driver)) {
        delete driver;
        return -1;
    }

    char arduinoPortName[32];
    snprintf(arduinoPortName, sizeof(arduinoPortName), "\\\\.\\%s", arduinoPort);

    HANDLE arduinoSerial = CreateFile(
        arduinoPortName,
        GENERIC_READ | GENERIC_WRITE,
        0,
        NULL,
        OPEN_EXISTING,
        0,
        NULL
    );

    if (arduinoSerial == INVALID_HANDLE_VALUE) {
        DWORD errorCode = GetLastError();
        printf("Error: cannot open Arduino port %s. Windows error code: %lu\n", arduinoPort, errorCode);
        delete driver;
        return -1;
    }

    DCB serialParameters = { 0 };
    serialParameters.DCBlength = sizeof(serialParameters);
    if (!GetCommState(arduinoSerial, &serialParameters)) {
        printf("Error: cannot read Arduino serial parameters.\n");
        CloseHandle(arduinoSerial);
        delete driver;
        return -1;
    }

    serialParameters.BaudRate = CBR_9600;
    serialParameters.ByteSize = 8;
    serialParameters.StopBits = ONESTOPBIT;
    serialParameters.Parity = NOPARITY;

    if (!SetCommState(arduinoSerial, &serialParameters)) {
        printf("Error: cannot set Arduino serial parameters.\n");
        CloseHandle(arduinoSerial);
        delete driver;
        return -1;
    }

    COMMTIMEOUTS timeouts = { 0 };
    timeouts.ReadIntervalTimeout = 50;
    timeouts.ReadTotalTimeoutConstant = 50;
    timeouts.ReadTotalTimeoutMultiplier = 10;
    SetCommTimeouts(arduinoSerial, &timeouts);

    if (scanHeightMm > 0) {
        char buffer[32];
        sprintf(buffer, "%d\n", scanHeightMm);
        DWORD bytesWritten;
        if (!WriteFile(arduinoSerial, buffer, strlen(buffer), &bytesWritten, NULL)) {
            printf("Error: could not send scan height to Arduino.\n");
            CloseHandle(arduinoSerial);
            delete driver;
            return -1;
        }
        printf("Scan height sent to Arduino: %s", buffer);
    }

    driver->setMotorSpeed();
    driver->startScan(0, 1);
    signal(SIGINT, ctrlc);

    int currentZMm = -1;
    bool stopRequested = false;

    printf("Waiting for the first Z value from Arduino...\n");
    while (currentZMm < 0) {
        if (readArduinoLine(arduinoSerial, currentZMm, stopRequested)) {
            if (stopRequested) {
                printf("Stop signal received before scanning started.\n");
                CloseHandle(arduinoSerial);
                delete driver;
                return -1;
            }
            if (currentZMm >= 0) {
                printf("Initial Z value received: %d mm\n", currentZMm);
            }
        }
    }

    while (!ctrl_c_pressed && !stopRequested) {
        int receivedZ = currentZMm;
        if (readArduinoLine(arduinoSerial, receivedZ, stopRequested)) {
            if (stopRequested) {
                printf("Stop signal received. Ending scan.\n");
                break;
            }
            currentZMm = receivedZ;
        }

        sl_lidar_response_measurement_node_hq_t nodes[8192];
        size_t count = _countof(nodes);

        if (SL_IS_OK(driver->grabScanDataHq(nodes, count))) {
            driver->ascendScanData(nodes, count);

            for (int i = 0; i < (int)count; ++i) {
                int quality = nodes[i].quality >> SL_LIDAR_RESP_MEASUREMENT_QUALITY_SHIFT;
                if (quality == 0) {
                    continue;
                }

                float angleDeg = (nodes[i].angle_z_q14 * 90.f) / 16384.f;
                float distanceMm = nodes[i].dist_mm_q2 / 4.0f;
                float angleRad = angleDeg * M_PI / 180.0f;

                float x = distanceMm * cos(angleRad);
                float y = distanceMm * sin(angleRad);
                int z = currentZMm;

                printf("angle: %03.2f, dist: %08.2f, quality: %3d, x: %.2f, y: %.2f, z: %d\n",
                       angleDeg, distanceMm, quality, x, y, z);

                outputFile << "x:" << x << ",y:" << y << ",z:" << z << std::endl;
            }
        }
    }

    driver->stop();
    delay(200);
    driver->setMotorSpeed(0);

    outputFile.close();
    CloseHandle(arduinoSerial);
    delete driver;
    return 0;
}
