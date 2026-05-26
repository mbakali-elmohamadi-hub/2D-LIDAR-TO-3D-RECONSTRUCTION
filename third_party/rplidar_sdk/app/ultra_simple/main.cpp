#include <stdio.h>
#include <stdlib.h>
#include <signal.h>
#include <string.h>
#include <Windows.h>
#include <fstream>
#include <cmath>
#include <string>
#include <cstdlib> // for atoi

#include "sl_lidar.h"
#include "sl_lidar_driver.h"

#define M_PI 3.14159265358979323846f
#define _countof(_Array) (int)(sizeof(_Array) / sizeof(_Array[0]))

#ifdef _WIN32
#define delay(x)   ::Sleep(x)
#endif

using namespace sl;

bool checkSLAMTECLIDARHealth(ILidarDriver * drv) {
    sl_result op_result;
    sl_lidar_response_device_health_t healthinfo;

    op_result = drv->getHealth(healthinfo);
    if (SL_IS_OK(op_result)) {
        printf("SLAMTEC Lidar health status: %d\n", healthinfo.status);
        if (healthinfo.status == SL_LIDAR_STATUS_ERROR) {
            printf("Error: internal SLAMTEC lidar error detected. Please reboot the device and try again.\n");
            return false;
        } else {
            return true;
        }
    } else {
        printf("Error: failed to retrieve lidar health code: %x\n", op_result);
        return false;
    }
}

bool ctrl_c_pressed;
void ctrlc(int) {
    ctrl_c_pressed = true;
}

// Lire depuis Arduino (port série), z est un int maintenant
bool lireDepuisArduino(HANDLE hSerial, int& z, bool& stopRequested) {
    static std::string line = "";
    char c;
    DWORD bytesRead;
    stopRequested = false;

    while (ReadFile(hSerial, &c, 1, &bytesRead, NULL) && bytesRead > 0) {
        if (c == '\n' || c == '\r') {
            if (line.empty()) continue;

            if (line.find("off") != std::string::npos) {
                stopRequested = true;
                line.clear();
                return true;
            }

            z = atoi(line.c_str());  // conversion en int
            line.clear();
            return true;
        } else {
            line += c;
        }
    }

    return false;
}

int main(int argc, const char * argv[]) {
    const char * opt_channel_param_first = NULL;
    sl_u32 opt_channel_param_second = 0;
    sl_result op_result;
    IChannel* _channel;
    std::ofstream output_file("C:\\Users\\HP\\Documents\\projet\\resultats.txt");

    DCB dcbSerialParams = { 0 };
    COMMTIMEOUTS timeouts = { 0 };

    // Nouveau usage attendu :
    // program.exe --channel --serial <portLiDAR> [baudrate] <portArduino> [distanceCible]
    if (argc < 6 || strcmp(argv[1], "--channel") != 0 || strcmp(argv[2], "--serial") != 0) {
        printf("Usage: %s --channel --serial <portLiDAR> [baudrate] <portArduino> [distanceCible]\n", argv[0]);
        return -1;
    }

    opt_channel_param_first = argv[3];
    if (argc > 4 && argv[4][0] != 'C') // heuristique simple, si argv[4] n'est pas un COM (ex: COM7)
        opt_channel_param_second = strtoul(argv[4], NULL, 10);
    else
        opt_channel_param_second = 115200;

    // Si baudrate est spécifié, alors portArduino sera argv[5], sinon argv[4]
    const char* portArduino = NULL;
    int distanceCible = 0;

    if (argc == 6) {
        // argv[4] = baudrate ou portArduino
        if (opt_channel_param_second == 115200) {
            portArduino = argv[4];
        } else {
            portArduino = argv[5];
        }
    } else if (argc >= 7) {
        portArduino = argv[5];
        distanceCible = atoi(argv[6]);
    }

    if (portArduino == NULL) {
        printf("Erreur: port Arduino non spécifié.\n");
        return -1;
    }

    printf("Vérification des ports : LiDAR = %s, Arduino = %s, Baudrate = %u, Distance = %d\n", 
           opt_channel_param_first, portArduino, opt_channel_param_second, distanceCible);

    ILidarDriver * drv = *createLidarDriver();
    if (!drv) {
        printf("Insufficient memory.\n");
        exit(-2);
    }

    sl_lidar_response_device_info_t devinfo;
    bool connectSuccess = false;

    _channel = (*createSerialPortChannel(opt_channel_param_first, opt_channel_param_second));
    if (SL_IS_OK((drv)->connect(_channel))) {
        op_result = drv->getDeviceInfo(devinfo);
        connectSuccess = SL_IS_OK(op_result);
    } else {
        delete drv;
        drv = NULL;
    }

    if (!connectSuccess) {
        printf("Error: cannot bind to serial port %s.\n", opt_channel_param_first);
        goto on_finished;
    }

    if (!checkSLAMTECLIDARHealth(drv)) {
        goto on_finished;
    }

    char arduinoPortName[20];
    snprintf(arduinoPortName, sizeof(arduinoPortName), "\\\\.\\%s", portArduino);

    HANDLE hSerial = CreateFile(
        arduinoPortName,
        GENERIC_READ | GENERIC_WRITE,
        0,
        NULL,
        OPEN_EXISTING,
        0,
        NULL
    );

    if (hSerial == INVALID_HANDLE_VALUE) {
        DWORD err = GetLastError();
        printf("Error: cannot open Arduino port %s. Windows error code: %lu\n", portArduino, err);
        goto on_finished;
    }

    dcbSerialParams.DCBlength = sizeof(dcbSerialParams);
    if (!GetCommState(hSerial, &dcbSerialParams)) {
        printf("Error getting serial port parameters.\n");
        goto on_finished;
    }

    dcbSerialParams.BaudRate = CBR_9600;
    dcbSerialParams.ByteSize = 8;
    dcbSerialParams.StopBits = ONESTOPBIT;
    dcbSerialParams.Parity   = NOPARITY;

    if (!SetCommState(hSerial, &dcbSerialParams)) {
        printf("Error setting serial port parameters.\n");
        goto on_finished;
    }

    timeouts.ReadIntervalTimeout         = 50;
    timeouts.ReadTotalTimeoutConstant   = 50;
    timeouts.ReadTotalTimeoutMultiplier = 10;

    if (!SetCommTimeouts(hSerial, &timeouts)) {
        printf("Error setting serial port timeouts.\n");
        goto on_finished;
    }

    // Envoi de la distance cible à l'Arduino, si > 0 (int)
    if (distanceCible > 0) {
        char buffer[32];
        sprintf(buffer, "%d\n", distanceCible);
        DWORD bytesWritten;
        if (!WriteFile(hSerial, buffer, strlen(buffer), &bytesWritten, NULL)) {
            printf("Error sending distance to Arduino.\n");
            goto on_finished;
        } else {
            printf("Distance cible envoyée à l'Arduino : %s", buffer);
        }
    }

    drv->setMotorSpeed();
    drv->startScan(0, 1);

    signal(SIGINT, ctrlc);
    int z = -1;
    bool stop = false;

    printf("Waiting for the first Z value from Arduino...\n");
    while (z < 0) {
        if (lireDepuisArduino(hSerial, z, stop)) {
            if (stop) {
                printf("Stop signal received before start. Exiting.\n");
                return -1;
            }
            if (z >= 0) {
                printf("Initial Z value received: %d\n", z);
                break;
            }
        }
    }

    while (!ctrl_c_pressed && !stop) {
        int zReceived = z;

        if (lireDepuisArduino(hSerial, zReceived, stop)) {
            if (stop) {
                printf("Stop signal received. Stopping.\n");
                break;
            }
            z = zReceived;
        }

        sl_lidar_response_measurement_node_hq_t nodes[8192];
        size_t count = _countof(nodes);

        if (SL_IS_OK(drv->grabScanDataHq(nodes, count))) {
            drv->ascendScanData(nodes, count);

            for (int pos = 0; pos < (int)count; ++pos) {
                int quality = nodes[pos].quality >> SL_LIDAR_RESP_MEASUREMENT_QUALITY_SHIFT;
                if (quality == 0) continue;

                float angle_deg = (nodes[pos].angle_z_q14 * 90.f) / 16384.f;
                float distance_mm = nodes[pos].dist_mm_q2 / 4.0f;

                float angle_rad = angle_deg * M_PI / 180.0f;
                float x = distance_mm * cos(angle_rad);
                float y = distance_mm * sin(angle_rad);

                printf("%s angle: %03.2f, dist: %08.2f, quality: %3d x: %.2f, y: %.2f, z: %d\n",
                    (nodes[pos].flag & SL_LIDAR_RESP_HQ_FLAG_SYNCBIT) ? "Begin Turn " : "  ",
                    angle_deg, distance_mm, quality, x, y, z);

                if (output_file.is_open()) {
                    output_file << "x:" << x << ",y:" << y << ",z:" << z << std::endl;
                }
            }
        }
    }

    drv->stop();
    delay(200);
    drv->setMotorSpeed(0);

    if (output_file.is_open()) {
        output_file.close();
    }

on_finished:
    if (drv) {
        delete drv;
        drv = NULL;
    }

    return 0;
}
