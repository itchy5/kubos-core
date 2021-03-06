/*
 * KubOS Core Flight Services
 * Copyright (C) 2015 Kubos Corporation
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */
#ifndef GPS_H
#define GPS_H

#include "msg.h"
#include "periph/uart.h"
#include "time.h"
#include "thread.h"

#ifdef __cplusplus
extern "C" {
#endif

#ifndef GPS_MSG_Q_SIZE
#define GPS_MSG_Q_SIZE 4
#endif

#ifndef GPS_BUFSIZE
#define GPS_BUFSIZE    128
#endif

#ifndef GPS_DATALOG_SIZE
#define GPS_DATALOG_SIZE (1024 * 512)
#endif

typedef struct gps_cfg {
    kernel_pid_t pid;
    uint16_t type;
    uart_t uart;
    uint32_t baudrate;
} gps_cfg_t;

typedef struct {
    /**
     * Hour: 0-23
     * Minute: 0-59
     * Seconds: 0-59
     * Year: 0-99
     * Month: 1-12
     * Day: 1-31
     */
    uint8_t hour, minute, seconds, year, month, day;

    /** Millis: 0-999 */
    uint16_t milliseconds;
    /** Latitude in degrees (-90, 90) */
    float latitude;
    /** Longitude in degrees (-180, 180) */
    float longitude;
    /** Altitude in meters */
    float altitude;
    /** Speed over ground, meters/sec */
    float speed;
    /** Vertical speed, meters/sec */
    float climb;
} gps_fix_t;

void gps_connect(gps_cfg_t* config);
gps_fix_t *gps_last_fix(void);

#ifdef __cplusplus
}
#endif

#endif // GPS_H
