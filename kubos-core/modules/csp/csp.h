/*
 * KubOS Core Flight Services
 * Copyright (C) 2016 Kubos Corporation
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

/**
 * @defgroup CSP
 * @addtogroup CSP
 * @{
 */

/**
 *
 * @file       csp.h
 * @brief KubOS CSP API
 *
 * Enabling this code requires certain configuration values to be present
 * in the application's configuration json. An example is given below:
 *
 *{
 *  "CSP": {
 *   "my_address":"1",
 *   "target_address":"1",
 *   "my_port":"10",
 *   "driver": {
 *     "usart": {
 *     },
 *      "interface":"if_kiss"
 *    },
 *  "usart_bus": "K_UART6",
 *  "max_msg_size":"32"
 *  }
 *}
 *
 * This would enable the CSP protocol and configure it for the usart K_UART6 bus
 * over the KISS interface in loopback mode.
 *
 */

#ifdef YOTTA_CFG_CSP

#include <csp/csp.h>

#define MY_ADDRESS YOTTA_CFG_CSP_MY_ADDRESS
#define TARGET_ADDRESS YOTTA_CFG_CSP_TARGET_ADDRESS
#define MY_PORT YOTTA_CFG_CSP_MY_PORT
#define CSP_DRIVER YOTTA_CFG_CSP_DRIVER_INTERFACE

/* different driver types */
typedef enum {
    if_kiss = 0,
    if_i2c,
    if_can,
} k_csp_driver;

/* csp status values */
typedef enum {
    K_CSP_OK = 0,
    K_CSP_ERROR,
} k_csp_status;

/**
 * choose driver to use and init csp routing
 * @param driver is the driver to initialize
 */
void k_init_csp(k_csp_driver driver);

/**
 * initialize kiss/uart interface
 */
void k_init_kiss_csp(void);

/**
 * Send a message string over CSP protocol
 * @param msg is the char pointer to message to send
 * @return k_csp_status, CSP_OK on success, CSP_ERROR on error
 */
k_csp_status k_csp_send(char * msg);

/**
 * Receive a CSP packet
 * @param packet is the pointer to the received CSP packet
 * @return k_csp_status, CSP_OK on success, CSP_ERROR on error
 */
k_csp_status k_csp_receive(char * msg);

#endif
