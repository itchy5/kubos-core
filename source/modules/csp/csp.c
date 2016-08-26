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

#ifdef YOTTA_CFG_CSP

#include "FreeRTOS.h"
#include "task.h"
#include "timers.h"
#include "queue.h"

#include <stdlib.h>

#include <csp/drivers/usart.h>
#include <csp/interfaces/csp_if_kiss.h>
#include <csp/arch/csp_thread.h>

#include "kubos-core/modules/csp/csp.h"
#include "kubos-hal/gpio.h"

/* static CSP interfaces */
static csp_iface_t csp_if_kiss;
static csp_kiss_handle_t csp_kiss_driver;

static xQueueHandle receive_queue;
static xQueueHandle send_queue;

#define BLINK_MS 100
static inline void blink(int pin)
{
    k_gpio_write(pin, 1);
    vTaskDelay(BLINK_MS / portTICK_RATE_MS);
    k_gpio_write(pin, 0);
}

CSP_DEFINE_TASK(task_csp_send)
{
    csp_packet_t * packet = NULL;
    csp_conn_t * conn;
    portBASE_TYPE status;
    char msg[YOTTA_CFG_CSP_MAX_MSG_SIZE]; /* static msg */

    /**
     * Try ping
     */

    csp_sleep_ms(200);

    int result = csp_ping(TARGET_ADDRESS, 100, 10, CSP_O_NONE);
    if (result) {
        #ifdef TARGET_LIKE_STM32
        blink(K_LED_ORANGE);
        #else
        blink(K_LED_GREEN);
        blink(K_LED_RED);
        #endif
    }

    while (1)
    {
        /* get msg from send queue */
        status = xQueueReceive(send_queue, msg, portMAX_DELAY);
        if (status != pdTRUE)
        {
            continue;
        }

        /* Get packet buffer for data */
        if (packet == NULL)
        {
            packet = csp_buffer_get(256);
            if (packet == NULL) {
                /* Could not get buffer element */
                return;
            }
        }

        /* Connect to host with regular UDP-like protocol and 100 ms timeout */
        conn = csp_connect(CSP_PRIO_NORM, TARGET_ADDRESS, MY_PORT, 100, CSP_O_NONE);
        if (conn == NULL)
        {
            /* Connect failed */
            /* Remember to free packet buffer */
            csp_buffer_free(packet);
            return;
        }

        /* Copy data to packet */
        strcpy((char *) packet->data, msg);

        /* Set packet length */
        packet->length = strlen(msg);

        /* Send packet */
        if (!csp_send(conn, packet, 100))
        {
            /* Send failed */
            csp_buffer_free(packet);
        }

        /* Close connection */
        csp_close(conn);
    }

    return CSP_TASK_RETURN;
}


CSP_DEFINE_TASK(task_csp_receive)
{
    /* Create socket without any socket options */
    csp_socket_t *sock = csp_socket(CSP_SO_NONE);

    /* Bind all ports to socket */
    csp_bind(sock, CSP_ANY);

    /* Create connections backlog queue */
    csp_listen(sock, YOTTA_CFG_CSP_MAX_MSG_WAITING);

    /* Pointer to current connection and packet */
    csp_conn_t * conn;
    csp_packet_t * packet;

    /* Process incoming connections */
    while (1)
    {
        /* Wait for connection, 100 ms timeout */
        if ((conn = csp_accept(sock, 100)) == NULL)
            continue;

        /* Read packets. Timout is 100 ms */
        while ((packet = csp_read(conn, 100)) != NULL)
        {
            switch (csp_conn_dport(conn))
            {
                case MY_PORT:
                    /* Process packet here */
                    /* store packet data */
                    xQueueSendToBack(receive_queue, (char *)packet->data, 0);
                    csp_buffer_free(packet);
                    break;

                default:
                    /* Let the service handler reply pings, buffer use, etc. */
                    #ifdef TARGET_LIKE_STM32
                    blink(K_LED_BLUE);
                    #else
                    blink(K_LED_RED);
                    blink(K_LED_GREEN);
                    #endif
                    csp_service_handler(conn, packet);
                    break;
            }
        }

        /* Close current connection, and handle next */
        csp_close(conn);
    }

    return CSP_TASK_RETURN;
}

k_csp_status k_csp_send(char * msg)
{
    portBASE_TYPE status;

    /* put user msg at end of queue */
    status = xQueueSendToBack(send_queue, msg, 0);
    if (status == pdTRUE)
    {
        blink(K_LED_RED);
        return K_CSP_OK;
    }
    else
    {
        return K_CSP_ERROR;
    }
}

k_csp_status k_csp_receive(char * msg)
{
    portBASE_TYPE status;

    /* get the msg from queue */
    status = xQueueReceive(receive_queue, msg, 0);
    if (status == pdTRUE)
    {
        blink(K_LED_GREEN);
        return K_CSP_OK;
    }
    else
    {
        return K_CSP_ERROR;
    }
}

void k_init_csp(k_csp_driver driver)
{
    switch(driver)
    {
        case if_kiss:
        {
            usart_init_default();
            k_init_kiss_csp();
            break;
        }
        case if_i2c:
        {
            /* I2C init function goes here */
            break;
        }
        case if_can:
        {
            /* CAN init function goes here */
            break;
        }
        default: /* NONE */
        {
            /* on-board */
            csp_buffer_init(5, 256);
            csp_init(MY_ADDRESS);
            csp_route_start_task(500, 1);
        }
    }

    /* create queue to store received data */
    receive_queue = xQueueCreate(YOTTA_CFG_CSP_MAX_MSG_WAITING, YOTTA_CFG_CSP_MAX_MSG_SIZE);
    /* create queue for messages to be sent */
    send_queue = xQueueCreate(YOTTA_CFG_CSP_MAX_MSG_WAITING, YOTTA_CFG_CSP_MAX_MSG_SIZE);

    /* create csp threads to handle send and receive */
    csp_thread_handle_t handle_csp_send, handle_csp_receive;
    csp_thread_create(task_csp_send, "CSP_SEND", configMINIMAL_STACK_SIZE*2, NULL, 2, &handle_csp_send);
    csp_thread_create(task_csp_receive, "CSP_RECIEVE", configMINIMAL_STACK_SIZE*2, NULL, 2, &handle_csp_receive);
}

void k_init_kiss_csp(void)
{
    /* init kiss interface */
    csp_kiss_init(&csp_if_kiss, &csp_kiss_driver, usart_putc, usart_insert, "KISS");

    /* Setup callback from USART RX to KISS RS */
    void my_usart_rx(uint8_t * buf, int len, void * pxTaskWoken)
    {
        csp_kiss_rx(&csp_if_kiss, buf, len, pxTaskWoken);
    }
    usart_set_callback(my_usart_rx);

    /* csp buffer and mtu in csp_iface must match */
    csp_buffer_init(5, 256);
    csp_init(MY_ADDRESS);
    /* set to route through KISS / UART */
    csp_route_set(TARGET_ADDRESS, &csp_if_kiss, CSP_NODE_MAC);
    csp_route_start_task(500, 1);
}

#endif
