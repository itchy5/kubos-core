/*
 * KubOS HAL
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

#include "FreeRTOS.h"
#include "task.h"
#include "timers.h"
#include "queue.h"


#include <csp/drivers/usart.h>
#include <stdlib.h>
#include <csp/interfaces/csp_if_kiss.h>
#include <csp/arch/csp_thread.h>

#include "kubos-core/modules/csp/csp.h"

/* static CSP interfaces */
static csp_iface_t csp_if_kiss;
static csp_kiss_handle_t csp_kiss_driver;

static xQueueHandle packet_queue;
static xQueueHandle msg_queue;

CSP_DEFINE_TASK(task_csp_send)
{
    csp_packet_t * packet;
    csp_conn_t * conn;
    portBASE_TYPE status;
    char * msg; /* msg pointer */

    /**
     * Try data packet to server
     */
    while (1) {
        status = xQueueReceive(msg_queue, &msg, portMAX_DELAY);
        if (status != pdTRUE) {
            continue;
        }

        /* Get packet buffer for data */
        packet = csp_buffer_get(100);
        if (packet == NULL) {
            /* Could not get buffer element */
            return;
        }

        /* Connect to host HOST, port PORT with regular UDP-like protocol and 100 ms timeout */
        conn = csp_connect(CSP_PRIO_NORM, TARGET_ADDRESS, MY_PORT, 100, CSP_O_NONE);
        if (conn == NULL) {
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
        if (!csp_send(conn, packet, 100)) {
            /* Send failed */
            csp_buffer_free(packet);
        }

        /* success */
        //blink(K_LED_RED);
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

    /* Create 10 connections backlog queue */
    csp_listen(sock, 10);

    /* Pointer to current connection and packet */
    csp_conn_t * conn;
    csp_packet_t * packet;

    /* Process incoming connections */
    while (1) {

        /* Wait for connection, 100 ms timeout */
        if ((conn = csp_accept(sock, 100)) == NULL)
            continue;

        /* Read packets. Timout is 100 ms */
        while ((packet = csp_read(conn, 100)) != NULL) {
            /* store packet */
            xQueueSendToBack(packet_queue, packet, 0);
            /* success */
            //blink(K_LED_GREEN);
        }

        /* Close current connection, and handle next */
        csp_close(conn);

    }

    return CSP_TASK_RETURN;
}

k_csp_status k_csp_send(char * msg)
{
    portBASE_TYPE status;

    status = xQueueSendToBack(msg_queue, msg, 0);
    if (status == pdTRUE)
    {
        K_CSP_OK;
    }
    else
    {
        return K_CSP_ERROR;
    }
}

k_csp_status k_csp_receive(csp_packet_t * packet)
{
    portBASE_TYPE status;
    status = xQueueReceive(packet_queue, packet, portMAX_DELAY);
    if (status == pdTRUE)
    {
        K_CSP_OK;
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
            csp_buffer_init(5, 100);
            csp_init(MY_ADDRESS);
            csp_route_start_task(500, 1);
        }
    }

    /* create queue to store received packets */
    packet_queue = xQueueCreate(10, sizeof(csp_packet_t));
    /* create queue for messages to be sent */
    packet_queue = xQueueCreate(10, YOTTA_CFG_CSP_MAX_MSG_SIZE);

    /* create csp threads to handle send and receive */
    csp_thread_handle_t handle_csp_send, handle_csp_receive;
    csp_thread_create(task_csp_send, "CSP_SEND", configMINIMAL_STACK_SIZE*2, NULL, 2, &handle_csp_send);
    csp_thread_create(task_csp_receive, "CSP_RECIEVE", configMINIMAL_STACK_SIZE, NULL, 2, &handle_csp_receive);
}

void k_init_kiss_csp(void)
{
    /* init kiss interface */
    csp_kiss_init(&csp_if_kiss, &csp_kiss_driver, usart_putc, usart_insert, "KISS");

    /* Setup callback from USART RX to KISS RS */
    void my_usart_rx(uint8_t * buf, int len, void * pxTaskWoken) {
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
