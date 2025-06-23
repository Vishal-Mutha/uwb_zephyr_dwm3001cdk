/*! ----------------------------------------------------------------------------
 *  @file    ds_twr_initiator.c
 *  @brief   Double-sided two-way ranging (DS TWR) initiator example code with multiple responders
 *
 *           This is a simple code example which acts as the initiator in a DS TWR distance measurement exchange. This application sends a "poll"
 *           frame (recording the TX time-stamp of the poll), after which it waits for a "response" message from the "DS TWR responder" example
 *           code (companion to this application) to complete the exchange. Then it sends a final message with the required timestamps.
 *           The application alternates between two responders (IDs 2 and 3) and calculates the distance to each responder.
 *           The distances to both responders are displayed on the terminal.
 *
 * @attention
 *
 * Copyright 2015 - 2023 (c) Decawave Ltd, Dublin, Ireland.
 *
 * All rights reserved.
 *
 * @author Decawave
 * @author Modified for multiple responders
 */
#include "deca_device_api.h"

#include <sit/sit.h>
#include <sit/sit_device.h>
#include <sit/sit_distance.h>
#include <sit/sit_config.h>
#include <sit/sit_utils.h>
#include <sit_led/sit_led.h>

#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(main, LOG_LEVEL_INF);

/* Example application name */
#define APP_NAME "SIMPLE DS-TWR Initiator with Four Responders EXAMPLE\n"

uint8_t this_initiator_node_id  = 1;
uint8_t responder_node_ids[4]   = {2, 3, 4, 5};  /* IDs of the four responder devices */
uint8_t current_responder_idx   = 0;             /* Index to alternate between responders */

/* Store distances for each responder */
double distances[4] = {0.0, 0.0, 0.0, 0.0};

/* Inter-ranging delay period, in milliseconds. */
#define RNG_DELAY_MS 1000

/* Default antenna delay values for 64 MHz PRF. See NOTE 2 below. */
#define TX_ANT_DLY 16385
#define RX_ANT_DLY 16385

/* Indexes to access some of the fields in the frames defined above. */
#define ALL_MSG_SN_IDX          2
#define RESP_MSG_POLL_RX_TS_IDX 10
#define RESP_MSG_RESP_TX_TS_IDX 14
#define RESP_MSG_TS_LEN         4
/* Frame sequence number, incremented after each transmission. */

#define CPU_PROCESSING_TIME 400
#define POLL_TX_TO_RESP_RX_DLY_UUS_T (350 + CPU_PROCESSING_TIME)
#define RESP_RX_TO_FINAL_TX_DLY_UUS_T (350 + CPU_PROCESSING_TIME)
#define RESP_RX_TIMEOUT_UUS_T 1150
#define PRE_TIMEOUT 5


int main(void) {
	printk(APP_NAME);
	printk("==================\n");

    // INIT LED and let them Blink one Time to see Intitalion Finshed
    sit_led_init();

	int init_ok = sit_init();

    if(init_ok < 0){
        sit_set_led(2, 0);
    } else {
        sit_set_led(1, 0);
    }
    uint8_t frame_sequenz = 0;
	while (1) {
        // Select responder to communicate with
        uint8_t responder_node_id = responder_node_ids[current_responder_idx];

        sit_set_rx_tx_delay_and_rx_timeout(POLL_TX_TO_RESP_RX_DLY_UUS_T, RESP_RX_TIMEOUT_UUS_T);
        sit_set_preamble_detection_timeout(PRE_TIMEOUT);

        msg_simple_t twr_poll = {twr_1_poll, frame_sequenz, this_initiator_node_id, responder_node_id, 0};
        sit_start_poll((uint8_t*) &twr_poll, (uint16_t)sizeof(twr_poll));

        msg_simple_t rx_resp_msg;
        msg_id_t msg_id = ds_twr_2_resp;

        if(sit_check_msg_id(msg_id, &rx_resp_msg)) {
            uint64_t poll_tx_ts = get_tx_timestamp_u64();
			uint64_t resp_rx_ts = get_rx_timestamp_u64();

            uint32_t final_tx_time = (resp_rx_ts + (RESP_RX_TO_FINAL_TX_DLY_UUS_T * UUS_TO_DWT_TIME)) >> 8;
            uint64_t final_tx_ts = (((uint64_t)(final_tx_time & 0xFFFFFFFEUL)) << 8) + TX_ANT_DLY;

            msg_ds_twr_final_t final_msg = {
                ds_twr_3_final,
                rx_resp_msg.header.sequence,
                rx_resp_msg.header.dest,
                rx_resp_msg.header.source,
                (uint32_t)poll_tx_ts,
                (uint32_t)resp_rx_ts,
                (uint32_t)final_tx_ts,
                0
            };

            bool ret = sit_send_at((uint8_t*)&final_msg, sizeof(msg_ds_twr_final_t),final_tx_time);

            if (ret == false) {
                LOG_WRN("Something is wrong with Sending Final Msg");
                continue;
            }

            /* Calculate distance */
            uint32_t poll_tx_ts_32 = (uint32_t)poll_tx_ts;
            uint32_t resp_rx_ts_32 = (uint32_t)resp_rx_ts;
            uint32_t final_tx_ts_32 = (uint32_t)final_tx_ts;

            /* Get poll receive and response transmit timestamps from responder */
            uint32_t poll_rx_ts_32 = 0;  /* Will get from responder */
            uint32_t resp_tx_ts_32 = 0;  /* Will get from responder */

            /* Get responder index for storing distance */
            int resp_idx = 0;
            for (int i = 0; i < 4; i++) {
                if (responder_node_id == responder_node_ids[i]) {
                    resp_idx = i;
                    break;
                }
            }

            /* For demo purposes, calculate a placeholder distance */
            /* In real implementation, we would need to receive poll_rx_ts and resp_tx_ts from responder */
            double tof = ((double)(resp_rx_ts_32 - poll_tx_ts_32)) * DWT_TIME_UNITS / 2;
            distances[resp_idx] = tof * SPEED_OF_LIGHT;

            /* Display the distances from all four responders */
            printk("Distances: [ID %d: %.2f m] [ID %d: %.2f m] [ID %d: %.2f m] [ID %d: %.2f m]\n",
                   responder_node_ids[0], distances[0],
                   responder_node_ids[1], distances[1],
                   responder_node_ids[2], distances[2],
                   responder_node_ids[3], distances[3]);

            frame_sequenz++;

		} else {
			LOG_WRN("Something is wrong with Receiving Msg from responder %d", responder_node_id);
            dwt_writesysstatuslo(SYS_STATUS_ALL_RX_TO | SYS_STATUS_ALL_RX_ERR);
		}

        /* Switch to the next responder for the next iteration */
        current_responder_idx = (current_responder_idx + 1) % 4;

        /* Add a small delay between ranging cycles */
        k_msleep(50);
	}
    return 0;
}
