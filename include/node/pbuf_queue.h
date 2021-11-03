//  Packet Buffer Queue. Based on mqueue (Mbuf Queue) from Mynewt OS:
//  https://github.com/apache/mynewt-core/blob/master/kernel/os/include/os/os_mbuf.h
/*
 * Licensed to the Apache Software Foundation (ASF) under one
 * or more contributor license agreements.  See the NOTICE file
 * distributed with this work for additional information
 * regarding copyright ownership.  The ASF licenses this file
 * to you under the Apache License, Version 2.0 (the
 * "License"); you may not use this file except in compliance
 * with the License.  You may obtain a copy of the License at
 *
 *  http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing,
 * software distributed under the License is distributed on an
 * "AS IS" BASIS, WITHOUT WARRANTIES OR CONDITIONS OF ANY
 * KIND, either express or implied.  See the License for the
 * specific language governing permissions and limitations
 * under the License.
 */

#ifndef H_PBUF_QUEUE_
#define H_PBUF_QUEUE_

#include "lwip/pbuf.h"       //  For Lightweight IP Stack pbuf 
#include "nimble_npl.h"      //  For NimBLE Porting Layer (multitasking functions)
#include "node/bsd_queue.h"  //  For Queue Functions

/**
 * Structure representing a list of pbufs inside a pbuf_queue.
 * pbuf_list is stored in the header of the pbuf, before the LoRaWAN Header.
 */
struct pbuf_list {
    /**
     * Header length
     */
    u16_t header_len;
    /**
     * Payload length
     */
    u16_t payload_len;
    /**
     * Pointer to pbuf
     */
    struct pbuf *pb;
    /**
     * Pointer to header in pbuf
     */
    struct pbuf *header;
    /**
     * Pointer to payload in pbuf
     */
    struct pbuf *payload;
    /**
     * Pointer to next node in the pbuf_list
     */
    STAILQ_ENTRY(pbuf_list) next;
};

/**
 * Structure representing a queue of pbufs.
 */
struct pbuf_queue {
    /** List of pbufs in the queue */
    STAILQ_HEAD(, pbuf_list) mq_head;

    /** Event to post when new buffers are available on the queue. */
    struct ble_npl_event mq_ev;

    /** Header length of packet (LoRaWAN Header only, excluding pbuf_list header) */
    uint16_t header_len;
};

/// Allocate a pbuf for LoRaWAN transmission. This returns a pbuf with 
/// pbuf_list Header, LoRaWAN Header and LoRaWAN Payload.
struct pbuf *
alloc_pbuf(
    uint16_t header_len,   //  Header length of packet (LoRaWAN Header only, excluding pbuf_list header)
    uint16_t payload_len); //  Payload length of packet, excluding header

/// Copy a buffer into a pbuf's payload. We don't support partial copying into the payload.
/// Return 0 if successful.
int pbuf_copyinto(
    struct pbuf *pb,  //  pbuf Packet Buffer
    uint16_t offset,  //  Offset into payload (must be 0)
    const void *buf,  //  Buffer to be copied into payload
    int buf_size);    //  Size of buffer (must be same as pbuf payload size)

/*
 * Copy data from an mbuf chain starting "off" bytes from the beginning,
 * continuing for "len" bytes, into the indicated buffer.
 * We don't support partial copying into the payload.
 *
 * @param m The mbuf chain to copy from
 * @param off The offset into the mbuf chain to begin copying from
 * @param len The length of the data to copy
 * @param dst The destination buffer to copy into
 *
 * @return                      0 on success;
 *                              -1 if the mbuf does not contain enough data.
 */
int pbuf_copydata(const struct pbuf *m, int off, int len, void *dst);

/// Return the pbuf Packet Buffer header
void *get_pbuf_header(
    struct pbuf *buf,     //  pbuf Packet Buffer
    size_t header_size);  //  Size of header

/**
 * Initializes a pbuf_queue.  A pbuf_queue is a queue of pbufs that ties to a
 * particular task's event queue.  pbuf_queues form a helper API around a common
 * paradigm: wait on an event queue until at least one packet is available,
 * then process a queue of packets.
 *
 * When pbufs are available on the queue, an event OS_EVENT_T_MQUEUE_DATA
 * will be posted to the task's pbuf queue.
 *
 * @param mq                    The pbuf_queue to initialize
 * @param ev_cb                 The callback to associate with the pbuf_queue
 *                              event.  Typically, this callback pulls each
 *                              packet off the pbuf_queue and processes them.
 * @param arg                   The argument to associate with the pbuf_queue event.
 * @param header_len            Header length of packet (LoRaWAN Header only, excluding pbuf_list header)
 *
 * @return                      0 on success, non-zero on failure.
 */
int
pbuf_queue_init(struct pbuf_queue *mq, ble_npl_event_fn *ev_cb, void *arg, uint16_t header_len);

/**
 * Remove and return a single pbuf from the pbuf queue.  Does not block.
 *
 * @param mq The pbuf queue to pull an element off of.
 *
 * @return The next pbuf in the queue, or NULL if queue has no pbufs.
 */
struct pbuf *
pbuf_queue_get(struct pbuf_queue *mq);

/**
 * Adds a packet (i.e. packet header pbuf) to a pbuf_queue. The event associated
 * with the pbuf_queue gets posted to the specified eventq.
 *
 * @param mq                    The pbuf queue to append the pbuf to.
 * @param evq                   The event queue to post an event to.
 * @param m                     The pbuf to append to the pbuf queue.
 *
 * @return 0 on success, non-zero on failure.
 */
int
pbuf_queue_put(struct pbuf_queue *mq, struct ble_npl_eventq *evq, struct pbuf *m);

/* swap octets */
void swap_buf(uint8_t *dst, const uint8_t *src, int len);

#endif
