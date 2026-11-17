/*
 * This file is part of the libsigrok project.
 *
 * GD32 logic analyzer simulator - network driver private declarations.
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 */

#ifndef LIBDSL_HARDWARE_GD32LAN_H
#define LIBDSL_HARDWARE_GD32LAN_H

#include <glib.h>
#include "../../libsigrok-internal.h"
#include "la_proto.h"

#ifdef _WIN32
#include <winsock2.h>
#else
#include <sys/socket.h>
#define SOCKET              int
#define INVALID_SOCKET      (-1)
#endif

/* Where the two serial to Ethernet servers listen. Override with the
 * GD32LA_HOST / GD32LA_PORT / GD32LA_PORT2 environment variables before
 * starting DSView. The device sends the even frame numbers over the first
 * link (USART2 on PB10/PB11) and the odd ones over the second (USART1 on
 * PA2/PA3), both 1500000 8N1. */
#define GD32LAN_DEFAULT_HOST        "192.168.0.115"
#define GD32LAN_DEFAULT_PORT        "2404"
#define GD32LAN_DEFAULT_PORT2       "2405"

/* The two serial links run at 2 x 1500000 8N1 = 300 kB/s gross, so the
 * analog stream (rate * 32 B/s) tops out at GD32LA_UART_MAX_ANALOG_RATE. */
#define GD32LAN_DEFAULT_ASRATE      SR_HZ(6400)
#define GD32LAN_DEFAULT_ALIMIT      SR_KHZ(64)
#define GD32LAN_MIN_SAMPLERATE      SR_KHZ(2)

/* Reported hardware depth, bounds the sample limit DSView accepts. */
#define GD32LAN_MAX_SAMPLES         SR_MHZ(64)

/* Polling interval of the session source that drains the receive queue. */
#define GD32LAN_POLL_INTERVAL_MS    5

/* While a capture runs the driver sends a PING every 300 ms, so the device
 * knows the host is still there. The device auto-stops its stream after
 * LA_WATCHDOG_MS (1.5 s) of complete silence.
 *
 * The period is measured on the monotonic clock and not by counting session
 * source invocations: a source with fd -1 only fires as fast as the timer
 * granularity allows (Windows defaults to 15.6 ms, whatever interval was
 * requested), so a tick counter stretched the keepalive to ~1.5 s and the
 * device watchdog fired before the first PING ever went out. Three missed
 * periods still leave room inside the 1.5 s watchdog. */
#define GD32LAN_KEEPALIVE_US        (300 * 1000)

/* Grace period between the STOP command and the TCP disconnect, so the
 * serial server has time to push the command out on the wire. */
#define GD32LAN_STOP_LINGER_MS      100

/* Connect / handshake timeouts. */
#define GD32LAN_CONNECT_TIMEOUT_MS  3000
#define GD32LAN_RECV_STEP_MS        250
#define GD32LAN_HDR_TIMEOUT_MS      4000

struct gd32lan_context {
    /* Back pointer, also the data feed destination. */
    struct sr_dev_inst *sdi;

    /* Serial servers to connect to, from GD32LA_HOST / GD32LA_PORT /
     * GD32LA_PORT2. */
    char host[64];
    char port[8];
    char port2[8];

    uint64_t samplerate;
    uint64_t limit_samples;

    /* Samples per channel already forwarded to the session. */
    uint64_t sent_samples;

    guint8 channels_enabled;

    /* Work mode of the running capture, always GD32LA_MODE_ANALOG. */
    guint8 stream_mode;

    gboolean is_loop;
    gboolean abort;
    gboolean limit_reached;
    gboolean ended;

    /* The two TCP connections and the threads that pump them into the
     * queues. Link 0 carries the even frame numbers, link 1 the odd ones. */
    SOCKET sock;
    GThread *rx_thread;
    GMutex queue_lock;
    GByteArray *queue;
    volatile gboolean rx_done;      /* link 0 died or was shut down */

    SOCKET sock2;
    GThread *rx_thread2;
    GByteArray *queue2;
    volatile gboolean rx_done2;     /* link 1 died or was shut down */

    /* Staging for the payload of the frame that is being forwarded. */
    guint8 frame_buf[GD32LA_ANALOG_FRAME_BYTES - GD32LA_FRAME_HEADER_BYTES];

    /* Sequence number bookkeeping of the frame merge, see
     * lan_forward_frames(). */
    uint32_t last_seq;
    gboolean seq_valid;
    gboolean link_warned;   /* one link died mid capture, warned once */
    uint64_t frames_lost;

    /* Monotonic time of the last keepalive PING, see GD32LAN_KEEPALIVE_US. */
    gint64 last_ping_us;
};

/* Private driver context: the one network device this driver exposes. */
struct gd32lan_drv_context {
    struct sr_context *sr_ctx;
    struct sr_dev_inst *sdi;
};

SR_PRIV struct sr_dev_driver gd32lan_driver_info;

#endif /* LIBDSL_HARDWARE_GD32LAN_H */
