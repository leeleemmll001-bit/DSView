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

/* Where the serial to Ethernet server listens. Override with the
 * GD32LA_HOST / GD32LA_PORT environment variables before starting DSView. */
#define GD32LAN_DEFAULT_HOST        "192.168.1.100"
#define GD32LAN_DEFAULT_PORT        "5000"

/* The serial link runs at 460800 8N1 = 46080 B/s gross, so the stream
 * ceilings of la_proto.h apply (GD32LA_UART_MAX_*). */
#define GD32LAN_DEFAULT_SAMPLERATE  SR_KHZ(10)
#define GD32LAN_DEFAULT_LIMIT       SR_KHZ(10)
#define GD32LAN_DEFAULT_ASRATE      SR_KHZ(10)
#define GD32LAN_DEFAULT_ALIMIT      SR_KHZ(10)
#define GD32LAN_MIN_SAMPLERATE      SR_KHZ(2)

/* Reported hardware depth, bounds the sample limit DSView accepts. */
#define GD32LAN_MAX_SAMPLES         SR_MHZ(64)
#define GD32LAN_MAX_MIXED_SAMPLES   SR_MHZ(8)

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

    /* Serial server to connect to, from GD32LA_HOST / GD32LA_PORT. */
    char host[64];
    char port[8];

    uint64_t samplerate;
    uint64_t limit_samples;

    /* Samples per channel already forwarded to the session. */
    uint64_t sent_samples;

    guint8 channels_enabled;

    /* Work mode of the running capture (GD32LA_MODE_*), derived from the
     * enabled probes, not from sdi->mode. */
    guint8 stream_mode;
    guint8 logic_mask;      /* bit n: digital channel n is in the stream */
    guint8 analog_mask;     /* bit n: analog channel n is in the stream */

    gboolean is_loop;
    gboolean abort;
    gboolean limit_reached;
    gboolean ended;

    /* The TCP connection and the thread that pumps it into queue. */
    SOCKET sock;
    GThread *rx_thread;
    GMutex queue_lock;
    GByteArray *queue;
    volatile gboolean rx_done;   /* the link died or was shut down */

    /* Tail of a sample group that a recv chunk cut in half. */
    guint8 carry[GD32LA_MIXED_GROUP_BYTES];
    guint carry_len;

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
