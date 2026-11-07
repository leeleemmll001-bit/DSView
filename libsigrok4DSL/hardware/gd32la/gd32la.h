/*
 * This file is part of the libsigrok project.
 *
 * GD32 logic analyzer simulator driver - private declarations.
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 */

#ifndef LIBDSL_HARDWARE_GD32LA_H
#define LIBDSL_HARDWARE_GD32LA_H

#include <glib.h>
#include "../../libsigrok-internal.h"
#include "la_proto.h"

/* The device exposes exactly one vendor specific interface. */
#define GD32LA_IFACE                    0

/* Number and size of the asynchronous bulk transfers kept in flight.
 *
 * The size is a multiple of every group size the device can send, so a
 * transfer never ends in the middle of a sample group:
 *
 *   logic  4032 / 16 = 252 groups of 64 samples
 *   analog 4032 /  2 = 2016 samples
 *   mixed  4032 / 18 = 224 groups of 8 samples
 */
#define GD32LA_NUM_TRANSFERS            4
#define GD32LA_TRANSFER_SIZE            4032

/* Timeout used for the short control style reads (header, drain). */
#define GD32LA_SHORT_TIMEOUT_MS         1000

/* Defaults applied to a freshly scanned device. Logical captures are mixed
 * signal captures now, so the default rate is the one of the analog side. */
#define GD32LA_DEFAULT_SAMPLERATE       SR_KHZ(100)
#define GD32LA_DEFAULT_LIMIT_SAMPLES    SR_KHZ(100)

/* Analog mode defaults: 100 kSa/s for one second. Two 8 bit channels at that
 * rate need 200 kB/s, well inside what a full speed bulk pipe can carry. */
#define GD32LA_DEFAULT_ASRATE           SR_KHZ(100)
#define GD32LA_DEFAULT_ALIMIT           SR_KHZ(100)
#define GD32LA_MAX_ASRATE               SR_KHZ(200)

/* Mixed captures need 18 bytes per 8 samples, that is 2.25 bytes per sample
 * and channel set, so the same ceiling applies as for the analog mode. */
#define GD32LA_MAX_MIXED_RATE           SR_KHZ(200)

/* The firmware cannot represent a 1 kHz square wave below this rate. */
#define GD32LA_MIN_SAMPLERATE           SR_KHZ(2)

/* Reported as the hardware depth, used by DSView to bound the sample limit.
 * DSView allocates the analog capture buffer as limit * channels bytes, so a
 * mixed capture is capped much lower than a pure logic one. */
#define GD32LA_MAX_SAMPLES              SR_MHZ(64)
#define GD32LA_MAX_MIXED_SAMPLES        SR_MHZ(8)

/* Polling interval of the session source that drives libusb completions. */
#define GD32LA_POLL_INTERVAL_MS         5

struct gd32la_context {
    /* Back pointer, also the data feed destination. */
    struct sr_dev_inst *sdi;

    uint64_t samplerate;
    uint64_t limit_samples;

    /* Samples per channel already forwarded to the session. */
    uint64_t sent_samples;

    guint8 channels_enabled;

    /* Work mode of the running capture (GD32LA_MODE_*). It is derived from
     * the enabled probes, not from sdi->mode: a logic capture that also has
     * analog probes enabled runs in mixed mode. */
    guint8 stream_mode;
    guint8 logic_mask;      /* bit n: digital channel n is in the stream */
    guint8 analog_mask;     /* bit n: analog channel n is in the stream */

    gboolean is_loop;
    gboolean abort;
    gboolean limit_reached;
    gboolean ended;

    struct libusb_transfer *transfers[GD32LA_NUM_TRANSFERS];
    /* Every buffer holds three areas: the raw sample stream read by libusb,
     * then the de-interleaved logic and analog payloads handed to DSView. */
    guint8 *buffers[GD32LA_NUM_TRANSFERS];
    int outstanding;
};

SR_PRIV struct sr_dev_driver gd32la_driver_info;

#endif /* LIBDSL_HARDWARE_GD32LA_H */
