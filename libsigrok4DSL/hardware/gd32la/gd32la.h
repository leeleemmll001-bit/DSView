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

/* Number and size of the asynchronous bulk transfers kept in flight. */
#define GD32LA_NUM_TRANSFERS            4
#define GD32LA_TRANSFER_SIZE            4096

/* Timeout used for the short control style reads (header, drain). */
#define GD32LA_SHORT_TIMEOUT_MS         1000

/* Defaults applied to a freshly scanned device. */
#define GD32LA_DEFAULT_SAMPLERATE       SR_MHZ(1)
#define GD32LA_DEFAULT_LIMIT_SAMPLES    SR_MHZ(1)

/* The firmware cannot represent a 1 kHz square wave below this rate. */
#define GD32LA_MIN_SAMPLERATE           SR_KHZ(2)

/* Reported as the hardware depth, used by DSView to bound the sample limit. */
#define GD32LA_MAX_SAMPLES              SR_MHZ(64)

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

    gboolean is_loop;
    gboolean abort;
    gboolean limit_reached;
    gboolean ended;

    struct libusb_transfer *transfers[GD32LA_NUM_TRANSFERS];
    guint8 *buffers[GD32LA_NUM_TRANSFERS];
    int outstanding;
};

SR_PRIV struct sr_dev_driver gd32la_driver_info;

#endif /* LIBDSL_HARDWARE_GD32LA_H */
