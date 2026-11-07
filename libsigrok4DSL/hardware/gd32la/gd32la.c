/*
 * This file is part of the libsigrok project.
 *
 * GD32 logic analyzer simulator driver.
 *
 * The device is a GD32F470VE running the firmware in MyLogic/firmware. It
 * enumerates as a vendor specific USB full speed device with two bulk
 * endpoints and streams one of two sample formats:
 *
 *   LOGIC   cross interleaved logic samples (LA_CROSS_DATA), as the
 *           DreamSourceLab logic analyzers send them
 *   ANALOG  interleaved 8 bit samples of two analog channels, as the
 *           DreamSourceLab DAQ devices send them
 *
 * Because the whole sample stream is produced on the MCU, this driver is
 * small: scan, open, configure, start/stop acquisition, and forward the bulk
 * data to the session bus.
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 */

#include <string.h>
#include <assert.h>

#include "gd32la.h"
#include "../../log.h"

#undef LOG_PREFIX
#define LOG_PREFIX "gd32la: "

static struct sr_dev_driver *di = &gd32la_driver_info;

static int dev_destroy(struct sr_dev_inst *sdi);
static void la_rebuild_channels(struct sr_dev_inst *sdi);

static const char *logic_probe_names[] = {
    "D0", "D1",
    NULL,
};

static const char *analog_probe_names[] = {
    "A0", "A1",
    NULL,
};

/* Full speed bulk tops out at about 1 MB/s, so the analog mode is limited to
 * 200 kSa/s with two channels (400 kB/s). */
static const uint64_t logic_samplerates[] = {
    SR_KHZ(10),
    SR_KHZ(20),
    SR_KHZ(50),
    SR_KHZ(100),
    SR_KHZ(200),
    SR_KHZ(500),
    SR_MHZ(1),
};

static const uint64_t analog_samplerates[] = {
    SR_KHZ(10),
    SR_KHZ(20),
    SR_KHZ(50),
    SR_KHZ(100),
    SR_KHZ(200),
};

/* Probe controls offered in the device options dialog, same set the DAQ demo
 * device exposes. */
static const int32_t probe_options[] = {
    SR_CONF_PROBE_COUPLING,
    SR_CONF_PROBE_VDIV,
    SR_CONF_PROBE_MAP_DEFAULT,
    SR_CONF_PROBE_MAP_UNIT,
    SR_CONF_PROBE_MAP_MIN,
    SR_CONF_PROBE_MAP_MAX,
};

static const uint64_t vdivs[] = {
    SR_mV(10),
    SR_mV(20),
    SR_mV(50),
    SR_mV(100),
    SR_mV(200),
    SR_mV(500),
    SR_V(1),
    SR_V(2),
};

static const uint8_t couplings[] = {
    SR_DC_COUPLING,
    SR_AC_COUPLING,
};

static const char *map_units[] = {
    "V", "A", "\xc2\xb0""C", "\xc2\xb0""F", "g", "m", "m/s",
};

static const int32_t sessions[] = {
    SR_CONF_SAMPLERATE,
    SR_CONF_LIMIT_SAMPLES,
};

/* -------------------------------------------------------------------------
 * Low level helpers
 * ---------------------------------------------------------------------- */

/* Write one 8 byte command record to the OUT endpoint. */
static int la_send_cmd(struct sr_usb_dev_inst *usb, uint8_t cmd, uint8_t arg,
                       uint32_t value)
{
    gd32la_cmd_t record;
    int transferred = 0;
    int ret;

    if (usb == NULL || usb->devhdl == NULL)
        return SR_ERR;

    record.cmd = cmd;
    record.arg = arg;
    record.reserved = 0;
    record.value = value;

    ret = libusb_bulk_transfer(usb->devhdl, GD32LA_EP_OUT,
                               (unsigned char *)&record, sizeof(record),
                               &transferred, GD32LA_SHORT_TIMEOUT_MS);
    if (ret != 0) {
        sr_err("Failed to send command 0x%02x: %s.", cmd, libusb_error_name(ret));
        return SR_ERR;
    }

    return SR_OK;
}

/* Read and discard whatever is still queued, so a new START always begins at
 * a packet boundary. */
static void la_drain(struct sr_usb_dev_inst *usb)
{
    uint8_t buf[GD32LA_PACKET_SIZE];
    int transferred = 0;
    int ret;
    int guard = 256;

    if (usb == NULL || usb->devhdl == NULL)
        return;

    while (guard-- > 0) {
        ret = libusb_bulk_transfer(usb->devhdl, GD32LA_EP_IN, buf, sizeof(buf),
                                   &transferred, 20);
        if (ret == LIBUSB_ERROR_TIMEOUT)
            break;
        if (ret != 0) {
            sr_dbg("Drain stopped: %s.", libusb_error_name(ret));
            break;
        }
    }
}

/* Wait for the 16 byte header that answers GD32LA_CMD_START. Returns SR_OK
 * when a valid header has been consumed. */
static int la_read_header(struct sr_usb_dev_inst *usb)
{
    uint8_t buf[64];
    gd32la_hdr_t *hdr = (gd32la_hdr_t *)buf;
    int transferred = 0;
    int attempt;
    int ret;

    for (attempt = 0; attempt < 8; attempt++) {
        memset(buf, 0, sizeof(buf));

        ret = libusb_bulk_transfer(usb->devhdl, GD32LA_EP_IN, buf, sizeof(buf),
                                   &transferred, GD32LA_SHORT_TIMEOUT_MS);
        if (ret == LIBUSB_ERROR_TIMEOUT) {
            sr_err("Timed out waiting for the sample stream header.");
            continue;
        }
        if (ret != 0) {
            sr_err("Bulk read failed: %s.", libusb_error_name(ret));
            return SR_ERR;
        }

        if (transferred >= GD32LA_HDR_LEN &&
            hdr->magic[0] == GD32LA_HDR_MAGIC0 &&
            hdr->magic[1] == GD32LA_HDR_MAGIC1 &&
            hdr->magic[2] == GD32LA_HDR_MAGIC2 &&
            hdr->magic[3] == GD32LA_HDR_MAGIC3) {

            if (hdr->version != GD32LA_HDR_VERSION) {
                sr_err("Unsupported device protocol version %u.",
                       (unsigned int)hdr->version);
                return SR_ERR;
            }
            if (hdr->channels != GD32LA_CHANNELS) {
                sr_err("Device reports %u channels, expected %u.",
                       (unsigned int)hdr->channels, GD32LA_CHANNELS);
                return SR_ERR;
            }
            if (hdr->mode == GD32LA_MODE_ANALOG &&
                hdr->analog_bits != GD32LA_ANALOG_BITS) {
                sr_err("Device streams %u bit analog samples, expected %u.",
                       (unsigned int)hdr->analog_bits,
                       (unsigned int)GD32LA_ANALOG_BITS);
                return SR_ERR;
            }

            sr_info("Sample stream started: %u Hz, %s mode, %u byte packets.",
                    (unsigned int)hdr->samplerate,
                    (hdr->mode == GD32LA_MODE_ANALOG) ? "analog" : "logic",
                    (unsigned int)hdr->packet_size);
            return SR_OK;
        }

        sr_dbg("Discarding %d stale bytes while looking for the header.",
               transferred);
    }

    sr_err("No valid sample stream header received.");
    return SR_ERR;
}

/* -------------------------------------------------------------------------
 * Data feed
 * ---------------------------------------------------------------------- */

/* Tell DSView the capture is over and stop the polling source. */
static void la_finish(struct gd32la_context *devc)
{
    struct sr_datafeed_packet packet;

    if (devc->ended)
        return;

    devc->ended = TRUE;

    packet.status = SR_PKT_OK;
    packet.type = SR_DF_END;
    packet.payload = NULL;
    ds_data_forward(devc->sdi, &packet);

    sr_session_source_remove(-1);
}

/* Cancel every outstanding transfer and wait until libusb reaped them, so the
 * buffers can be freed safely. */
static void la_stop_transfers(struct gd32la_context *devc)
{
    struct drv_context *drvc = di->priv;
    int i;
    int guard = 100;

    for (i = 0; i < GD32LA_NUM_TRANSFERS; i++) {
        if (devc->transfers[i] != NULL)
            libusb_cancel_transfer(devc->transfers[i]);
    }

    while (devc->outstanding > 0 && guard-- > 0) {
        struct timeval tv;

        tv.tv_sec = 0;
        tv.tv_usec = 5000;
        libusb_handle_events_timeout_completed(drvc->sr_ctx->libusb_ctx, &tv, NULL);
    }

    for (i = 0; i < GD32LA_NUM_TRANSFERS; i++) {
        if (devc->transfers[i] != NULL) {
            libusb_free_transfer(devc->transfers[i]);
            devc->transfers[i] = NULL;
        }
        g_free(devc->buffers[i]);
        devc->buffers[i] = NULL;
    }

    devc->outstanding = 0;
}

static void LIBUSB_CALL la_transfer_cb(struct libusb_transfer *transfer)
{
    struct gd32la_context *devc = transfer->user_data;
    struct sr_datafeed_packet packet;
    struct sr_datafeed_logic logic;

    if (transfer->status == LIBUSB_TRANSFER_CANCELLED) {
        devc->outstanding--;
        return;
    }

    if (transfer->status != LIBUSB_TRANSFER_COMPLETED) {
        sr_err("Bulk transfer failed: %d.", transfer->status);
        devc->outstanding--;
        if (!devc->abort) {
            devc->abort = TRUE;
            la_finish(devc);
        }
        return;
    }

    if (transfer->actual_length > 0 && !devc->abort && !devc->limit_reached) {
        packet.status = SR_PKT_OK;

        if (devc->sdi->mode == ANALOG) {
            struct sr_datafeed_analog analog;

            packet.type = SR_DF_ANALOG;
            packet.payload = &analog;

            analog.probes = devc->sdi->channels;
            analog.num_samples = transfer->actual_length / (int)GD32LA_ANALOG_GROUP_BYTES;
            analog.unit_bits = GD32LA_ANALOG_BITS;
            analog.unit_pitch = 0;
            analog.mq = SR_MQ_VOLTAGE;
            analog.unit = SR_UNIT_VOLT;
            analog.mqflags = SR_MQFLAG_AC;
            analog.data = transfer->buffer;

            ds_data_forward(devc->sdi, &packet);

            devc->sent_samples += (uint64_t)analog.num_samples;
        } else {
            packet.type = SR_DF_LOGIC;
            packet.payload = &logic;

            logic.length = transfer->actual_length;
            logic.format = LA_CROSS_DATA;
            logic.index = 0;
            logic.order = 0;
            logic.unitsize = 1;
            logic.data_error = 0;
            logic.error_pattern = 0;
            logic.data = transfer->buffer;

            ds_data_forward(devc->sdi, &packet);

            devc->sent_samples += transfer->actual_length * 8 /
                                  GD32LA_CHANNELS;
        }

        if (!devc->is_loop && devc->limit_samples > 0 &&
            devc->sent_samples >= devc->limit_samples) {
            sr_info("Sample limit reached (%" PRIu64 " samples).",
                    devc->sent_samples);
            devc->limit_reached = TRUE;
        }
    }

    if (!devc->abort && !devc->limit_reached) {
        if (libusb_submit_transfer(transfer) == 0)
            return;
        sr_err("Failed to resubmit the bulk transfer.");
    }

    devc->outstanding--;
    if (devc->outstanding <= 0) {
        devc->outstanding = 0;
        la_finish(devc);
    }
}

static int la_start_transfers(struct gd32la_context *devc,
                              struct sr_usb_dev_inst *usb)
{
    int i;

    devc->outstanding = 0;

    for (i = 0; i < GD32LA_NUM_TRANSFERS; i++) {
        devc->buffers[i] = g_try_malloc0(GD32LA_TRANSFER_SIZE);
        if (devc->buffers[i] == NULL) {
            sr_err("Out of memory for transfer buffer.");
            la_stop_transfers(devc);
            return SR_ERR_MALLOC;
        }

        devc->transfers[i] = libusb_alloc_transfer(0);
        if (devc->transfers[i] == NULL) {
            sr_err("libusb_alloc_transfer() failed.");
            la_stop_transfers(devc);
            return SR_ERR_MALLOC;
        }

        libusb_fill_bulk_transfer(devc->transfers[i], usb->devhdl,
                                  GD32LA_EP_IN, devc->buffers[i],
                                  GD32LA_TRANSFER_SIZE, la_transfer_cb,
                                  devc, 0);   /* 0 = no timeout, stop cancels */

        if (libusb_submit_transfer(devc->transfers[i]) != 0) {
            sr_err("Failed to submit the bulk transfer.");
            devc->outstanding = i;
            la_stop_transfers(devc);
            return SR_ERR;
        }

        devc->outstanding++;
    }

    return SR_OK;
}

/* Timer driven source: pump libusb so the transfer callbacks run. */
static int la_receive_data(int fd, int revents, const struct sr_dev_inst *sdi)
{
    struct gd32la_context *devc = sdi->priv;
    struct drv_context *drvc = di->priv;
    struct timeval tv;

    (void)fd;

    tv.tv_sec = 0;
    tv.tv_usec = 0;
    libusb_handle_events_timeout_completed(drvc->sr_ctx->libusb_ctx, &tv, NULL);

    /* revents == -1: the session aborted the acquisition. */
    if (revents == -1 && !devc->abort) {
        sr_info("Acquisition aborted by the session.");
        devc->abort = TRUE;
    }

    if (devc->abort && devc->outstanding <= 0)
        la_finish(devc);

    return TRUE;
}

/* -------------------------------------------------------------------------
 * Driver callbacks
 * ---------------------------------------------------------------------- */

static int hw_init(struct sr_context *sr_ctx)
{
    return std_hw_init(sr_ctx, di, LOG_PREFIX);
}

static int hw_cleanup(void)
{
    if (di->priv != NULL) {
        g_free(di->priv);
        di->priv = NULL;
    }
    return SR_OK;
}

static GSList *hw_scan(GSList *options)
{
    struct drv_context *drvc;
    struct sr_dev_inst *sdi;
    struct gd32la_context *devc;
    struct sr_usb_dev_inst *usb;
    struct libusb_device **devlist;
    struct libusb_device_descriptor des;
    GSList *devices = NULL;
    libusb_device *device;
    int i;

    (void)options;

    drvc = di->priv;
    if (drvc == NULL)
        return NULL;

    if (libusb_get_device_list(drvc->sr_ctx->libusb_ctx, &devlist) < 0) {
        sr_err("Failed to enumerate USB devices.");
        return NULL;
    }

    for (i = 0; devlist[i]; i++) {
        device = devlist[i];

        if (libusb_get_device_descriptor(device, &des) != 0)
            continue;

        if (des.idVendor != GD32LA_VID || des.idProduct != GD32LA_PID)
            continue;

        if (sr_usb_device_is_exists(device)) {
            sr_dbg("Device %04x:%04x is already in the list.",
                   des.idVendor, des.idProduct);
            continue;
        }

        devc = g_try_malloc0(sizeof(struct gd32la_context));
        if (devc == NULL)
            break;

        devc->samplerate = GD32LA_DEFAULT_SAMPLERATE;
        devc->limit_samples = GD32LA_DEFAULT_LIMIT_SAMPLES;
        devc->channels_enabled = GD32LA_CHANNELS;
        devc->is_loop = FALSE;
        devc->ended = TRUE;

        sdi = sr_dev_inst_new(LOGIC, SR_ST_INACTIVE, "GigaDevice",
                              "GD32F470 LA Simulator", "1.0");
        if (sdi == NULL) {
            g_free(devc);
            break;
        }

        sdi->priv = devc;
        sdi->driver = di;
        sdi->dev_type = DEV_TYPE_USB;
        devc->sdi = sdi;

        usb = sr_usb_dev_inst_new(libusb_get_bus_number(device),
                                  libusb_get_device_address(device));
        if (usb == NULL) {
            dev_destroy(sdi);
            break;
        }
        usb->usb_dev = device;
        sdi->conn = usb;

        la_rebuild_channels(sdi);

        sr_info("Found device %04x:%04x on bus %d address %d.",
                des.idVendor, des.idProduct, usb->bus, usb->address);

        devices = g_slist_append(devices, sdi);
    }

    libusb_free_device_list(devlist, 0);

    return devices;
}

static const GSList *hw_dev_mode_list(const struct sr_dev_inst *sdi)
{
    GSList *l = NULL;

    (void)sdi;

    /* The same device can run as a logic analyzer or as a two channel data
     * acquisition unit, DSView lets the user pick one per session. */
    l = g_slist_append(l, (gpointer)&sr_mode_list[0]);   /* LOGIC  */
    l = g_slist_append(l, (gpointer)&sr_mode_list[1]);   /* ANALOG */

    return l;
}

/* Build the channel list that matches sdi->mode. Called from the scan and
 * whenever the user switches the work mode. */
static void la_rebuild_channels(struct sr_dev_inst *sdi)
{
    struct gd32la_context *devc = sdi->priv;
    gboolean analog = (sdi->mode == ANALOG);

    sr_dev_probes_free(sdi);

    for (uint16_t j = 0; j < GD32LA_CHANNELS; j++) {
        struct sr_channel *probe;

        probe = sr_channel_new(j,
                               analog ? SR_CHANNEL_ANALOG : SR_CHANNEL_LOGIC,
                               TRUE,
                               analog ? analog_probe_names[j] : logic_probe_names[j]);
        if (probe == NULL) {
            sr_err("Failed to create channel %u.", (unsigned int)j);
            break;
        }

        if (analog) {
            /* Same defaults the DAQ demo device uses: 8 bit samples centred
             * on 128, 1 V/div, AC coupling, -5 V .. +5 V display range. */
            probe->bits = GD32LA_ANALOG_BITS;
            probe->coupling = SR_AC_COUPLING;
            probe->vdiv = SR_V(1);
            probe->vfactor = SR_mV(1);
            probe->hw_offset = GD32LA_ANALOG_MID;
            probe->offset = GD32LA_ANALOG_MID;
            probe->trig_value = GD32LA_ANALOG_MID;
            probe->map_default = TRUE;
            probe->map_unit = map_units[0];
            probe->map_min = -5.0;
            probe->map_max = 5.0;
        } else {
            probe->bits = 1;
        }

        sdi->channels = g_slist_append(sdi->channels, probe);
    }

    if (devc != NULL)
        devc->channels_enabled = GD32LA_CHANNELS;
}

static int hw_dev_open(struct sr_dev_inst *sdi)
{
    struct sr_usb_dev_inst *usb;
    int ret;

    if (sdi->status == SR_ST_ACTIVE)
        return SR_OK;

    usb = sdi->conn;
    if (usb == NULL || usb->usb_dev == NULL) {
        sr_err("No USB device handle available.");
        return SR_ERR;
    }

    ret = libusb_open(usb->usb_dev, &usb->devhdl);
    if (ret != 0 || usb->devhdl == NULL) {
        sr_err("Failed to open the device: %s.", libusb_error_name(ret));
        ds_set_last_error(SR_ERR_DEVICE_USB_IO_ERROR);
        return SR_ERR;
    }

    /* On Windows the WinUSB driver is bound through an inf file, there is no
     * kernel driver to detach. */
    if (libusb_kernel_driver_active(usb->devhdl, GD32LA_IFACE) == 1) {
        if (libusb_detach_kernel_driver(usb->devhdl, GD32LA_IFACE) != 0)
            sr_warn("Failed to detach the kernel driver.");
    }

    ret = libusb_claim_interface(usb->devhdl, GD32LA_IFACE);
    if (ret != 0) {
        sr_err("Failed to claim interface %d: %s.", GD32LA_IFACE,
               libusb_error_name(ret));
        libusb_close(usb->devhdl);
        usb->devhdl = NULL;
        ds_set_last_error(SR_ERR_DEVICE_USB_IO_ERROR);
        return SR_ERR;
    }

    sdi->status = SR_ST_ACTIVE;
    sr_info("Device opened.");

    return SR_OK;
}

static int hw_dev_close(struct sr_dev_inst *sdi)
{
    struct sr_usb_dev_inst *usb = sdi->conn;
    struct gd32la_context *devc = sdi->priv;

    if (usb == NULL || usb->devhdl == NULL) {
        sdi->status = SR_ST_INACTIVE;
        return SR_OK;
    }

    if (devc != NULL) {
        devc->abort = TRUE;
        la_send_cmd(usb, GD32LA_CMD_STOP, 0, 0);
        la_stop_transfers(devc);
        devc->ended = TRUE;
    }

    libusb_release_interface(usb->devhdl, GD32LA_IFACE);
    libusb_close(usb->devhdl);
    usb->devhdl = NULL;

    sdi->status = SR_ST_INACTIVE;
    sr_info("Device closed.");

    return SR_OK;
}

static int dev_destroy(struct sr_dev_inst *sdi)
{
    hw_dev_close(sdi);
    sr_dev_inst_free(sdi);

    return SR_OK;
}

static int hw_dev_acquisition_start(struct sr_dev_inst *sdi, void *cb_data)
{
    struct gd32la_context *devc = sdi->priv;
    struct sr_usb_dev_inst *usb = sdi->conn;
    uint32_t samplerate;
    uint8_t mode;
    int ret;

    (void)cb_data;

    if (sdi->status != SR_ST_ACTIVE) {
        sr_err("Device is not open.");
        ds_set_last_error(SR_ERR_DEVICE_CLOSED);
        return SR_ERR_DEVICE_CLOSED;
    }

    devc->sdi = sdi;
    devc->sent_samples = 0;
    devc->abort = FALSE;
    devc->limit_reached = FALSE;
    devc->ended = FALSE;

    if (sdi->mode == ANALOG) {
        mode = GD32LA_MODE_ANALOG;

        /* Two 8 bit channels at the requested rate have to fit the full
         * speed bulk pipe. */
        if (devc->samplerate > GD32LA_MAX_ASRATE) {
            sr_warn("Analog mode tops out at %" PRIu64 " Hz, clamping the "
                    "samplerate.", (uint64_t)GD32LA_MAX_ASRATE);
            devc->samplerate = GD32LA_MAX_ASRATE;
        }
        if (devc->samplerate < GD32LA_MIN_SAMPLERATE)
            devc->samplerate = GD32LA_DEFAULT_ASRATE;
        if (devc->limit_samples == 0)
            devc->limit_samples = GD32LA_DEFAULT_ALIMIT;
    } else {
        mode = GD32LA_MODE_LOGIC;
        if (devc->samplerate < GD32LA_MIN_SAMPLERATE)
            devc->samplerate = GD32LA_DEFAULT_SAMPLERATE;
    }

    samplerate = (uint32_t)devc->samplerate;

    sr_info("Starting acquisition: %" PRIu64 " Hz, limit %" PRIu64
            " samples, %s mode.", devc->samplerate, devc->limit_samples,
            (mode == GD32LA_MODE_ANALOG) ? "analog" : "logic");

    /* Stop whatever the device was doing and flush the pipe, so the header
     * of the new stream is the first thing we read. */
    la_send_cmd(usb, GD32LA_CMD_STOP, 0, 0);
    la_drain(usb);

    ret = la_send_cmd(usb, GD32LA_CMD_START, mode, samplerate);
    if (ret != SR_OK) {
        ds_set_last_error(SR_ERR_DEVICE_USB_IO_ERROR);
        return ret;
    }

    ret = la_read_header(usb);
    if (ret != SR_OK) {
        la_send_cmd(usb, GD32LA_CMD_STOP, 0, 0);
        ds_set_last_error(SR_ERR_DEVICE_USB_IO_ERROR);
        return ret;
    }

    ret = la_start_transfers(devc, usb);
    if (ret != SR_OK) {
        la_send_cmd(usb, GD32LA_CMD_STOP, 0, 0);
        return ret;
    }

    sr_session_source_add(-1, 0, GD32LA_POLL_INTERVAL_MS, la_receive_data, sdi);

    return SR_OK;
}

static int hw_dev_acquisition_stop(const struct sr_dev_inst *sdi, void *cb_data)
{
    struct gd32la_context *devc = sdi->priv;
    struct sr_usb_dev_inst *usb = sdi->conn;

    (void)cb_data;

    if (devc == NULL)
        return SR_OK;

    sr_info("Stopping acquisition.");

    devc->abort = TRUE;

    if (usb != NULL && usb->devhdl != NULL)
        la_send_cmd(usb, GD32LA_CMD_STOP, 0, 0);

    la_stop_transfers(devc);
    la_finish(devc);

    return SR_OK;
}

static int hw_dev_status_get(const struct sr_dev_inst *sdi,
                             struct sr_status *status, gboolean prg)
{
    (void)sdi;
    (void)status;
    (void)prg;

    return SR_ERR_NA;
}

/* -------------------------------------------------------------------------
 * Configuration
 * ---------------------------------------------------------------------- */

static int config_get(int id, GVariant **data, const struct sr_dev_inst *sdi,
                      const struct sr_channel *ch,
                      const struct sr_channel_group *cg)
{
    struct gd32la_context *devc;

    (void)cg;

    if (sdi == NULL || sdi->priv == NULL)
        return SR_ERR_ARG;

    devc = sdi->priv;

    switch (id) {
    case SR_CONF_SAMPLERATE:
        *data = g_variant_new_uint64(devc->samplerate);
        break;
    case SR_CONF_LIMIT_SAMPLES:
        *data = g_variant_new_uint64(devc->limit_samples);
        break;
    case SR_CONF_HW_DEPTH:
        *data = g_variant_new_uint64(GD32LA_MAX_SAMPLES);
        break;
    case SR_CONF_DEVICE_MODE:
        *data = g_variant_new_int16(sdi->mode);
        break;
    case SR_CONF_UNIT_BITS:
        *data = g_variant_new_byte((sdi->mode == ANALOG) ?
                                   GD32LA_ANALOG_BITS : 1);
        break;
    case SR_CONF_VLD_CH_NUM:
        *data = g_variant_new_int16(GD32LA_CHANNELS);
        break;
    case SR_CONF_HAVE_ZERO:
        *data = g_variant_new_boolean(FALSE);
        break;
    case SR_CONF_LOOP_MODE:
        *data = g_variant_new_boolean(devc->is_loop);
        break;
    case SR_CONF_PROBE_EN:
        if (ch == NULL)
            return SR_ERR_ARG;
        *data = g_variant_new_boolean(ch->enabled);
        break;
    case SR_CONF_PROBE_OFFSET:
        if (ch == NULL)
            return SR_ERR_ARG;
        *data = g_variant_new_uint16(ch->offset);
        break;
    case SR_CONF_PROBE_HW_OFFSET:
        if (ch == NULL)
            return SR_ERR_ARG;
        *data = g_variant_new_uint16(ch->hw_offset);
        break;
    case SR_CONF_PROBE_VDIV:
        if (ch == NULL)
            return SR_ERR_ARG;
        *data = g_variant_new_uint64(ch->vdiv);
        break;
    case SR_CONF_PROBE_FACTOR:
        if (ch == NULL)
            return SR_ERR_ARG;
        *data = g_variant_new_uint64(ch->vfactor);
        break;
    case SR_CONF_PROBE_COUPLING:
        if (ch == NULL)
            return SR_ERR_ARG;
        *data = g_variant_new_byte(ch->coupling);
        break;
    case SR_CONF_TRIGGER_VALUE:
        if (ch == NULL)
            return SR_ERR_ARG;
        *data = g_variant_new_byte(ch->trig_value);
        break;
    case SR_CONF_PROBE_MAP_DEFAULT:
        if (ch == NULL)
            return SR_ERR_ARG;
        *data = g_variant_new_boolean(ch->map_default);
        break;
    case SR_CONF_PROBE_MAP_UNIT:
        if (ch == NULL || ch->map_unit == NULL)
            return SR_ERR_ARG;
        *data = g_variant_new_string(ch->map_unit);
        break;
    case SR_CONF_PROBE_MAP_MIN:
        if (ch == NULL)
            return SR_ERR_ARG;
        *data = g_variant_new_double(ch->map_min);
        break;
    case SR_CONF_PROBE_MAP_MAX:
        if (ch == NULL)
            return SR_ERR_ARG;
        *data = g_variant_new_double(ch->map_max);
        break;
    default:
        return SR_ERR_NA;
    }

    return SR_OK;
}

static int config_set(int id, GVariant *data, struct sr_dev_inst *sdi,
                      struct sr_channel *ch,
                      struct sr_channel_group *cg)
{
    struct gd32la_context *devc;
    int mode;

    (void)cg;

    if (sdi == NULL || sdi->priv == NULL)
        return SR_ERR_ARG;

    devc = sdi->priv;

    switch (id) {
    case SR_CONF_SAMPLERATE:
        devc->samplerate = g_variant_get_uint64(data);
        sr_dbg("Samplerate set to %" PRIu64 ".", devc->samplerate);
        break;
    case SR_CONF_LIMIT_SAMPLES:
        devc->limit_samples = g_variant_get_uint64(data);
        sr_dbg("Sample limit set to %" PRIu64 ".", devc->limit_samples);
        break;
    case SR_CONF_DEVICE_MODE:
        mode = g_variant_get_int16(data);
        if (mode != LOGIC && mode != ANALOG) {
            sr_err("Unsupported work mode %d.", mode);
            return SR_ERR_ARG;
        }
        if (sdi->mode != mode) {
            sdi->mode = mode;
            la_rebuild_channels(sdi);

            /* Each mode has its own sane starting point. */
            if (mode == ANALOG) {
                devc->samplerate = GD32LA_DEFAULT_ASRATE;
                devc->limit_samples = GD32LA_DEFAULT_ALIMIT;
            } else {
                devc->samplerate = GD32LA_DEFAULT_SAMPLERATE;
                devc->limit_samples = GD32LA_DEFAULT_LIMIT_SAMPLES;
            }

            sr_info("Work mode set to %s.",
                    (mode == ANALOG) ? "analog" : "logic");
        }
        break;
    case SR_CONF_LOOP_MODE:
        devc->is_loop = g_variant_get_boolean(data);
        sr_dbg("Loop mode: %s.", devc->is_loop ? "on" : "off");
        break;
    case SR_CONF_PROBE_EN:
        if (ch == NULL)
            return SR_ERR_ARG;
        ch->enabled = g_variant_get_boolean(data);
        break;
    case SR_CONF_PROBE_OFFSET:
        if (ch == NULL)
            return SR_ERR_ARG;
        ch->offset = g_variant_get_uint16(data);
        break;
    case SR_CONF_PROBE_HW_OFFSET:
        if (ch == NULL)
            return SR_ERR_ARG;
        ch->hw_offset = g_variant_get_uint16(data);
        break;
    case SR_CONF_PROBE_VDIV:
        if (ch == NULL)
            return SR_ERR_ARG;
        ch->vdiv = g_variant_get_uint64(data);
        break;
    case SR_CONF_PROBE_FACTOR:
        if (ch == NULL)
            return SR_ERR_ARG;
        ch->vfactor = g_variant_get_uint64(data);
        break;
    case SR_CONF_PROBE_COUPLING:
        if (ch == NULL)
            return SR_ERR_ARG;
        ch->coupling = g_variant_get_byte(data);
        break;
    case SR_CONF_TRIGGER_VALUE:
        if (ch == NULL)
            return SR_ERR_ARG;
        ch->trig_value = g_variant_get_byte(data);
        break;
    case SR_CONF_PROBE_MAP_DEFAULT:
        if (ch == NULL)
            return SR_ERR_ARG;
        ch->map_default = g_variant_get_boolean(data);
        if (ch->map_default) {
            ch->map_unit = map_units[0];
            ch->map_min = -5.0;
            ch->map_max = 5.0;
        }
        break;
    case SR_CONF_PROBE_MAP_UNIT:
        if (ch == NULL)
            return SR_ERR_ARG;
        ch->map_unit = g_variant_get_string(data, NULL);
        break;
    case SR_CONF_PROBE_MAP_MIN:
        if (ch == NULL)
            return SR_ERR_ARG;
        ch->map_min = g_variant_get_double(data);
        break;
    case SR_CONF_PROBE_MAP_MAX:
        if (ch == NULL)
            return SR_ERR_ARG;
        ch->map_max = g_variant_get_double(data);
        break;
    default:
        return SR_ERR_NA;
    }

    return SR_OK;
}

static int config_list(int key, GVariant **data, const struct sr_dev_inst *sdi,
                       const struct sr_channel_group *cg)
{
    GVariant *gvar;
    GVariantBuilder gvb;
    const uint64_t *rates;
    gsize rate_count;

    (void)cg;

    if (sdi != NULL && sdi->mode == ANALOG) {
        rates = analog_samplerates;
        rate_count = ARRAY_SIZE(analog_samplerates);
    } else {
        rates = logic_samplerates;
        rate_count = ARRAY_SIZE(logic_samplerates);
    }

    switch (key) {
    case SR_CONF_DEVICE_SESSIONS:
        *data = g_variant_new_from_data(G_VARIANT_TYPE("ai"), sessions,
                                        ARRAY_SIZE(sessions) * sizeof(int32_t),
                                        TRUE, NULL, NULL);
        break;
    case SR_CONF_DEVICE_OPTIONS:
        /* The simulator has no hardware options worth exposing. */
        return SR_ERR_NA;
    case SR_CONF_SAMPLERATE:
        g_variant_builder_init(&gvb, G_VARIANT_TYPE("a{sv}"));
        gvar = g_variant_new_from_data(G_VARIANT_TYPE("at"), rates,
                                       rate_count * sizeof(uint64_t),
                                       TRUE, NULL, NULL);
        g_variant_builder_add(&gvb, "{sv}", "samplerates", gvar);
        *data = g_variant_builder_end(&gvb);
        break;
    case SR_CONF_PROBE_CONFIGS:
        *data = g_variant_new_from_data(G_VARIANT_TYPE("ai"), probe_options,
                                        ARRAY_SIZE(probe_options) * sizeof(int32_t),
                                        TRUE, NULL, NULL);
        break;
    case SR_CONF_PROBE_VDIV:
        g_variant_builder_init(&gvb, G_VARIANT_TYPE("a{sv}"));
        gvar = g_variant_new_from_data(G_VARIANT_TYPE("at"), vdivs,
                                       ARRAY_SIZE(vdivs) * sizeof(uint64_t),
                                       TRUE, NULL, NULL);
        g_variant_builder_add(&gvb, "{sv}", "vdivs", gvar);
        *data = g_variant_builder_end(&gvb);
        break;
    case SR_CONF_PROBE_COUPLING:
        g_variant_builder_init(&gvb, G_VARIANT_TYPE("a{sv}"));
        gvar = g_variant_new_from_data(G_VARIANT_TYPE("ay"), couplings,
                                       ARRAY_SIZE(couplings) * sizeof(uint8_t),
                                       TRUE, NULL, NULL);
        g_variant_builder_add(&gvb, "{sv}", "coupling", gvar);
        *data = g_variant_builder_end(&gvb);
        break;
    case SR_CONF_PROBE_MAP_UNIT:
        *data = g_variant_new_strv((const char * const *)map_units,
                                   ARRAY_SIZE(map_units));
        break;
    default:
        return SR_ERR_NA;
    }

    return SR_OK;
}

SR_PRIV struct sr_dev_driver gd32la_driver_info = {
    .name = "gd32la",
    .longname = "GD32F470 logic analyzer and DAQ simulator",
    .api_version = 1,
    .driver_type = DRIVER_TYPE_HARDWARE,
    .init = hw_init,
    .cleanup = hw_cleanup,
    .scan = hw_scan,
    .dev_mode_list = hw_dev_mode_list,
    .config_get = config_get,
    .config_set = config_set,
    .config_list = config_list,
    .dev_open = hw_dev_open,
    .dev_close = hw_dev_close,
    .dev_destroy = dev_destroy,
    .dev_status_get = hw_dev_status_get,
    .dev_acquisition_start = hw_dev_acquisition_start,
    .dev_acquisition_stop = hw_dev_acquisition_stop,
    .priv = NULL,
};
