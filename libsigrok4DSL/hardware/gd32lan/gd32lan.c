/*
 * This file is part of the libsigrok project.
 *
 * GD32 16 channel data acquisition simulator - network driver (TCP client).
 *
 * The board streams 16 analog channels (16 bit) over TWO UARTs, both
 * 1500000 8N1. Two serial to Ethernet servers in TCP server mode bridge
 * them to the network; the device alternates the frames between the links
 * (even sequence numbers on link 0, odd on link 1), and this driver
 * connects as a TCP client to both, merges the frames by sequence number
 * and feeds the ordered stream to the session.
 *
 * Address of the serial servers comes from the environment:
 *
 *   GD32LA_HOST    default 192.168.1.100
 *   GD32LA_PORT    default 2404   (link 0, even frames)
 *   GD32LA_PORT2   default 2405   (link 1, odd frames)
 *
 * Transfer model: DSView's session poll (g_poll) is not reliable for
 * sockets on Windows, so one receive thread per link pumps its socket
 * into a queue and a 5 ms session timer merges and drains both queues in
 * the session thread, where ds_data_forward() is safe to call.
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 */

#include <string.h>
#include <stdlib.h>

#ifdef _WIN32
#include <ws2tcpip.h>
#else
#include <sys/types.h>
#include <sys/socket.h>
#include <netdb.h>
#include <unistd.h>
#include <fcntl.h>
#include <errno.h>
#include <netinet/in.h>
#include <netinet/tcp.h>
#define closesocket(fd)     close(fd)
#endif

#include "gd32lan.h"
#include "../../log.h"

#undef LOG_PREFIX
#define LOG_PREFIX "gd32lan: "

static struct sr_dev_driver *di = &gd32lan_driver_info;

static int dev_destroy(struct sr_dev_inst *sdi);
static void lan_rebuild_channels(struct sr_dev_inst *sdi);
static void lan_stop_link(struct gd32lan_context *devc);

static const char *analog_probe_names[] = {
    "A0", "A1", "A2", "A3", "A4", "A5", "A6", "A7",
    "A8", "A9", "A10", "A11", "A12", "A13", "A14", "A15",
    NULL,
};

/* Samplerates the two 1500000 baud serial links can carry:
 * analog = rate * 32 B/s (16 channels, 2 bytes each). */
static const uint64_t analog_samplerates[] = {
    SR_KHZ(2),      /*  64 kB/s, 35% of the 184 kB/s a 921600 link carries */
    SR_KHZ(5),      /* 160 kB/s, 87% - real time, occasional frame losses */
    SR_HZ(6400),    /* 204.8 kB/s against 184 kB/s: the device FIFO absorbs
                     * the 10% deficit, a capture arrives slightly late */
};

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
 * Probe helpers (identical to the USB driver)
 * ---------------------------------------------------------------------- */

static void lan_count_probes(const struct sr_dev_inst *sdi, guint *logic,
                             guint *analog)
{
    const GSList *l;

    *logic = 0;
    *analog = 0;

    for (l = sdi->channels; l; l = l->next) {
        const struct sr_channel *probe = l->data;

        if (probe->type == SR_CHANNEL_LOGIC)
            (*logic)++;
        else if (probe->type == SR_CHANNEL_ANALOG)
            (*analog)++;
    }
}

static gboolean lan_analog_in_use(const struct sr_dev_inst *sdi)
{
    const GSList *l;

    for (l = sdi->channels; l; l = l->next) {
        const struct sr_channel *probe = l->data;

        if (probe->type == SR_CHANNEL_ANALOG && probe->enabled)
            return TRUE;
    }

    return FALSE;
}

/* -------------------------------------------------------------------------
 * Socket helpers
 * ---------------------------------------------------------------------- */

static void lan_set_nonblocking(SOCKET fd)
{
#ifdef _WIN32
    u_long mode = 1;

    ioctlsocket(fd, FIONBIO, &mode);
#else
    int flags = fcntl(fd, F_GETFL, 0);

    if (flags >= 0)
        fcntl(fd, F_SETFL, flags | O_NONBLOCK);
#endif
}

static void lan_set_blocking(SOCKET fd)
{
#ifdef _WIN32
    u_long mode = 0;

    ioctlsocket(fd, FIONBIO, &mode);
#else
    int flags = fcntl(fd, F_GETFL, 0);

    if (flags >= 0)
        fcntl(fd, F_SETFL, flags & ~O_NONBLOCK);
#endif
}

/* Connect with a timeout. Returns SR_OK and a blocking socket, or SR_ERR. */
static int lan_connect(const char *host, const char *port, SOCKET *out)
{
    struct addrinfo hints;
    struct addrinfo *res = NULL;
    struct addrinfo *ai;
    SOCKET fd = INVALID_SOCKET;
    int ret = SR_ERR;

    memset(&hints, 0, sizeof(hints));
    hints.ai_family = AF_UNSPEC;
    hints.ai_socktype = SOCK_STREAM;

    if (getaddrinfo(host, port, &hints, &res) != 0) {
        sr_err("Cannot resolve %s:%s.", host, port);
        return SR_ERR;
    }

    for (ai = res; ai != NULL; ai = ai->ai_next) {
        int rc;
        int err = 0;
        fd_set wfds;
        struct timeval tv;
        socklen_t err_len = (socklen_t)sizeof(err);

        fd = socket(ai->ai_family, ai->ai_socktype, ai->ai_protocol);
        if (fd == INVALID_SOCKET)
            continue;

        lan_set_nonblocking(fd);

        rc = connect(fd, ai->ai_addr, (int)ai->ai_addrlen);
#ifdef _WIN32
        if (rc != 0 && WSAGetLastError() == WSAEWOULDBLOCK)
#else
        if (rc != 0 && errno == EINPROGRESS)
#endif
        {
            FD_ZERO(&wfds);
            FD_SET(fd, &wfds);
            tv.tv_sec = GD32LAN_CONNECT_TIMEOUT_MS / 1000;
            tv.tv_usec = (GD32LAN_CONNECT_TIMEOUT_MS % 1000) * 1000;

            rc = select((int)fd + 1, NULL, &wfds, NULL, &tv);
            if (rc > 0) {
                if (getsockopt(fd, SOL_SOCKET, SO_ERROR,
                               (char *)&err, &err_len) == 0 && err == 0)
                    rc = 0;     /* connected */
                else
                    rc = -1;
            } else {
                rc = -1;        /* timeout or select error */
            }
        }

        if (rc == 0) {
            int one = 1;

            lan_set_blocking(fd);
            setsockopt(fd, IPPROTO_TCP, TCP_NODELAY, (char *)&one,
                       sizeof(one));
            ret = SR_OK;
            break;
        }

        closesocket(fd);
        fd = INVALID_SOCKET;
    }

    freeaddrinfo(res);

    if (ret != SR_OK) {
        sr_err("Cannot connect to %s:%s (timeout %d ms).", host, port,
               GD32LAN_CONNECT_TIMEOUT_MS);
        return SR_ERR;
    }

    *out = fd;
    return SR_OK;
}

/* Send the whole buffer, returns SR_OK when every byte is gone. */
static int lan_send_all(SOCKET fd, const void *data, int len)
{
    const char *p = data;

    while (len > 0) {
        int n = send(fd, p, len, 0);

        if (n <= 0)
            return SR_ERR;

        p += n;
        len -= n;
    }

    return SR_OK;
}

/* Write one 8 byte command record to one link. */
static int lan_send_cmd_fd(SOCKET fd, uint8_t cmd, uint8_t arg, uint32_t value)
{
    gd32la_cmd_t record;

    if (fd == INVALID_SOCKET)
        return SR_ERR;

    record.cmd = cmd;
    record.arg = arg;
    record.reserved = 0;
    record.value = value;

    if (lan_send_all(fd, &record, (int)sizeof(record)) != SR_OK) {
        sr_err("Failed to send command 0x%02x.", cmd);
        return SR_ERR;
    }

    return SR_OK;
}

/* Send a command to every open link. */
static int lan_send_cmd(struct gd32lan_context *devc, uint8_t cmd,
                        uint8_t arg, uint32_t value)
{
    int ret = SR_OK;

    if (lan_send_cmd_fd(devc->sock, cmd, arg, value) != SR_OK)
        ret = SR_ERR;
    if (lan_send_cmd_fd(devc->sock2, cmd, arg, value) != SR_OK)
        ret = SR_ERR;

    return ret;
}

/* Read and discard whatever the serial server still has queued, so a new
 * START always begins on a header boundary. */
static void lan_drain(SOCKET fd)
{
    uint8_t buf[1024];
    int guard = 256;

#ifdef _WIN32
    DWORD tv = 100;

    setsockopt(fd, SOL_SOCKET, SO_RCVTIMEO, (const char *)&tv, sizeof(tv));
#else
    struct timeval tv;

    tv.tv_sec = 0;
    tv.tv_usec = 100 * 1000;
    setsockopt(fd, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));
#endif

    while (guard-- > 0) {
        int n = recv(fd, (char *)buf, (int)sizeof(buf), 0);

        if (n <= 0)
            break;
    }
}

/* Receive exactly len bytes with a per call timeout, returns SR_OK when the
 * buffer is full. A dead link or too many timeouts gives SR_ERR. */
static int lan_recv_exact(SOCKET fd, uint8_t *buf, int len, int max_wait_ms)
{
    int got = 0;
    int waited = 0;

    while (got < len) {
        int n = recv(fd, (char *)buf + got, len - got, 0);

        if (n > 0) {
            got += n;
            continue;
        }
        if (n == 0)
            return SR_ERR;      /* orderly shutdown */

#ifdef _WIN32
        if (WSAGetLastError() == WSAETIMEDOUT ||
            WSAGetLastError() == WSAEWOULDBLOCK)
#else
        if (errno == EAGAIN || errno == EWOULDBLOCK)
#endif
        {
            waited += GD32LAN_RECV_STEP_MS;
            if (waited >= max_wait_ms)
                return SR_ERR;
            continue;
        }

        return SR_ERR;
    }

    return SR_OK;
}

static const char *lan_mode_name(guint8 mode)
{
    switch (mode) {
    case GD32LA_MODE_ANALOG:
        return "analog";
    case GD32LA_MODE_MIXED:
        return "mixed";
    default:
        return "logic";
    }
}

/* Wait for the header that answers GD32LA_CMD_START on one link. Stale
 * bytes left in the serial server are skipped until the magic shows up. */
static int lan_read_header(SOCKET fd, guint8 stream_mode)
{
    gd32la_hdr_t hdr;
    uint8_t window[4] = {0, 0, 0, 0};
    int waited = 0;

#ifdef _WIN32
    DWORD tv = GD32LAN_RECV_STEP_MS;

    setsockopt(fd, SOL_SOCKET, SO_RCVTIMEO, (const char *)&tv, sizeof(tv));
#else
    struct timeval tv;

    tv.tv_sec = 0;
    tv.tv_usec = GD32LAN_RECV_STEP_MS * 1000;
    setsockopt(fd, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));
#endif

    /* byte hunt for "GDLA" */
    while (waited < GD32LAN_HDR_TIMEOUT_MS) {
        uint8_t b;
        int n = recv(fd, (char *)&b, 1, 0);

        if (n == 1) {
            window[0] = window[1];
            window[1] = window[2];
            window[2] = window[3];
            window[3] = b;

            if (window[0] == GD32LA_HDR_MAGIC0 &&
                window[1] == GD32LA_HDR_MAGIC1 &&
                window[2] == GD32LA_HDR_MAGIC2 &&
                window[3] == GD32LA_HDR_MAGIC3)
                break;
            continue;
        }
        if (n == 0) {
            sr_err("Link closed while waiting for the header.");
            return SR_ERR;
        }
        waited += GD32LAN_RECV_STEP_MS;
    }

    if (waited >= GD32LAN_HDR_TIMEOUT_MS) {
        sr_err("No sample stream header within %d ms.",
               GD32LAN_HDR_TIMEOUT_MS);
        return SR_ERR;
    }

    memcpy(hdr.magic, window, 4);

    if (lan_recv_exact(fd, ((uint8_t *)&hdr) + 4,
                       GD32LA_HDR_LEN - 4, GD32LAN_HDR_TIMEOUT_MS) != SR_OK) {
        sr_err("Truncated sample stream header.");
        return SR_ERR;
    }

    if (hdr.version != GD32LA_HDR_VERSION) {
        sr_err("Unsupported device protocol version %u, expected %u.",
               (unsigned int)hdr.version, (unsigned int)GD32LA_HDR_VERSION);
        return SR_ERR;
    }
    if (hdr.mode != stream_mode) {
        sr_err("Device streams %s data, requested %s.",
               lan_mode_name(hdr.mode), lan_mode_name(stream_mode));
        return SR_ERR;
    }
    if (hdr.analog_channels != GD32LA_ANALOG_CHANNELS) {
        sr_err("Device reports %u analog channels, expected %u.",
               (unsigned int)hdr.analog_channels,
               (unsigned int)GD32LA_ANALOG_CHANNELS);
        return SR_ERR;
    }
    if (hdr.analog_bits != GD32LA_ANALOG_BITS) {
        sr_err("Device streams %u bit analog samples, expected %u.",
               (unsigned int)hdr.analog_bits,
               (unsigned int)GD32LA_ANALOG_BITS);
        return SR_ERR;
    }

    sr_info("Sample stream started on this link: %u Hz, %s mode.",
            (unsigned int)hdr.samplerate, lan_mode_name(hdr.mode));
    return SR_OK;
}

/* -------------------------------------------------------------------------
 * Data feed
 * ---------------------------------------------------------------------- */

/* Tell DSView the capture is over and stop the polling source. */
static void lan_finish(struct gd32lan_context *devc)
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

/* Check the head of a queue for a complete frame. Drops leading bytes until
 * the frame magic shows up (resynchronisation after a lost serial byte) and
 * returns FALSE while less than a whole frame is queued. The caller must
 * hold queue_lock. */
static gboolean lan_head_frame(GByteArray *queue, uint32_t *seq)
{
    while (queue->len >= GD32LA_FRAME_HEADER_BYTES) {
        if (queue->data[0] == GD32LA_FRAME_MAGIC0 &&
            queue->data[1] == GD32LA_FRAME_MAGIC1) {
            if (queue->len < GD32LA_ANALOG_FRAME_BYTES)
                return FALSE;

            *seq = (uint32_t)queue->data[2] |
                   ((uint32_t)queue->data[3] << 8) |
                   ((uint32_t)queue->data[4] << 16) |
                   ((uint32_t)queue->data[5] << 24);
            return TRUE;
        }

        g_byte_array_remove_index(queue, 0);
    }

    return FALSE;
}

/* Merge the frames of the two links back into one ordered stream and
 * forward them to the session.
 *
 * The device alternates the frames between the links: link 0 carries the
 * even sequence numbers, link 1 the odd ones. The merge therefore always
 * waits for the frame with the expected sequence number on the link its
 * parity points at - waiting is essential, a frame that has not arrived
 * yet is not lost, and grabbing the other link's newer frame instead
 * would turn a short delay into a permanent hole.
 *
 * A hole is only declared when the expected frame can never arrive any
 * more: either the next frame of the same link shows up (the frames in
 * between were lost on that serial link) or the link itself died. A hole
 * just shortens the capture, it never garbles the stream. */
static void lan_forward_frames(struct gd32lan_context *devc)
{
    struct sr_datafeed_packet packet;
    struct sr_datafeed_analog analog;

    for (;;) {
        uint32_t expected = devc->seq_valid ? (devc->last_seq + 1) : 0;
        uint32_t link = expected & 1u;
        GByteArray *q = link ? devc->queue2 : devc->queue;
        gboolean dead = link ? devc->rx_done2 : devc->rx_done;
        uint32_t seq = 0;
        gboolean have, forward = FALSE;

        g_mutex_lock(&devc->queue_lock);
        have = lan_head_frame(q, &seq);

        /* The expected frame can never arrive when its link is dead; fall
         * back to the other link so a broken cable does not wedge the
         * capture. */
        if (!have && dead) {
            q = link ? devc->queue : devc->queue2;
            have = lan_head_frame(q, &seq);
        }

        if (have) {
            if (!devc->seq_valid || (int32_t)(seq - expected) >= 0) {
                if (devc->seq_valid && seq != expected) {
                    uint32_t lost = seq - expected;

                    devc->frames_lost += lost;
                    sr_warn("%u frame(s) lost before seq %u.",
                            (unsigned int)lost, (unsigned int)seq);
                }

                memcpy(devc->frame_buf,
                       q->data + GD32LA_FRAME_HEADER_BYTES,
                       sizeof(devc->frame_buf));
                g_byte_array_remove_range(q, 0, GD32LA_ANALOG_FRAME_BYTES);
                devc->last_seq = seq;
                devc->seq_valid = TRUE;
                forward = TRUE;
            } else {
                /* a lagging duplicate, drop it and wait for the next */
                g_byte_array_remove_range(q, 0, GD32LA_ANALOG_FRAME_BYTES);
            }
        }
        g_mutex_unlock(&devc->queue_lock);

        if (!forward)
            return;

        analog.probes = devc->sdi->channels;
        analog.num_samples = GD32LA_FRAME_SAMPLES;
        analog.unit_bits = GD32LA_ANALOG_BITS;
        analog.unit_pitch = 0;
        analog.mq = SR_MQ_VOLTAGE;
        analog.unit = SR_UNIT_VOLT;
        analog.mqflags = SR_MQFLAG_AC;
        analog.data = devc->frame_buf;

        packet.status = SR_PKT_OK;
        packet.type = SR_DF_ANALOG;
        packet.payload = &analog;
        ds_data_forward(devc->sdi, &packet);

        devc->sent_samples += GD32LA_FRAME_SAMPLES;
    }
}
/* Receive threads: each pumps one socket into its session visible queue. */
static void lan_rx_pump(struct gd32lan_context *devc, SOCKET sock,
                        GByteArray *queue)
{
    uint8_t buf[4096];

    while (!devc->abort) {
        int n = recv(sock, (char *)buf, (int)sizeof(buf), 0);

        if (n > 0) {
            g_mutex_lock(&devc->queue_lock);
            g_byte_array_append(queue, buf, (guint)n);
            g_mutex_unlock(&devc->queue_lock);
            continue;
        }

        /* 0 = orderly shutdown, < 0 = error or our own closesocket */
        break;
    }
}

static gpointer lan_rx_thread(gpointer data)
{
    struct gd32lan_context *devc = data;

    lan_rx_pump(devc, devc->sock, devc->queue);
    devc->rx_done = TRUE;
    return NULL;
}

static gpointer lan_rx_thread2(gpointer data)
{
    struct gd32lan_context *devc = data;

    lan_rx_pump(devc, devc->sock2, devc->queue2);
    devc->rx_done2 = TRUE;
    return NULL;
}

/* Stop the device stream and give the serial servers time to push the
 * command out before the TCP connections go away. Without this the
 * device keeps transmitting at full rate after DSView stopped reading,
 * because nothing tells it to stop. */
static void lan_send_stop(struct gd32lan_context *devc)
{
    if (devc->sock != INVALID_SOCKET || devc->sock2 != INVALID_SOCKET) {
        lan_send_cmd(devc, GD32LA_CMD_STOP, 0, 0);
        g_usleep(GD32LAN_STOP_LINGER_MS * 1000);
    }
}

/* Timer driven session source: merge the frames of both links and forward
 * them in the session thread. */
static int lan_receive_data(int fd, int revents, const struct sr_dev_inst *sdi)
{
    struct gd32lan_context *devc = sdi->priv;
    gboolean queues_empty;
    gboolean all_dead;

    (void)fd;

    if (revents == -1 && !devc->abort) {
        sr_info("Acquisition aborted by the session.");
        devc->abort = TRUE;
    }

    /* keepalive: prove to the device that the host is still there, it
     * auto-stops after 1.5 s of silence (the PING is ignored by the
     * firmware while a capture is running).
     *
     * The period is tested against the monotonic clock on every invocation
     * instead of counting invocations: the timer behind a source with fd -1
     * can fire far slower than the requested interval, and a counter then
     * lets the device watchdog expire before the first PING is sent. */
    if (!devc->abort && !devc->limit_reached && !devc->ended &&
        (devc->sock != INVALID_SOCKET || devc->sock2 != INVALID_SOCKET)) {
        gint64 now = g_get_monotonic_time();

        if (now - devc->last_ping_us >= GD32LAN_KEEPALIVE_US) {
            devc->last_ping_us = now;
            if (lan_send_cmd(devc, GD32LA_CMD_PING, 0, 0) != SR_OK)
                sr_warn("Keepalive PING failed.");
        }
    }

    /* one link dying mid capture does not end the capture: the other one
     * keeps delivering frames, only with holes in the sequence */
    if (!devc->link_warned && !devc->abort && !devc->limit_reached &&
        !devc->ended && (devc->rx_done || devc->rx_done2)) {
        devc->link_warned = TRUE;
        sr_warn("One serial link died mid capture, continuing on the other "
                "one at the cost of lost frames.");
    }

    if (!devc->abort && !devc->limit_reached)
        lan_forward_frames(devc);

    if (!devc->is_loop && devc->limit_samples > 0 &&
        devc->sent_samples >= devc->limit_samples) {
        sr_info("Sample limit reached (%" PRIu64 " samples).",
                devc->sent_samples);
        devc->limit_reached = TRUE;
    }

    g_mutex_lock(&devc->queue_lock);
    queues_empty = (devc->queue->len == 0 && devc->queue2->len == 0);
    g_mutex_unlock(&devc->queue_lock);

    all_dead = devc->rx_done && devc->rx_done2;

    if (!devc->ended &&
        (devc->abort || devc->limit_reached ||
         (all_dead && queues_empty))) {
        if (all_dead && !devc->abort && !devc->limit_reached)
            sr_err("Links to %s:%s and %s:%s lost.",
                   devc->host, devc->port, devc->host, devc->port2);
        /* tell the device to stop streaming before the links go away,
         * otherwise it transmits forever (serial links have no
         * disconnect state the device could notice) */
        lan_send_stop(devc);
        lan_stop_link(devc);
        lan_finish(devc);
    }

    return TRUE;
}

/* -------------------------------------------------------------------------
 * Link setup / teardown
 * ---------------------------------------------------------------------- */

/* Close both connections and join both receive threads. Idempotent,
 * callable from the session source as well as from the stop callback. */
static void lan_stop_link(struct gd32lan_context *devc)
{
    if (devc->sock != INVALID_SOCKET) {
        shutdown(devc->sock, 2 /* SD_BOTH / SHUT_RDWR */);
        closesocket(devc->sock);
        devc->sock = INVALID_SOCKET;
    }

    if (devc->sock2 != INVALID_SOCKET) {
        shutdown(devc->sock2, 2 /* SD_BOTH / SHUT_RDWR */);
        closesocket(devc->sock2);
        devc->sock2 = INVALID_SOCKET;
    }

    if (devc->rx_thread != NULL) {
        g_thread_join(devc->rx_thread);
        devc->rx_thread = NULL;
    }

    if (devc->rx_thread2 != NULL) {
        g_thread_join(devc->rx_thread2);
        devc->rx_thread2 = NULL;
    }
}

/* Put a freshly connected socket back into blocking mode with no receive
 * timeout for the streaming phase. */
static void lan_set_blocking_stream(SOCKET fd)
{
#ifdef _WIN32
    DWORD tv = 0;

    setsockopt(fd, SOL_SOCKET, SO_RCVTIMEO, (const char *)&tv, sizeof(tv));
#else
    struct timeval tv;

    tv.tv_sec = 0;
    tv.tv_usec = 0;
    setsockopt(fd, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));
#endif
}

static int lan_start_link(struct gd32lan_context *devc)
{
    int ret;

    /* link 0: even frame numbers */
    ret = lan_connect(devc->host, devc->port, &devc->sock);
    if (ret != SR_OK)
        return ret;

    /* link 1: odd frame numbers */
    ret = lan_connect(devc->host, devc->port2, &devc->sock2);
    if (ret != SR_OK) {
        closesocket(devc->sock);
        devc->sock = INVALID_SOCKET;
        return ret;
    }

    devc->rx_done = FALSE;
    devc->rx_done2 = FALSE;

    /* Stop whatever the device was doing and flush both serial servers, so
     * the header of the new stream is the first thing we read on each link. */
    lan_send_cmd(devc, GD32LA_CMD_STOP, 0, 0);
    lan_drain(devc->sock);
    lan_drain(devc->sock2);

    /* The device accepts START on either link and restarts both streams,
     * so both links get the command and answer with the header. */
    if (lan_send_cmd_fd(devc->sock, GD32LA_CMD_START, devc->stream_mode,
                        (uint32_t)devc->samplerate) != SR_OK) {
        ret = SR_ERR;
        goto err;
    }
    if (lan_send_cmd_fd(devc->sock2, GD32LA_CMD_START, devc->stream_mode,
                        (uint32_t)devc->samplerate) != SR_OK) {
        ret = SR_ERR;
        goto err;
    }

    if (lan_read_header(devc->sock, devc->stream_mode) != SR_OK) {
        ret = SR_ERR;
        goto err;
    }
    if (lan_read_header(devc->sock2, devc->stream_mode) != SR_OK) {
        ret = SR_ERR;
        goto err;
    }

    /* back to blocking with no timeout for the streaming phase */
    lan_set_blocking_stream(devc->sock);
    lan_set_blocking_stream(devc->sock2);

    devc->rx_thread = g_thread_new("gd32lan-rx0", lan_rx_thread, devc);
    devc->rx_thread2 = g_thread_new("gd32lan-rx1", lan_rx_thread2, devc);
    if (devc->rx_thread == NULL || devc->rx_thread2 == NULL) {
        sr_err("Failed to start the receive threads.");
        ret = SR_ERR;
        goto err;
    }

    return SR_OK;

err:
    lan_send_stop(devc);
    lan_stop_link(devc);
    return ret;
}

/* -------------------------------------------------------------------------
 * Driver callbacks
 * ---------------------------------------------------------------------- */

static int hw_init(struct sr_context *sr_ctx)
{
    struct gd32lan_drv_context *drvc;

#ifdef _WIN32
    WSADATA wsa;

    if (WSAStartup(MAKEWORD(2, 2), &wsa) != 0) {
        sr_err("WSAStartup failed.");
        return SR_ERR;
    }
#endif

    drvc = g_try_malloc0(sizeof(struct gd32lan_drv_context));
    if (drvc == NULL)
        return SR_ERR_MALLOC;

    drvc->sr_ctx = sr_ctx;
    drvc->sdi = NULL;
    di->priv = drvc;

    return SR_OK;
}

static int hw_cleanup(void)
{
    struct gd32lan_drv_context *drvc = di->priv;

    if (drvc != NULL) {
        g_free(drvc);
        di->priv = NULL;
    }

#ifdef _WIN32
    WSACleanup();
#endif

    return SR_OK;
}

static GSList *hw_scan(GSList *options)
{
    struct gd32lan_drv_context *drvc = di->priv;
    struct sr_dev_inst *sdi;
    struct gd32lan_context *devc;
    GSList *devices = NULL;
    const char *host;
    const char *port;
    const char *port2;

    (void)options;

    if (drvc == NULL)
        return NULL;

    /* One network device per application run. The pointer stays valid
     * because nothing ever detaches a network device; dev_destroy() clears
     * it on shutdown. */
    if (drvc->sdi != NULL)
        return NULL;

    devc = g_try_malloc0(sizeof(struct gd32lan_context));
    if (devc == NULL)
        return NULL;

    host = g_getenv("GD32LA_HOST");
    port = g_getenv("GD32LA_PORT");
    port2 = g_getenv("GD32LA_PORT2");
    if (host == NULL || *host == '\0')
        host = GD32LAN_DEFAULT_HOST;
    if (port == NULL || *port == '\0')
        port = GD32LAN_DEFAULT_PORT;
    if (port2 == NULL || *port2 == '\0')
        port2 = GD32LAN_DEFAULT_PORT2;

    g_strlcpy(devc->host, host, sizeof(devc->host));
    g_strlcpy(devc->port, port, sizeof(devc->port));
    g_strlcpy(devc->port2, port2, sizeof(devc->port2));

    devc->samplerate = GD32LAN_DEFAULT_ASRATE;
    devc->limit_samples = GD32LAN_DEFAULT_ALIMIT;
    devc->channels_enabled = GD32LA_ANALOG_CHANNELS;
    devc->stream_mode = GD32LA_MODE_ANALOG;
    devc->is_loop = FALSE;
    devc->ended = TRUE;
    devc->sock = INVALID_SOCKET;
    devc->sock2 = INVALID_SOCKET;
    devc->queue = g_byte_array_new();
    devc->queue2 = g_byte_array_new();
    g_mutex_init(&devc->queue_lock);

    sdi = sr_dev_inst_new(ANALOG, SR_ST_INACTIVE, "GigaDevice",
                          "GD32F470 16ch DA Simulator (network)", "1.1");
    if (sdi == NULL) {
        g_byte_array_free(devc->queue, TRUE);
        g_byte_array_free(devc->queue2, TRUE);
        g_mutex_clear(&devc->queue_lock);
        g_free(devc);
        return NULL;
    }

    sdi->priv = devc;
    sdi->driver = di;
    sdi->dev_type = DEV_TYPE_SERIAL;
    sdi->conn = NULL;
    devc->sdi = sdi;

    lan_rebuild_channels(sdi);

    drvc->sdi = sdi;

    sr_info("Registered network device, will connect to %s:%s and %s:%s.",
            devc->host, devc->port, devc->host, devc->port2);

    devices = g_slist_append(devices, sdi);

    return devices;
}

static const GSList *hw_dev_mode_list(const struct sr_dev_inst *sdi)
{
    GSList *l = NULL;

    (void)sdi;

    /* Analog data acquisition only: 16 channels, 16 bit. */
    l = g_slist_append(l, (gpointer)&sr_mode_list[1]);   /* ANALOG */

    return l;
}

/* Describe the analog probe defaults: 16 bit samples centred on 32768,
 * 1 V/div, AC coupling, -5 V .. +5 V display range. */
static void lan_setup_analog_probe(struct sr_channel *probe)
{
    probe->bits = GD32LA_ANALOG_BITS;
    probe->coupling = SR_AC_COUPLING;
    probe->vdiv = SR_V(1);
    probe->vfactor = SR_mV(1);
    probe->hw_offset = GD32LA_ANALOG_MID;
    probe->offset = GD32LA_ANALOG_MID;
    /* The trigger threshold register is 8 bit wide and triggers are not
     * implemented; keep it at mid scale so the dialog shows a sane value. */
    probe->trig_value = 0x80;
    probe->map_default = TRUE;
    probe->map_unit = map_units[0];
    probe->map_min = -5.0;
    probe->map_max = 5.0;
}

/* Analog data acquisition only: the 16 analog probes. */
static void lan_rebuild_channels(struct sr_dev_inst *sdi)
{
    struct gd32lan_context *devc = sdi->priv;
    guint16 j;

    sr_dev_probes_free(sdi);

    for (j = 0; j < GD32LA_ANALOG_CHANNELS; j++) {
        struct sr_channel *probe;

        probe = sr_channel_new(j, SR_CHANNEL_ANALOG, TRUE,
                               analog_probe_names[j]);
        if (probe == NULL) {
            sr_err("Failed to create analog channel %u.", (unsigned int)j);
            return;
        }

        lan_setup_analog_probe(probe);
        sdi->channels = g_slist_append(sdi->channels, probe);
    }

    if (devc != NULL)
        devc->channels_enabled = GD32LA_ANALOG_CHANNELS;
}

static int hw_dev_open(struct sr_dev_inst *sdi)
{
    if (sdi->status == SR_ST_ACTIVE)
        return SR_OK;

    /* Nothing to hold on to between captures: the TCP connection is opened
     * per acquisition, idle serial servers tend to drop silent clients. */
    sdi->status = SR_ST_ACTIVE;
    sr_info("Device opened (will connect to the serial server on start).");

    return SR_OK;
}

static int hw_dev_close(struct sr_dev_inst *sdi)
{
    struct gd32lan_context *devc = sdi->priv;

    if (devc != NULL) {
        devc->abort = TRUE;
        lan_send_stop(devc);
        lan_stop_link(devc);
        devc->ended = TRUE;
    }

    sdi->status = SR_ST_INACTIVE;
    sr_info("Device closed.");

    return SR_OK;
}

static int dev_destroy(struct sr_dev_inst *sdi)
{
    struct gd32lan_drv_context *drvc = di->priv;
    struct gd32lan_context *devc = sdi->priv;

    hw_dev_close(sdi);

    if (devc != NULL) {
        if (devc->queue != NULL)
            g_byte_array_free(devc->queue, TRUE);
        if (devc->queue2 != NULL)
            g_byte_array_free(devc->queue2, TRUE);
        g_mutex_clear(&devc->queue_lock);
    }

    if (drvc != NULL && drvc->sdi == sdi)
        drvc->sdi = NULL;

    sr_dev_inst_free(sdi);

    return SR_OK;
}

/* The device streams analog frames only; check that at least one analog
 * channel is enabled. */
static int lan_pick_stream_mode(struct sr_dev_inst *sdi)
{
    struct gd32lan_context *devc = sdi->priv;

    if (!lan_analog_in_use(sdi)) {
        sr_err("No channel enabled.");
        return SR_ERR_ARG;
    }

    devc->stream_mode = GD32LA_MODE_ANALOG;
    return SR_OK;
}

static int hw_dev_acquisition_start(struct sr_dev_inst *sdi, void *cb_data)
{
    struct gd32lan_context *devc = sdi->priv;
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
    devc->last_ping_us = 0;
    devc->last_seq = 0;
    devc->seq_valid = FALSE;
    devc->link_warned = FALSE;
    devc->frames_lost = 0;

    /* Drop whatever the receive threads collected after the last poll of
     * the previous capture. Nobody consumed those bytes (the session source
     * is removed at the end of a capture), and their length is arbitrary,
     * so letting them head the next stream breaks the frame alignment. */
    g_mutex_lock(&devc->queue_lock);
    g_byte_array_set_size(devc->queue, 0);
    g_byte_array_set_size(devc->queue2, 0);
    g_mutex_unlock(&devc->queue_lock);

    ret = lan_pick_stream_mode(sdi);
    if (ret != SR_OK)
        return ret;

    /* Clamp to what the two 1500000 baud serial links can carry. */
    if (devc->samplerate > GD32LA_UART_MAX_ANALOG_RATE) {
        sr_warn("The analog stream tops out at %u Hz over the serial links, "
                "clamping the samplerate.",
                (unsigned int)GD32LA_UART_MAX_ANALOG_RATE);
        devc->samplerate = GD32LA_UART_MAX_ANALOG_RATE;
    }
    if (devc->samplerate < GD32LAN_MIN_SAMPLERATE)
        devc->samplerate = GD32LAN_DEFAULT_ASRATE;
    if (devc->limit_samples == 0)
        devc->limit_samples = GD32LAN_DEFAULT_ALIMIT;

    sr_info("Starting acquisition: %" PRIu64 " Hz, limit %" PRIu64
            " samples, %s mode on two links (%s:%s and %s:%s).",
            devc->samplerate, devc->limit_samples,
            lan_mode_name(devc->stream_mode),
            devc->host, devc->port, devc->host, devc->port2);

    ret = lan_start_link(devc);
    if (ret != SR_OK) {
        ds_set_last_error(SR_ERR_DEVICE_USB_IO_ERROR);
        return ret;
    }

    /* The device watchdog restarts with the START command, so the keepalive
     * clock starts here and not when the first poll happens. */
    devc->last_ping_us = g_get_monotonic_time();

    sr_session_source_add(-1, 0, GD32LAN_POLL_INTERVAL_MS,
                          lan_receive_data, sdi);

    return SR_OK;
}

static int hw_dev_acquisition_stop(const struct sr_dev_inst *sdi,
                                   void *cb_data)
{
    struct gd32lan_context *devc = sdi->priv;

    (void)cb_data;

    if (devc == NULL)
        return SR_OK;

    sr_info("Stopping acquisition.");

    devc->abort = TRUE;

    lan_send_stop(devc);
    lan_stop_link(devc);
    lan_finish(devc);

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
    struct gd32lan_context *devc;
    guint logic_probes;
    guint analog_probes;

    (void)cg;

    if (sdi == NULL || sdi->priv == NULL)
        return SR_ERR_ARG;

    devc = sdi->priv;
    lan_count_probes(sdi, &logic_probes, &analog_probes);

    switch (id) {
    case SR_CONF_SAMPLERATE:
        *data = g_variant_new_uint64(devc->samplerate);
        break;
    case SR_CONF_LIMIT_SAMPLES:
        *data = g_variant_new_uint64(devc->limit_samples);
        break;
    case SR_CONF_HW_DEPTH:
        *data = g_variant_new_uint64(GD32LAN_MAX_SAMPLES);
        break;
    case SR_CONF_DEVICE_MODE:
        *data = g_variant_new_int16(sdi->mode);
        break;
    case SR_CONF_UNIT_BITS:
        *data = g_variant_new_byte(GD32LA_ANALOG_BITS);
        break;
    case SR_CONF_MAX_HEIGHT_VALUE:
        /* Row height is (value + 1) * 20 px; the analog waveforms need
         * room to breathe. */
        *data = g_variant_new_byte(1);
        break;
    case SR_CONF_VLD_CH_NUM:
        *data = g_variant_new_int16((int16_t)(logic_probes + analog_probes));
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
    struct gd32lan_context *devc;
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
        if (mode != ANALOG) {
            sr_err("Unsupported work mode %d, the device is analog only.",
                   mode);
            return SR_ERR_ARG;
        }
        if (sdi->mode != mode) {
            sdi->mode = mode;
            lan_rebuild_channels(sdi);
            devc->samplerate = GD32LAN_DEFAULT_ASRATE;
            devc->limit_samples = GD32LAN_DEFAULT_ALIMIT;

            sr_info("Work mode set to analog.");
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

    (void)cg;

    if (sdi == NULL)
        return SR_ERR_ARG;

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
        gvar = g_variant_new_from_data(G_VARIANT_TYPE("at"),
                                       analog_samplerates,
                                       ARRAY_SIZE(analog_samplerates) *
                                       sizeof(uint64_t),
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

SR_PRIV struct sr_dev_driver gd32lan_driver_info = {
    .name = "gd32lan",
    .longname = "GD32F470 16ch DA Simulator (network)",
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
