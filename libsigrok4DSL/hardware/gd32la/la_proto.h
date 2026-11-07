/*!
    \file    la_proto.h
    \brief   GD32 logic analyzer simulator - USB protocol shared definitions

    This header is shared by the GD32F470 firmware and the DSView driver
    (libsigrok4DSL/hardware/gd32la). Keep the two copies identical.

    Protocol summary
    ----------------
    USB: vendor specific class (0xFF), one interface, two bulk endpoints.

      EP 0x01 OUT  64 B   host -> device, 8 byte fixed command records
      EP 0x81 IN   64 B   device -> host, sample stream

    Command record (8 bytes, little endian):

      offset 0  u8   command id   (GD32LA_CMD_xxx)
      offset 1  u8   argument     (mode for GD32LA_CMD_START)
      offset 2  u16  reserved (0)
      offset 4  u32  value        (samplerate in Hz for GD32LA_CMD_START)

    Every GD32LA_CMD_START is answered with one header packet of exactly
    GD32LA_HDR_LEN (20) bytes. Because 20 < 64 the packet is a short packet
    and terminates the bulk transfer on the host side, so the driver can read
    it with a small buffer before switching to streaming reads. All packets
    that follow the header carry raw sample data only.

    The device has three work modes, selected by the argument of
    GD32LA_CMD_START:

      GD32LA_MODE_LOGIC   2 digital channels, LA_CROSS_DATA layout
      GD32LA_MODE_ANALOG  2 analog channels, 8 bit interleaved samples
      GD32LA_MODE_MIXED   both of the above in one stream, so the host can
                          display digital and analog traces side by side

    Logic sample layout (LA_CROSS_DATA, same as DreamSourceLab devices)
    -----------------------------------------------------------------
    The stream is a sequence of 8 byte blocks, one block per enabled channel,
    channels rotating in ascending order:

      [ch0 byte 0..7][ch1 byte 0..7][ch0 byte 8..15][ch1 byte 8..15] ...

    Inside a block bit 0 is the oldest sample, bit 7 the newest, so one byte
    carries 8 consecutive samples of that channel. One sample group is
    GD32LA_GROUP_BYTES = 16 bytes, which is 8 bytes * 8 samples = 64 samples
    per channel.

    Analog sample layout
    --------------------
    Plain interleaved bytes, one byte per channel per sample:

      [ch0 sample n][ch1 sample n][ch0 sample n+1][ch1 sample n+1] ...

    Values are unsigned 8 bit centred on GD32LA_ANALOG_MID (128), which is
    what DSView's analog view expects from a 1 V/div, AC coupled probe.

    Mixed sample layout
    -------------------
    One mixed group covers GD32LA_SAMPLES_PER_BYTE (8) samples and is
    GD32LA_MIXED_GROUP_BYTES (18) bytes long:

      byte 0                 D0, 8 samples, bit 0 oldest
      byte 1                 D1, 8 samples, bit 0 oldest
      byte 2 + 2*n + 0       A0 sample n
      byte 2 + 2*n + 1       A1 sample n

    Digital and analog stay in lockstep, so sample n of every channel of a
    group belongs to the same instant on the DSView time axis.
*/

#ifndef LA_PROTO_H
#define LA_PROTO_H

#include <stdint.h>

/* USB identifiers. 0x28E9 is GigaDevice, 0x5701 is picked freely for this
 * simulator. Change both here and in the WinUSB inf file if they clash. */
#define GD32LA_VID                     0x28E9U
#define GD32LA_PID                     0x5701U

/* Endpoint addresses */
#define GD32LA_EP_OUT                  0x01U
#define GD32LA_EP_IN                   0x81U

/* Endpoint max packet size (full speed bulk) */
#define GD32LA_PACKET_SIZE             64U

/* Logic channel configuration */
#define GD32LA_CHANNELS                2U
#define GD32LA_SAMPLES_PER_BYTE        8U
#define GD32LA_GROUP_BYTES             (GD32LA_CHANNELS * 8U)   /* 16 */

/* Analog channel configuration */
#define GD32LA_ANALOG_CHANNELS         2U
#define GD32LA_ANALOG_BITS             8U
#define GD32LA_ANALOG_MID              128U
#define GD32LA_ANALOG_GROUP_BYTES      (GD32LA_ANALOG_CHANNELS * 1U)

/* Mixed group: 8 samples of each digital channel (one byte per channel) plus
 * 8 interleaved analog sample pairs. */
#define GD32LA_MIXED_GROUP_BYTES       (GD32LA_CHANNELS + \
                                        GD32LA_ANALOG_GROUP_BYTES * \
                                        GD32LA_SAMPLES_PER_BYTE)   /* 18 */

/* Work modes, passed in the argument byte of GD32LA_CMD_START */
#define GD32LA_MODE_LOGIC              0U
#define GD32LA_MODE_ANALOG             1U
#define GD32LA_MODE_MIXED              2U

/* Command ids */
#define GD32LA_CMD_START               0x01U   /* arg = mode, value = samplerate */
#define GD32LA_CMD_STOP                0x02U
#define GD32LA_CMD_PING                0x03U
#define GD32LA_CMD_SET_PHASE           0x04U   /* arg: 0 = ch1 90 deg, 1 = ch1 in phase */

/* Header packet magic */
#define GD32LA_HDR_MAGIC0              'G'
#define GD32LA_HDR_MAGIC1              'D'
#define GD32LA_HDR_MAGIC2              'L'
#define GD32LA_HDR_MAGIC3              'A'
#define GD32LA_HDR_VERSION             3U
#define GD32LA_HDR_LEN                 20U

#if defined(__GNUC__) || defined(__clang__)
#define GD32LA_PACKED  __attribute__((packed))
#else
#define GD32LA_PACKED
#endif

/* 8 byte command written by the host to GD32LA_EP_OUT */
typedef struct GD32LA_PACKED {
    uint8_t  cmd;
    uint8_t  arg;
    uint16_t reserved;
    uint32_t value;
} gd32la_cmd_t;

/* 20 byte answer to GD32LA_CMD_START */
typedef struct GD32LA_PACKED {
    uint8_t  magic[4];       /* "GDLA" */
    uint8_t  version;        /* GD32LA_HDR_VERSION */
    uint8_t  channels;       /* GD32LA_CHANNELS, logic channels */
    uint16_t packet_size;    /* GD32LA_PACKET_SIZE */
    uint32_t samplerate;     /* Hz, echo of the START command */
    uint32_t flags;          /* bit0: ch1 phase shifted by 90 degrees */
    uint8_t  mode;           /* GD32LA_MODE_LOGIC / _ANALOG / _MIXED */
    uint8_t  analog_channels;/* GD32LA_ANALOG_CHANNELS */
    uint8_t  analog_bits;    /* GD32LA_ANALOG_BITS */
    uint8_t  reserved;
} gd32la_hdr_t;

#if defined(__STDC_VERSION__) && (__STDC_VERSION__ >= 201112L)
_Static_assert(sizeof(gd32la_cmd_t) == 8, "gd32la_cmd_t must be 8 bytes");
_Static_assert(sizeof(gd32la_hdr_t) == GD32LA_HDR_LEN, "gd32la_hdr_t must be 20 bytes");
#endif

#endif /* LA_PROTO_H */
