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

    The device streams analog samples only. The mode byte of
    GD32LA_CMD_START selects the stream format:

      GD32LA_MODE_ANALOG  16 analog channels, 16 bit interleaved samples

    GD32LA_MODE_LOGIC and GD32LA_MODE_MIXED are legacy single link formats
    from the digital era and are no longer produced.

    Logic sample layout (legacy, unused)
    ------------------------------------
    The stream used to be a sequence of 8 byte blocks, one block per enabled
    channel, channels rotating in ascending order:

      [ch0 byte 0..7][ch1 byte 0..7][ch0 byte 8..15][ch1 byte 8..15] ...

    Inside a block bit 0 is the oldest sample, bit 7 the newest, so one byte
    carries 8 consecutive samples of that channel. One sample group is
    GD32LA_GROUP_BYTES = 16 bytes, which is 8 bytes * 8 samples = 64 samples
    per channel.

    Analog sample layout
    --------------------
    Plain interleaved samples, little endian, GD32LA_ANALOG_BYTES (2) bytes per
    channel per sample, low byte first:

      [ch0 sample n lo][ch0 sample n hi][ch1 sample n lo][ch1 sample n hi] ...

    Values are unsigned 16 bit centred on GD32LA_ANALOG_MID (32768), which is
    what DSView's analog view expects from an AC coupled probe: it reads
    (unit_bits + 7) / 8 bytes per sample with the low byte first and maps
    0 .. 2^bits - 1 onto the full channel height.

    Analog frame layout (dual UART transport)
    ------------------------------------------
    The transport uses TWO serial links (USART2 on PB10/PB11 and USART1 on
    PA2/PA3, both 1500000 8N1) because one link cannot carry 16 channels.
    Frames alternate between the links: frame 0 goes out on link 0, frame 1
    on link 1, frame 2 on link 0 again, and so on. Each link therefore runs
    at half the sample data rate.

    One frame is GD32LA_ANALOG_FRAME_BYTES (2054) bytes and holds
    GD32LA_FRAME_SAMPLES (64) samples of every channel:

      byte  0          0xAA, frame magic, byte 1 is 0x55
      byte  2 ..  5    frame sequence number, uint32 little endian, starts
                       at 0 on every START and wraps at 2^32
      byte  6 .. 2053  64 samples * 16 channels * 2 bytes, interleaved as
                       above

    The magic lets the receiver resynchronise after a lost byte: when the
    first two bytes of a frame do not match, it drops one byte and looks
    again. The sequence numbers let the host merge the two links back into
    one ordered stream and tell lost frames apart (a hole in the sequence
    shortens the capture instead of garbling it).
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
#define GD32LA_BYTES_PER_CHANNEL       8U

/* One cross block: 8 bytes of every channel, that is 64 samples per channel.
 * This is the granularity DSView's LogicSnapshot works with. */
#define GD32LA_GROUP_SAMPLES           (GD32LA_BYTES_PER_CHANNEL * \
                                        GD32LA_SAMPLES_PER_BYTE)          /* 64 */
#define GD32LA_GROUP_BYTES             (GD32LA_CHANNELS * \
                                        GD32LA_BYTES_PER_CHANNEL)         /* 16 */

/* Analog channel configuration. 16 channels, 16 bit unsigned samples,
 * little endian. */
#define GD32LA_ANALOG_CHANNELS         16U
#define GD32LA_ANALOG_BITS             16U
#define GD32LA_ANALOG_BYTES            ((GD32LA_ANALOG_BITS + 7U) / 8U)   /* 2 */
#define GD32LA_ANALOG_MID              32768U
#define GD32LA_ANALOG_GROUP_BYTES      (GD32LA_ANALOG_CHANNELS * \
                                        GD32LA_ANALOG_BYTES)             /* 32 */

/* One frame on the wire: 6 byte frame header + 64 samples of all channels.
 * The frames alternate between the two UART links, see the header comment. */
#define GD32LA_FRAME_MAGIC0            0xAAU
#define GD32LA_FRAME_MAGIC1            0x55U
#define GD32LA_FRAME_HEADER_BYTES      6U
#define GD32LA_FRAME_SAMPLES           64U
#define GD32LA_ANALOG_FRAME_BYTES      (GD32LA_FRAME_HEADER_BYTES + \
                                        GD32LA_ANALOG_GROUP_BYTES * \
                                        GD32LA_GROUP_SAMPLES)             /* 2054 */

/* Legacy mixed group of the digital era: one complete logic cross block
 * followed by the analog samples of the same 64 instants. Unused, the
 * analog stream is framed as above. */
#define GD32LA_MIXED_GROUP_BYTES       (GD32LA_GROUP_BYTES + \
                                        GD32LA_ANALOG_GROUP_BYTES * \
                                        GD32LA_GROUP_SAMPLES)

/* Work modes, passed in the argument byte of GD32LA_CMD_START */
#define GD32LA_MODE_LOGIC              0U
#define GD32LA_MODE_ANALOG             1U
#define GD32LA_MODE_MIXED              2U

/* Samplerate ceilings of the dual UART transport (2 x 1500000 8N1 =
 * 300 kB/s gross, keep roughly 20% head room for the serial servers and
 * their buffers):
 *   analog rate * 32 B/s -> 6400 Sa/s = 204.8 kB/s (68%)
 * The logic / mixed ceilings belong to the legacy single link formats. */
#define GD32LA_UART_MAX_LOGIC_RATE     128000U
#define GD32LA_UART_MAX_ANALOG_RATE    5000U
#define GD32LA_UART_MAX_MIXED_RATE     6400U

/* Command ids */
#define GD32LA_CMD_START               0x01U   /* arg = mode, value = samplerate */
#define GD32LA_CMD_STOP                0x02U
#define GD32LA_CMD_PING                0x03U   /* keepalive: while a capture runs
                                                * the host PINGs every 500 ms;
                                                * the device stops streaming after
                                                * 1.5 s without any RX byte */
#define GD32LA_CMD_SET_PHASE           0x04U   /* arg: 0 = ch1 90 deg, 1 = ch1 in phase */

/* Header packet magic */
#define GD32LA_HDR_MAGIC0              'G'
#define GD32LA_HDR_MAGIC1              'D'
#define GD32LA_HDR_MAGIC2              'L'
#define GD32LA_HDR_MAGIC3              'A'
/* 6: 16 analog channels, no digital channels, frames carry a magic and a
 *    sequence number and alternate between two UART links */
#define GD32LA_HDR_VERSION             6U
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
    uint8_t  channels;       /* digital channels, always 0 in v6 */
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
