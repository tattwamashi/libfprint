/*
 * FocalTech FT9366 (Realtek RTS5811 USB bridge) driver for libfprint
 *
 * Copyright (C) 2026 v8v88v8v88
 *
 * This library is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public
 * License as published by the Free Software Foundation; either
 * version 2.1 of the License, or (at your option) any later version.
 *
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public
 * License along with this library; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA
 */

#define FP_COMPONENT "ft9366"
#include "fpi-log.h"

#include "drivers_api.h"
#include "fpi-byte-utils.h"
#include "ft9366-match.h"

#include <math.h>

/*
 * Wire protocol, worked out from usbmon captures (notes:
 * https://github.com/V8V88V8V88/ft9366-rek/blob/main/re/PROTOCOL.md):
 *
 *   bulk OUT 0x01 / bulk IN 0x82
 *   byte 0 = opcode, byte 1 = ~opcode, multi-byte fields big-endian
 *
 *   04 fb AAAA NNNN            read  N x u16 from register AAAA
 *   05 fa AAAA NNNN VVVV...    write N x u16 to   register AAAA
 *   06 f9 AAAA NNNN            burst read N x u16 (image frame)
 *   08 f7 RR 00 00             read  8-bit register RR
 *   09 f6 RR VV                write 8-bit register RR
 *   c0 3f 00 / c2 3d 00 / c4 3b 00    sensor: idle / finger detect / capture
 *   5a a5 00 ... a5 5a 00      bracket around sensor status reads (r80)
 *
 * Interrupts: the bridge answers FT9366_IRQ_PENDING on 9a8b once the sensor
 * raised its IRQ line; the reason is in 9a82 and is acked by writing it to 9a84.
 */

#define FT9366_EP_OUT (0x01 | FPI_USB_ENDPOINT_OUT)
#define FT9366_EP_IN  (0x02 | FPI_USB_ENDPOINT_IN)


#define FT9366_REG_FRAME      0x9a05  /* image burst read source */
#define FT9366_REG_IRQ_FLAGS  0x9a82  /* why the IRQ fired */
#define FT9366_REG_IRQ_ACK    0x9a84  /* write the flags back to clear */
#define FT9366_REG_IRQ_PIN    0x9a8b  /* reads FT9366_IRQ_PENDING when IRQ is up */
#define FT9366_REG_FDT_BASE   0x80b0  /* 4 x u16 finger-detect baseline */
#define FT9366_REG_FDT_VALUE  0x80b8  /* 4 x u16 finger-detect measurement */
#define FT9366_REG_DAC        0x9801  /* 0xfc80 | dac */
#define FT9366_REG_FDT_MODE   0x9881

#define FT9366_IRQ_PENDING    0x9362
#define FT9366_IRQ_DOWN       0x0002
#define FT9366_IRQ_FDT_DONE   0x0008
#define FT9366_IRQ_BASE_BAD   0x0010  /* finger-detect baseline is off (taken with a finger on) */
#define FT9366_IRQ_FRAME      0x0020

#define FT9366_FDT_MODE_DOWN  0x0f8d
#define FT9366_FDT_MODE_UP    0x0f0c
#define FT9366_FDT_MODE_MANUAL 0x0f0e

/* values the vendor driver settled on for this sensor */
#define FT9366_FDT_DAC        0x1a
#define FT9366_IMG_DAC        0x3b
#define FT9366_FDT_BASE_DROP  31   /* baseline = empty-sensor measurement - this */
#define FT9366_MAX_REINITS    3    /* sensor recalibrations per touch for a bad baseline */
#define FT9366_FDT_STABLE     8    /* measurement noise: readings this close are "unchanged" */
#define FT9366_FDT_TOUCH_DROP 50   /* a zone this far below baseline is touched */

#define FT9366_TIMEOUT        1000
#define FT9366_POLL_MS        20
#define FT9366_HEAL_POLLS     10   /* measure while idle every ~0.3 s: also the software finger-down fallback */

/* matcher */
#define FT9366_SETTLE_MS       40   /* finger-down IRQ fires on first contact: let it land */
#define FT9366_MAX_FRAMES      4    /* frames per touch; the best-covered one is kept */
#define FT9366_GOOD_COVERAGE   0.9  /* stop taking frames once one covers this much */
#define FT9366_FRAME_GAP_MS    25
#define FT9366_ENROLL_STAGES   20   /* each touch sees ~3x4 mm: more touches, more finger */
#define FT9366_DUP_OVERLAP     0.95 /* an enroll touch this covered by earlier ones... */
#define FT9366_DUP_NCC         0.9  /* ...and this well aligned adds nothing new */
#define FT9366_MAX_DUPS        1    /* then take it anyway rather than loop */
#define FT9366_MIN_COVERAGE    0.4  /* share of the sensor a usable touch covers */
#define FT9366_PRINT_VERSION   2

/* ---- op tables ---------------------------------------------------------- */

typedef enum {
  OP_WR16,     /* reg = val */
  OP_WR8,
  OP_RD16,     /* read val x u16 */
  OP_RD8,
  OP_BURST,    /* read val x u16 */
  OP_CMD,      /* 3-byte command, reg = opcode */
  OP_RAW,      /* raw bridge command, reply val bytes */
  OP_DAC,      /* WR16 DAC = 0xfc80 | val */
  OP_FDT_BASE, /* WR16 FDT_BASE = self->fdt_base */
  OP_BRIDGE_CFG, /* 4b 00 ...: arm the bridge's own finger detect */
} Ft9366OpKind;

typedef struct
{
  Ft9366OpKind  kind;
  guint16       reg;
  guint16       val;
  const guint8 *raw;
  guint8        raw_len;
} Ft9366Op;

#define W16(r, v)   { OP_WR16, r, v, NULL, 0 }
#define W8(r, v)    { OP_WR8, r, v, NULL, 0 }
#define R16(r, n)   { OP_RD16, r, n, NULL, 0 }
#define R8(r)       { OP_RD8, r, 1, NULL, 0 }
#define BURST(r, n) { OP_BURST, r, n, NULL, 0 }
#define CMD(c)      { OP_CMD, c, 0, NULL, 0 }
#define DAC(d)      { OP_DAC, 0, d, NULL, 0 }
#define FDT_BASE()  { OP_FDT_BASE, FT9366_REG_FDT_BASE, 0, NULL, 0 }
#define BRIDGE_CFG() { OP_BRIDGE_CFG, 0, 2, NULL, 0 }
#define RAW(b, n)   { OP_RAW, 0, n, b, sizeof (b) }

static const guint8 bridge_status[] = { 0x4c, 0x5a, 0x01, 0x00 };
static const guint8 bridge_start[] = { 0x44, 0x80, 0x00, 0x00 };

/* bridge handshake + sensor reset */
static const Ft9366Op seq_hello[] = {
  RAW (bridge_status, 4),
  RAW (bridge_start, 0),
  RAW (bridge_status, 4),
  W8 (0xf1, 0x03), W8 (0xf4, 0xc0), W8 (0xf4, 0xc1), W8 (0xf4, 0xc0), W8 (0xf4, 0xc0),
  R8 (0xf3),
  W8 (0xf1, 0x13), W8 (0xf4, 0xc0), W8 (0xf4, 0xc1), W8 (0xf4, 0xc0), W8 (0xf4, 0xc0),
  R8 (0xf3),
  R8 (0x9b),
  W16 (FT9366_REG_IRQ_ACK, 0xffff),
  CMD (0x5a), R8 (0x80), CMD (0xa5),
};

/* analog front end: image capture configuration */
static const Ft9366Op seq_afe_image[] = {
  DAC (FT9366_IMG_DAC),
  W16 (0x9800, 0x4ffe), W16 (0x9804, 0x27ca),
  W16 (0x9806, 0x04bb), W16 (0x980a, 0x0483), W16 (0x980b, 0x0208),
  W16 (0x9807, 0x18e1), W16 (0x9887, 0x0002),
};

/* software finger-down: the chip is still in finger-detect mode after the
 * manual measurement, put it back to idle like the init path does */
static const Ft9366Op seq_idle_afe_image[] = {
  CMD (0x5a), R8 (0x80), CMD (0xc0), CMD (0xa5),
  DAC (FT9366_IMG_DAC),
  W16 (0x9800, 0x4ffe), W16 (0x9804, 0x27ca),
  W16 (0x9806, 0x04bb), W16 (0x980a, 0x0483), W16 (0x980b, 0x0208),
  W16 (0x9807, 0x18e1), W16 (0x9887, 0x0002),
};

/* one-time chip setup, after seq_afe_image */
static const Ft9366Op seq_setup[] = {
  W16 (0x9805, 0x0000), W16 (0x9811, 0x01fe),
  W16 (0x9a83, 0x0020), W16 (0x9a83, 0x0060),
  W8 (0x8e, 0xf4), W8 (0x90, 0x00), W8 (0x91, 0x07), W8 (0x92, 0xd0), W8 (0x90, 0x01),
  DAC (FT9366_FDT_DAC), W16 (0x980c, 0x0000),
  W16 (FT9366_REG_FDT_MODE, 0x0f0c),
  W16 (0x9800, 0x07fe), W16 (0x9804, 0x27c8), W16 (0x9807, 0x1671),
  W16 (0x9808, 0x0801), W16 (0x9887, 0x0005),
  W16 (0x9806, 0x09bb), W16 (0x980a, 0x0987), W16 (0x980b, 0x0491),
  W16 (0x9805, 0x0000), W16 (0x980d, 0x0384), W16 (0x9888, 0x0000),
  W16 (0x9880, 0x2d32), W16 (FT9366_REG_FDT_MODE, 0x0f8e), W16 (0x9884, 0x0003),
  W16 (0x9a8a, 0x00ff), W16 (0x9a83, 0x0068),
  W16 (0x980c, 0x0600),
};

/* start one finger-detect measurement, FDT_DONE follows */
static const Ft9366Op seq_fdt_measure[] = {
  DAC (FT9366_FDT_DAC),
  CMD (0xc2), W16 (0x9885, 0x0001),
};

/* manual measurement to confirm the finger really left (vendor does the same) */
static const Ft9366Op seq_up_measure[] = {
  W16 (FT9366_REG_FDT_MODE, FT9366_FDT_MODE_MANUAL),
  DAC (FT9366_FDT_DAC),
  CMD (0xc2), W16 (0x9885, 0x0001),
};

static const Ft9366Op seq_fdt_values[] = {
  W16 (FT9366_REG_IRQ_ACK, FT9366_IRQ_FDT_DONE),
  R16 (FT9366_REG_FDT_VALUE, 4),
};

/* back to idle */
static const Ft9366Op seq_idle[] = {
  CMD (0x5a), R8 (0x80), CMD (0xc0), CMD (0xa5),
};

/* capture one frame (after seq_afe_image) */
static const Ft9366Op seq_frame[] = {
  CMD (0xc4), R8 (0x80),
  W16 (0x9800, 0x4fff),
  BURST (FT9366_REG_FRAME, FT9366_FRAME_PIXELS),
};

static const Ft9366Op seq_frame_done[] = {
  W16 (FT9366_REG_IRQ_ACK, FT9366_IRQ_FRAME),
  CMD (0x5a), R8 (0x80), CMD (0xc0), CMD (0xa5),
};

/* finger-detect configuration + baseline */
static const Ft9366Op seq_afe_fdt[] = {
  DAC (FT9366_FDT_DAC),
  W16 (0x9800, 0x07fe), W16 (0x9804, 0x27c8), W16 (0x9807, 0x1671),
  W16 (0x9808, 0x0801), W16 (0x9887, 0x0005),
  W16 (0x9806, 0x09bb), W16 (0x980a, 0x0987), W16 (0x980b, 0x0491),
  W8 (0x9a, 0x5a), FDT_BASE (), W8 (0x9a, 0x00),
};

/* sent once, at the end of activation: the bridge keeps these settings */
static const Ft9366Op seq_bridge_cfg[] = {
  W16 (FT9366_REG_FDT_MODE, FT9366_FDT_MODE_DOWN),
  W16 (FT9366_REG_IRQ_ACK, 0x002f),
  CMD (0xc2),
  BRIDGE_CFG (),
  RAW (bridge_status, 4),
};

/* down: IRQ with FT9366_IRQ_DOWN in 9a82. up: the IRQ itself means finger left */
static const Ft9366Op seq_arm_down[] = {
  W16 (FT9366_REG_FDT_MODE, FT9366_FDT_MODE_DOWN),
  W16 (FT9366_REG_IRQ_ACK, 0x002f),
  CMD (0xc2),
  RAW (bridge_status, 4),
};

static const Ft9366Op seq_arm_up[] = {
  W16 (FT9366_REG_FDT_MODE, FT9366_FDT_MODE_UP),
  W16 (FT9366_REG_IRQ_ACK, 0x002f),
  CMD (0xc2),
  RAW (bridge_status, 4),
};

static const Ft9366Op seq_poll_irq[] = {
  R16 (FT9366_REG_IRQ_PIN, 1),
};

static const Ft9366Op seq_read_irq[] = {
  R16 (FT9366_REG_IRQ_FLAGS, 1),
};

static const Ft9366Op seq_ack_irq[] = {
  W16 (FT9366_REG_IRQ_ACK, 0x002f),
};

/* ---- device ------------------------------------------------------------- */

typedef enum {
  OP_NONE,
  OP_ENROLL,
  OP_VERIFY,
  OP_IDENTIFY,
} Ft9366Action;

struct _FpiDeviceFt9366
{
  FpDevice        parent;

  FpiSsm         *ssm;
  Ft9366Action    action;
  gboolean        wait_up;
  guint           polls;           /* polls since the last baseline check */
  gboolean        have_base;       /* fdt_base / background are valid */
  gboolean        finger_at_init;  /* the last init measured a touched sensor */
  gboolean        sw_down;         /* finger-down came from a measurement, not the chip IRQ */

  /* op runner */
  const Ft9366Op *seq;
  gsize           seq_len;
  gsize           seq_pos;
  guint8          reply[FT9366_FRAME_PIXELS * 2];
  gsize           reply_len;

  guint16         fdt_base[4];
  gboolean        base_suspect;    /* chip flagged the baseline (IRQ_BASE_BAD) */
  gboolean        reinit_pending;  /* capture ended to recalibrate the sensor */
  guint           reinits;
  gboolean        have_last_fdt;
  guint16         last_fdt[4];     /* previous manual measurement */
  guint16         background[FT9366_FRAME_PIXELS];
  guint16         frame[FT9366_FRAME_PIXELS];

  /* best of the frames taken for the current touch */
  guint16         best_frame[FT9366_FRAME_PIXELS];
  gdouble         best_coverage;
  guint           frames_taken;

  /* last capture after processing: 1..254 = pixel, FT9366_PX_NONE = no finger */
  guint8          image[FT9366_FRAME_PIXELS];
  guint8          dead[FT9366_FRAME_PIXELS];  /* from the background: always ~0 */

  /* enroll */
  GPtrArray      *enroll_images;
  guint           enroll_dups;     /* consecutive near-duplicate touches */
};
G_DECLARE_FINAL_TYPE (FpiDeviceFt9366, fpi_device_ft9366, FPI, DEVICE_FT9366, FpDevice);
G_DEFINE_TYPE (FpiDeviceFt9366, fpi_device_ft9366, FP_TYPE_DEVICE);

static guint16
reply_u16 (FpiDeviceFt9366 *self, guint i)
{
  return FP_READ_UINT16_BE (&self->reply[2 * i]);
}

/* ---- op runner ---------------------------------------------------------- */
/*
 * seq_run() sends every op of a table in order, reads the reply where there
 * is one, and moves the ssm to its next state once the table is done. The
 * reply of the last reading op stays in self->reply.
 */

static void seq_step (FpiSsm          *ssm,
                      FpiDeviceFt9366 *self);

static void
seq_op_done (FpiSsm *ssm, FpiDeviceFt9366 *self)
{
  self->seq_pos++;
  if (self->seq_pos < self->seq_len)
    seq_step (ssm, self);
  else
    fpi_ssm_next_state (ssm);
}

static void
seq_reply_cb (FpiUsbTransfer *transfer, FpDevice *dev,
              gpointer user_data, GError *error)
{
  FpiDeviceFt9366 *self = FPI_DEVICE_FT9366 (dev);

  if (error)
    {
      fpi_ssm_mark_failed (transfer->ssm, error);
      return;
    }

  self->reply_len = transfer->actual_length;
  memcpy (self->reply, transfer->buffer, transfer->actual_length);
  seq_op_done (transfer->ssm, self);
}

static gsize
op_reply_len (const Ft9366Op *op)
{
  switch (op->kind)
    {
    case OP_RD16:
    case OP_BURST:
      return 2 * op->val;

    case OP_RD8:
      return 1;

    case OP_RAW:
      return op->val;

    case OP_BRIDGE_CFG:
      return 2;

    case OP_WR16:
    case OP_WR8:
    case OP_CMD:
    case OP_DAC:
    case OP_FDT_BASE:
    default:
      return 0;
    }
}

static void
seq_sent_cb (FpiUsbTransfer *transfer, FpDevice *dev,
             gpointer user_data, GError *error)
{
  FpiDeviceFt9366 *self = FPI_DEVICE_FT9366 (dev);
  gsize len;
  FpiUsbTransfer *in;

  if (error)
    {
      fpi_ssm_mark_failed (transfer->ssm, error);
      return;
    }

  len = op_reply_len (&self->seq[self->seq_pos]);
  if (len == 0)
    {
      seq_op_done (transfer->ssm, self);
      return;
    }

  in = fpi_usb_transfer_new (dev);
  in->ssm = transfer->ssm;
  in->short_is_error = TRUE;
  fpi_usb_transfer_fill_bulk (in, FT9366_EP_IN, len);
  fpi_usb_transfer_submit (in, FT9366_TIMEOUT, fpi_device_get_cancellable (dev),
                           seq_reply_cb, NULL);
}

static FpiUsbTransfer *
op_build (FpDevice *dev, FpiDeviceFt9366 *self, const Ft9366Op *op)
{
  FpiUsbTransfer *t = fpi_usb_transfer_new (dev);
  guint8 opcode;
  gsize len;

  switch (op->kind)
    {
    case OP_RAW:
      fpi_usb_transfer_fill_bulk (t, FT9366_EP_OUT, op->raw_len);
      memcpy (t->buffer, op->raw, op->raw_len);
      return t;

    case OP_BRIDGE_CFG:
      /* 4b 00, fdt dac, img dac, 0xc8 (integration time?), baseline twice */
      fpi_usb_transfer_fill_bulk (t, FT9366_EP_OUT, 24);
      memset (t->buffer, 0, 24);
      t->buffer[0] = 0x4b;
      t->buffer[2] = FT9366_FDT_DAC;
      t->buffer[4] = FT9366_IMG_DAC;
      t->buffer[6] = 0xc8;
      for (guint i = 0; i < 4; i++)
        {
          FP_WRITE_UINT16_BE (&t->buffer[7 + 2 * i], self->fdt_base[i]);
          FP_WRITE_UINT16_BE (&t->buffer[15 + 2 * i], self->fdt_base[i]);
        }
      return t;

    case OP_CMD:
      fpi_usb_transfer_fill_bulk (t, FT9366_EP_OUT, 3);
      t->buffer[0] = op->reg;
      t->buffer[1] = ~op->reg;
      t->buffer[2] = 0;
      return t;

    case OP_WR8:
      opcode = 0x09;
      len = 4;
      break;

    case OP_RD8:
      opcode = 0x08;
      len = 5;
      break;

    case OP_RD16:
      opcode = 0x04;
      len = 6;
      break;

    case OP_BURST:
      opcode = 0x06;
      len = 6;
      break;

    case OP_FDT_BASE:
      opcode = 0x05;
      len = 6 + 2 * 8;
      break;

    case OP_WR16:
    case OP_DAC:
    default:
      opcode = 0x05;
      len = 8;
      break;
    }

  fpi_usb_transfer_fill_bulk (t, FT9366_EP_OUT, len);
  memset (t->buffer, 0, len);
  t->buffer[0] = opcode;
  t->buffer[1] = ~opcode;

  switch (op->kind)
    {
    case OP_WR8:
      t->buffer[2] = op->reg;
      t->buffer[3] = op->val;
      break;

    case OP_RD8:
      t->buffer[2] = op->reg;
      break;

    case OP_RD16:
    case OP_BURST:
      FP_WRITE_UINT16_BE (&t->buffer[2], op->reg);
      FP_WRITE_UINT16_BE (&t->buffer[4], op->val);
      break;

    case OP_FDT_BASE:
      /* 4 zones + 4 unused slots, as the vendor driver writes it */
      FP_WRITE_UINT16_BE (&t->buffer[2], op->reg);
      FP_WRITE_UINT16_BE (&t->buffer[4], 8);
      for (guint i = 0; i < 4; i++)
        FP_WRITE_UINT16_BE (&t->buffer[6 + 2 * i], self->fdt_base[i]);
      break;

    case OP_DAC:
      FP_WRITE_UINT16_BE (&t->buffer[2], FT9366_REG_DAC);
      FP_WRITE_UINT16_BE (&t->buffer[4], 1);
      FP_WRITE_UINT16_BE (&t->buffer[6], 0xfc80 | op->val);
      break;

    case OP_WR16:
      FP_WRITE_UINT16_BE (&t->buffer[2], op->reg);
      FP_WRITE_UINT16_BE (&t->buffer[4], 1);
      FP_WRITE_UINT16_BE (&t->buffer[6], op->val);
      break;

    case OP_BRIDGE_CFG:
    case OP_CMD:
    case OP_RAW:
    default:
      g_assert_not_reached ();
    }

  return t;
}

static void
seq_step (FpiSsm *ssm, FpiDeviceFt9366 *self)
{
  FpDevice *dev = FP_DEVICE (self);
  FpiUsbTransfer *t = op_build (dev, self, &self->seq[self->seq_pos]);

  t->ssm = ssm;
  t->short_is_error = TRUE;
  fpi_usb_transfer_submit (t, FT9366_TIMEOUT, fpi_device_get_cancellable (dev),
                           seq_sent_cb, NULL);
}

static void
seq_run_n (FpiSsm *ssm, FpDevice *dev, const Ft9366Op *seq, gsize len)
{
  FpiDeviceFt9366 *self = FPI_DEVICE_FT9366 (dev);

  self->seq = seq;
  self->seq_len = len;
  self->seq_pos = 0;
  seq_step (ssm, self);
}

#define seq_run(ssm, dev, table) seq_run_n (ssm, dev, table, G_N_ELEMENTS (table))

static void
store_frame (FpiDeviceFt9366 *self, guint16 *dst)
{
  guint nonzero = 0;
  guint16 max = 0;
  guint64 sum = 0;

  for (guint i = 0; i < FT9366_FRAME_PIXELS; i++)
    {
      dst[i] = reply_u16 (self, i);
      nonzero += dst[i] != 0;
      sum += dst[i];
      max = MAX (max, dst[i]);
    }
  fp_dbg ("frame stored: %u of %d pixels nonzero, mean %u, max %u, reply %" G_GSIZE_FORMAT " bytes",
          nonzero, FT9366_FRAME_PIXELS, (guint) (sum / FT9366_FRAME_PIXELS), max, self->reply_len);
}

/* A zone that reads 0 (unused on some units) must not wrap to 65505 and
 * poison the baseline the chip compares against. */
static inline guint16
fdt_base_from (guint16 reading)
{
  return reading > FT9366_FDT_BASE_DROP ? reading - FT9366_FDT_BASE_DROP : 0;
}

/* ---- activation: reset, finger-detect baseline, background frame -------- */

enum {
  INIT_HELLO,
  INIT_AFE_IMAGE,
  INIT_SETUP,
  INIT_FDT_MEASURE,
  INIT_FDT_POLL,
  INIT_FDT_CHECK,
  INIT_FDT_VALUES,
  INIT_FDT_STORE,
  INIT_FDT_IDLE,
  INIT_BG_AFE,
  INIT_BG_FRAME,
  INIT_BG_STORE,
  INIT_BG_DONE,
  INIT_BRIDGE_AFE,
  INIT_BRIDGE_CFG,
  INIT_NUM_STATES,
};

static void
init_run_state (FpiSsm *ssm, FpDevice *dev)
{
  FpiDeviceFt9366 *self = FPI_DEVICE_FT9366 (dev);

  switch (fpi_ssm_get_cur_state (ssm))
    {
    case INIT_HELLO:
      seq_run (ssm, dev, seq_hello);
      break;

    case INIT_AFE_IMAGE:
      seq_run (ssm, dev, seq_afe_image);
      break;

    case INIT_SETUP:
      seq_run (ssm, dev, seq_setup);
      break;

    case INIT_FDT_MEASURE:
      seq_run (ssm, dev, seq_fdt_measure);
      break;

    case INIT_FDT_POLL:
      seq_run (ssm, dev, seq_read_irq);
      break;

    case INIT_FDT_CHECK:
      if (reply_u16 (self, 0) & FT9366_IRQ_FDT_DONE)
        fpi_ssm_next_state (ssm);
      else
        fpi_ssm_jump_to_state_delayed (ssm, INIT_FDT_POLL, 5);
      break;

    case INIT_FDT_VALUES:
      seq_run (ssm, dev, seq_fdt_values);
      break;

    case INIT_FDT_STORE:
      /*
       * A baseline sampled while a finger rests on the sensor breaks finger
       * detection for the whole session (the vendor driver's GDM bug). Once
       * we have a good one, keep it whenever the new sample looks touched.
       */
      self->finger_at_init = FALSE;
      for (guint i = 0; i < 4; i++)
        if (self->have_base &&
            reply_u16 (self, i) - FT9366_FDT_BASE_DROP + FT9366_FDT_TOUCH_DROP < self->fdt_base[i])
          self->finger_at_init = TRUE;

      if (self->finger_at_init)
        {
          fp_dbg ("sensor touched during activation, keeping baseline");
        }
      else
        {
          for (guint i = 0; i < 4; i++)
            self->fdt_base[i] = fdt_base_from (reply_u16 (self, i));
          self->have_base = TRUE;
        }
      fp_dbg ("fdt baseline %u %u %u %u",
              self->fdt_base[0], self->fdt_base[1], self->fdt_base[2], self->fdt_base[3]);
      fpi_ssm_next_state (ssm);
      break;

    case INIT_FDT_IDLE:
      seq_run (ssm, dev, seq_idle);
      break;

    case INIT_BG_AFE:
      seq_run (ssm, dev, seq_afe_image);
      break;

    case INIT_BG_FRAME:
      seq_run (ssm, dev, seq_frame);
      break;

    case INIT_BG_STORE:
      if (!self->finger_at_init)
        {
          store_frame (self, self->background);
          ft9366_dead_pixels (self->background, self->dead);
        }
      fpi_ssm_next_state (ssm);
      break;

    case INIT_BG_DONE:
      seq_run (ssm, dev, seq_frame_done);
      break;

    case INIT_BRIDGE_AFE:
      seq_run (ssm, dev, seq_afe_fdt);
      break;

    case INIT_BRIDGE_CFG:
      seq_run (ssm, dev, seq_bridge_cfg);
      break;

    default:
      g_assert_not_reached ();
    }
}

/* ---- image processing --------------------------------------------------- */

/* self->frame -> self->image; returns the share of the sensor covered */
static gdouble
process_frame (FpiDeviceFt9366 *self, gboolean dump)
{
  const gint w = FT9366_FRAME_WIDTH, h = FT9366_FRAME_HEIGHT;
  gdouble coverage = ft9366_process_frame (self->frame, self->background, self->dead,
                                           self->image);

  /* development aid: FT9366_DUMP=dir writes every capture as PGM, plus the
   * raw 16-bit frame and background so processing can be redone offline */
  if (dump && g_getenv ("FT9366_DUMP"))
    {
      g_autofree char *base = g_strdup_printf ("%s/ft9366-%" G_GINT64_FORMAT,
                                               g_getenv ("FT9366_DUMP"), g_get_real_time ());
      g_autofree char *path = g_strconcat (base, ".pgm", NULL);
      g_autofree char *header = g_strdup_printf ("P5 %d %d 255\n", w, h);
      g_autofree char *header16 = g_strdup_printf ("P5 %d %d 65535\n", w, h);
      const guint16 *raw[2] = { self->frame, self->background };
      const char *suffix[2] = { ".frame.pgm", ".bg.pgm" };
      g_autoptr(GByteArray) pgm = g_byte_array_new ();

      g_byte_array_append (pgm, (guint8 *) header, strlen (header));
      g_byte_array_append (pgm, self->image, FT9366_FRAME_PIXELS);
      g_file_set_contents (path, (char *) pgm->data, pgm->len, NULL);

      for (guint k = 0; k < 2; k++)
        {
          g_autofree char *raw_path = g_strconcat (base, suffix[k], NULL);
          g_autoptr(GByteArray) out = g_byte_array_new ();

          g_byte_array_append (out, (guint8 *) header16, strlen (header16));
          for (guint i = 0; i < FT9366_FRAME_PIXELS; i++)
            {
              guint8 be[2] = { raw[k][i] >> 8, raw[k][i] & 0xff };
              g_byte_array_append (out, be, 2);
            }
          g_file_set_contents (raw_path, (char *) out->data, out->len, NULL);
        }
    }

  return coverage;
}

/* ---- prints ------------------------------------------------------------- */

static GVariant *
print_data_new (GPtrArray *images)
{
  GVariantBuilder builder;

  g_variant_builder_init (&builder, G_VARIANT_TYPE ("aay"));
  for (guint i = 0; i < images->len; i++)
    g_variant_builder_add_value (&builder,
                                 g_variant_new_fixed_array (G_VARIANT_TYPE_BYTE,
                                                            g_ptr_array_index (images, i),
                                                            FT9366_FRAME_PIXELS, 1));

  return g_variant_new ("(qaay)", FT9366_PRINT_VERSION, &builder);
}

/* the current capture against an enrolled print: its images are registered
 * into a template (ft9366-match.c) and matched as a whole */
static gboolean
match_print (FpiDeviceFt9366 *self, FpPrint *print, Ft9366MatchInfo *info)
{
  g_autoptr(GVariant) data = NULL;
  g_autoptr(GVariant) images = NULL;
  g_autoptr(Ft9366Features) probe = NULL;
  g_autoptr(Ft9366Template) tpl = NULL;
  g_autofree const guint8 **imgs = NULL;
  guint16 version;
  guint n = 0;

  *info = (Ft9366MatchInfo) { 0 };
  g_object_get (print, "fpi-data", &data, NULL);
  if (!data || !g_variant_check_format_string (data, "(qaay)", FALSE))
    return FALSE;

  g_variant_get (data, "(q@aay)", &version, &images);
  if (version != FT9366_PRINT_VERSION)
    return FALSE;

  imgs = g_new (const guint8 *, g_variant_n_children (images));
  for (gsize i = 0; i < g_variant_n_children (images); i++)
    {
      g_autoptr(GVariant) child = g_variant_get_child_value (images, i);
      gsize len;
      const guint8 *img = g_variant_get_fixed_array (child, &len, 1);

      /* the data stays valid: images holds a reference to every child */
      if (len == FT9366_FRAME_PIXELS)
        imgs[n++] = img;
    }
  if (n == 0)
    return FALSE;

  tpl = ft9366_template_new (imgs, n);
  probe = ft9366_features_new (self->image);
  ft9366_template_match_info (tpl, probe, info);
  return ft9366_match_accept (info);
}

/* ---- capture loop ------------------------------------------------------- */
/*
 * One run: wait for a finger, grab a frame into self->image, wait for the
 * finger to leave. While waiting for a finger the baseline is re-checked
 * every FT9366_HEAL_POLLS polls, so a baseline taken with a finger on the
 * sensor heals itself as soon as the finger is lifted.
 */

enum {
  CAPTURE_ARM_AFE,       /* finger-detect config + baseline */
  CAPTURE_ARM,           /* wait for finger down (or up) */
  CAPTURE_POLL,
  CAPTURE_POLL_CHECK,
  CAPTURE_IRQ_READ,
  CAPTURE_IRQ_ACK,
  CAPTURE_IRQ_CHECK,
  CAPTURE_AFE_IMAGE,
  CAPTURE_FRAME,
  CAPTURE_FRAME_STORE,
  CAPTURE_FRAME_DONE,
  CAPTURE_FRAME_NEXT,    /* another frame, or keep the best one */
  CAPTURE_TO_UP,
  CAPTURE_MEASURE_IDLE,  /* manual measurement: confirm up / re-check baseline */
  CAPTURE_MEASURE,
  CAPTURE_MEASURE_POLL,
  CAPTURE_MEASURE_POLL_CHECK,
  CAPTURE_MEASURE_VALUES,
  CAPTURE_MEASURE_CHECK,
  CAPTURE_NUM_STATES,
};

static void
capture_run_state (FpiSsm *ssm, FpDevice *dev)
{
  FpiDeviceFt9366 *self = FPI_DEVICE_FT9366 (dev);
  guint16 flags;

  if (g_cancellable_is_cancelled (fpi_device_get_cancellable (dev)))
    {
      fpi_ssm_mark_failed (ssm, g_error_new_literal (G_IO_ERROR, G_IO_ERROR_CANCELLED,
                                                     "Cancelled"));
      return;
    }

  switch (fpi_ssm_get_cur_state (ssm))
    {
    case CAPTURE_ARM_AFE:
      self->polls = 0;
      seq_run (ssm, dev, seq_afe_fdt);
      break;

    case CAPTURE_ARM:
      if (self->wait_up)
        seq_run (ssm, dev, seq_arm_up);
      else
        seq_run (ssm, dev, seq_arm_down);
      break;

    case CAPTURE_POLL:
      seq_run (ssm, dev, seq_poll_irq);
      break;

    case CAPTURE_POLL_CHECK:
      if (reply_u16 (self, 0) == FT9366_IRQ_PENDING)
        fpi_ssm_next_state (ssm);
      else if (++self->polls >= FT9366_HEAL_POLLS)
        fpi_ssm_jump_to_state (ssm, CAPTURE_MEASURE_IDLE);
      else
        fpi_ssm_jump_to_state_delayed (ssm, CAPTURE_POLL, FT9366_POLL_MS);
      break;

    case CAPTURE_IRQ_READ:
      seq_run (ssm, dev, seq_read_irq);
      break;

    case CAPTURE_IRQ_ACK:
      /* a write: the flags read above stay in the reply buffer */
      seq_run (ssm, dev, seq_ack_irq);
      break;

    case CAPTURE_IRQ_CHECK:
      flags = reply_u16 (self, 0);
      fp_dbg ("irq flags 0x%04x, waiting for %s", flags, self->wait_up ? "up" : "down");

      if (self->wait_up)
        /* in up mode the IRQ itself is the event (9a82 still says "down"),
         * but it also fires early; confirm before trusting it */
        fpi_ssm_jump_to_state (ssm, CAPTURE_MEASURE_IDLE);
      else if ((flags & FT9366_IRQ_BASE_BAD) && !(flags & FT9366_IRQ_DOWN))
        {
          /* The chip keeps raising this while its baseline is wrong (taken
           * with a finger on), so the idle re-check never gets its turn.
           * Measure now; MEASURE_CHECK re-takes the baseline once the
           * readings settle. */
          self->base_suspect = TRUE;
          fpi_ssm_jump_to_state (ssm, CAPTURE_MEASURE_IDLE);
        }
      else if (flags & FT9366_IRQ_DOWN)
        {
          fpi_device_report_finger_status (dev, FP_FINGER_STATUS_NEEDED | FP_FINGER_STATUS_PRESENT);
          self->frames_taken = 0;
          self->best_coverage = -1;
          fpi_ssm_jump_to_state_delayed (ssm, CAPTURE_AFE_IMAGE, FT9366_SETTLE_MS);
        }
      else
        fpi_ssm_jump_to_state (ssm, CAPTURE_ARM_AFE);
      break;

    case CAPTURE_AFE_IMAGE:
      if (self->sw_down)
        {
          self->sw_down = FALSE;
          seq_run (ssm, dev, seq_idle_afe_image);
        }
      else
        seq_run (ssm, dev, seq_afe_image);
      break;

    case CAPTURE_FRAME:
      seq_run (ssm, dev, seq_frame);
      break;

    case CAPTURE_FRAME_STORE:
      {
        gdouble coverage;

        store_frame (self, self->frame);
        coverage = process_frame (self, FALSE);
        fp_dbg ("frame %u covers %.0f%%", self->frames_taken, coverage * 100);
        if (coverage > self->best_coverage)
          {
            self->best_coverage = coverage;
            memcpy (self->best_frame, self->frame, sizeof self->frame);
          }
        self->frames_taken++;
        fpi_ssm_next_state (ssm);
      }
      break;

    case CAPTURE_FRAME_DONE:
      seq_run (ssm, dev, seq_frame_done);
      break;

    case CAPTURE_FRAME_NEXT:
      if (self->best_coverage < FT9366_GOOD_COVERAGE && self->frames_taken < FT9366_MAX_FRAMES)
        {
          fpi_ssm_jump_to_state_delayed (ssm, CAPTURE_AFE_IMAGE, FT9366_FRAME_GAP_MS);
          break;
        }
      memcpy (self->frame, self->best_frame, sizeof self->frame);
      fpi_ssm_next_state (ssm);
      break;

    case CAPTURE_TO_UP:
      self->wait_up = TRUE;
      fpi_ssm_jump_to_state (ssm, CAPTURE_ARM_AFE);
      break;

    case CAPTURE_MEASURE_IDLE:
      seq_run (ssm, dev, seq_idle);
      break;

    case CAPTURE_MEASURE:
      seq_run (ssm, dev, seq_up_measure);
      break;

    case CAPTURE_MEASURE_POLL:
      seq_run (ssm, dev, seq_read_irq);
      break;

    case CAPTURE_MEASURE_POLL_CHECK:
      if (reply_u16 (self, 0) & FT9366_IRQ_FDT_DONE)
        fpi_ssm_next_state (ssm);
      else
        fpi_ssm_jump_to_state_delayed (ssm, CAPTURE_MEASURE_POLL, 5);
      break;

    case CAPTURE_MEASURE_VALUES:
      seq_run (ssm, dev, seq_fdt_values);
      break;

    case CAPTURE_MEASURE_CHECK:
      {
        gboolean touched = FALSE, above = TRUE;

        /* Only zone 0 follows manual measurements on these units: zones 1-3
         * keep the value they had when the finger last landed, so they would
         * report "touched" forever after a capture. */
        for (guint i = 0; i < 1; i++)
          {
            gint level = reply_u16 (self, i) - FT9366_FDT_BASE_DROP;

            if (level + FT9366_FDT_TOUCH_DROP < self->fdt_base[i])
              touched = TRUE;
            if (level < self->fdt_base[i] + FT9366_FDT_TOUCH_DROP)
              above = FALSE;
          }

        fp_dbg ("measure %u %u %u %u: %s", reply_u16 (self, 0), reply_u16 (self, 1),
                reply_u16 (self, 2), reply_u16 (self, 3), touched ? "touched" : "free");

        if (self->base_suspect)
          {
            gboolean stable = self->have_last_fdt;

            for (guint i = 0; i < 4; i++)
              {
                stable &= ABS ((gint) reply_u16 (self, i) - (gint) self->last_fdt[i]) <= FT9366_FDT_STABLE;
                self->last_fdt[i] = reply_u16 (self, i);
              }
            self->have_last_fdt = TRUE;

            /* A baseline taken with a finger on is off in both directions
             * (some zones lower, some higher), so "above" below never
             * triggers. Two steady readings in a row mean nothing is moving
             * on the sensor. The chip's own calibration from that activation
             * is off too and keeps raising IRQ_BASE_BAD whatever baseline we
             * write, so recalibrate it from scratch, like the vendor driver
             * does ("wrong bae, rek"). If a finger is still resting, the chip
             * flags it again once it lifts. */
            if (stable && self->reinits < FT9366_MAX_REINITS)
              {
                self->reinits++;
                self->have_base = FALSE;
                self->base_suspect = FALSE;
                self->have_last_fdt = FALSE;
                self->reinit_pending = TRUE;
                fp_info ("finger-detect baseline was taken with a finger on, recalibrating");
                fpi_ssm_mark_completed (ssm);
                break;
              }
            if (stable)
              {
                for (guint i = 0; i < 4; i++)
                  self->fdt_base[i] = fdt_base_from (reply_u16 (self, i));
                self->base_suspect = FALSE;
                self->have_last_fdt = FALSE;
                touched = FALSE;
                above = FALSE;
                fp_dbg ("baseline re-taken: %u %u %u %u", self->fdt_base[0],
                        self->fdt_base[1], self->fdt_base[2], self->fdt_base[3]);
              }
          }

        if (!touched && above)
          {
            /* the sensor reads clearly above the baseline: the baseline was
             * taken with a finger on it. Take the free reading instead. */
            for (guint i = 0; i < 4; i++)
              self->fdt_base[i] = fdt_base_from (reply_u16 (self, i));
            fp_dbg ("baseline healed to %u %u %u %u", self->fdt_base[0],
                    self->fdt_base[1], self->fdt_base[2], self->fdt_base[3]);
          }

        if (!self->wait_up && touched)
          {
            /* The chip's own finger-down IRQ never fires on some units (zones
             * 1-3 read 0). The manual measurement sees the touch, so use it. */
            fp_dbg ("software finger-down from measurement");
            self->sw_down = TRUE;
            fpi_device_report_finger_status (dev, FP_FINGER_STATUS_NEEDED | FP_FINGER_STATUS_PRESENT);
            self->frames_taken = 0;
            self->best_coverage = -1;
            fpi_ssm_jump_to_state_delayed (ssm, CAPTURE_AFE_IMAGE, FT9366_SETTLE_MS);
            break;
          }

        if (self->wait_up && !touched)
          {
            self->wait_up = FALSE;
            fpi_device_report_finger_status (dev, FP_FINGER_STATUS_NONE);
            fpi_ssm_mark_completed (ssm);
          }
        else
          {
            fpi_ssm_jump_to_state_delayed (ssm, CAPTURE_ARM_AFE, FT9366_POLL_MS);
          }
      }
      break;

    default:
      g_assert_not_reached ();
    }
}

/* ---- actions ------------------------------------------------------------ */

static void start_capture (FpiDeviceFt9366 *self);
static void start_action (FpiDeviceFt9366 *self, Ft9366Action action);

static void
action_fail (FpiDeviceFt9366 *self, GError *error)
{
  FpDevice *dev = FP_DEVICE (self);
  Ft9366Action action = self->action;

  self->action = OP_NONE;
  g_clear_pointer (&self->enroll_images, g_ptr_array_unref);

  switch (action)
    {
    case OP_ENROLL:
      fpi_device_enroll_complete (dev, NULL, error);
      break;

    case OP_VERIFY:
      fpi_device_verify_complete (dev, error);
      break;

    case OP_IDENTIFY:
      fpi_device_identify_complete (dev, error);
      break;

    case OP_NONE:
    default:
      g_error_free (error);
    }
}

static void
enroll_image (FpiDeviceFt9366 *self, gdouble coverage)
{
  FpDevice *dev = FP_DEVICE (self);
  FpPrint *print = NULL;

  if (coverage < FT9366_MIN_COVERAGE)
    {
      fpi_device_enroll_progress (dev, self->enroll_images->len, NULL,
                                  fpi_device_retry_new (FP_DEVICE_RETRY_CENTER_FINGER));
      start_capture (self);
      return;
    }

  if (self->enroll_images->len > 0 && self->enroll_dups < FT9366_MAX_DUPS)
    {
      g_autoptr(Ft9366Template) tpl = ft9366_template_new ((const guint8 * const *) self->enroll_images->pdata,
                                                           self->enroll_images->len);
      g_autoptr(Ft9366Features) probe = ft9366_features_new (self->image);
      Ft9366MatchInfo info;

      ft9366_template_match_info (tpl, probe, &info);
      if (ft9366_match_accept (&info) && info.overlap >= FT9366_DUP_OVERLAP &&
          info.ncc >= FT9366_DUP_NCC)
        {
          fp_dbg ("enroll touch already covered (%.0f%%, correlation %.2f)",
                  info.overlap * 100, info.ncc);
          self->enroll_dups++;
          fpi_device_enroll_progress (dev, self->enroll_images->len, NULL,
                                      fpi_device_retry_new_msg (FP_DEVICE_RETRY_CENTER_FINGER,
                                                                "Move your finger slightly"));
          start_capture (self);
          return;
        }
    }
  self->enroll_dups = 0;

  g_ptr_array_add (self->enroll_images, g_memdup2 (self->image, FT9366_FRAME_PIXELS));
  fpi_device_enroll_progress (dev, self->enroll_images->len, NULL, NULL);

  if (self->enroll_images->len < fp_device_get_nr_enroll_stages (dev))
    {
      start_capture (self);
      return;
    }

  fpi_device_get_enroll_data (dev, &print);
  fpi_print_set_type (print, FPI_PRINT_RAW);
  g_object_set (print, "fpi-data", print_data_new (self->enroll_images), NULL);

  self->action = OP_NONE;
  g_clear_pointer (&self->enroll_images, g_ptr_array_unref);
  fpi_device_enroll_complete (dev, g_object_ref (print), NULL);
}

static void
verify_image (FpiDeviceFt9366 *self, gdouble coverage)
{
  FpDevice *dev = FP_DEVICE (self);
  FpPrint *print = NULL;
  Ft9366MatchInfo info;
  gboolean match;

  self->action = OP_NONE;

  if (coverage < FT9366_MIN_COVERAGE)
    {
      fpi_device_verify_report (dev, FPI_MATCH_ERROR, NULL,
                                fpi_device_retry_new (FP_DEVICE_RETRY_CENTER_FINGER));
      fpi_device_verify_complete (dev, NULL);
      return;
    }

  fpi_device_get_verify_data (dev, &print);
  match = match_print (self, print, &info);
  fp_info ("verify score %u inliers, ridge correlation %.2f over %.0f%%: %s",
           info.inliers, info.ncc, info.overlap * 100, match ? "match" : "no match");

  fpi_device_verify_report (dev, match ? FPI_MATCH_SUCCESS : FPI_MATCH_FAIL, NULL, NULL);
  fpi_device_verify_complete (dev, NULL);
}

static void
identify_image (FpiDeviceFt9366 *self, gdouble coverage)
{
  FpDevice *dev = FP_DEVICE (self);
  GPtrArray *prints = NULL;
  FpPrint *match = NULL;
  guint best = 0;

  self->action = OP_NONE;

  if (coverage < FT9366_MIN_COVERAGE)
    {
      fpi_device_identify_report (dev, NULL, NULL,
                                  fpi_device_retry_new (FP_DEVICE_RETRY_CENTER_FINGER));
      fpi_device_identify_complete (dev, NULL);
      return;
    }

  fpi_device_get_identify_data (dev, &prints);
  for (guint i = 0; i < prints->len; i++)
    {
      FpPrint *print = g_ptr_array_index (prints, i);
      Ft9366MatchInfo info;
      gboolean accept = match_print (self, print, &info);

      fp_info ("identify print %u: %u inliers, ridge correlation %.2f: %s", i,
               info.inliers, info.ncc, accept ? "match" : "no match");
      if (accept && info.inliers > best)
        {
          best = info.inliers;
          match = print;
        }
    }

  fpi_device_identify_report (dev, match, NULL, NULL);
  fpi_device_identify_complete (dev, NULL);
}

static void
capture_complete (FpiSsm *ssm, FpDevice *dev, GError *error)
{
  FpiDeviceFt9366 *self = FPI_DEVICE_FT9366 (dev);
  gdouble coverage;

  self->ssm = NULL;

  if (error)
    {
      action_fail (self, error);
      return;
    }

  if (self->reinit_pending)
    {
      self->reinit_pending = FALSE;
      start_action (self, self->action);
      return;
    }
  self->reinits = 0;

  coverage = process_frame (self, TRUE);
  fp_dbg ("finger covered %.0f%% of the sensor (best of %u frames)", coverage * 100,
          self->frames_taken);

  switch (self->action)
    {
    case OP_ENROLL:
      enroll_image (self, coverage);
      break;

    case OP_VERIFY:
      verify_image (self, coverage);
      break;

    case OP_IDENTIFY:
      identify_image (self, coverage);
      break;

    case OP_NONE:
    default:
      break;
    }
}

static void
start_capture (FpiDeviceFt9366 *self)
{
  self->wait_up = FALSE;
  fpi_device_report_finger_status (FP_DEVICE (self), FP_FINGER_STATUS_NEEDED);
  self->ssm = fpi_ssm_new (FP_DEVICE (self), capture_run_state, CAPTURE_NUM_STATES);
  fpi_ssm_start (self->ssm, capture_complete);
}

static void
init_complete (FpiSsm *ssm, FpDevice *dev, GError *error)
{
  FpiDeviceFt9366 *self = FPI_DEVICE_FT9366 (dev);

  self->ssm = NULL;
  if (error)
    action_fail (self, error);
  else
    start_capture (self);
}

/* every action starts from a freshly initialised sensor */
static void
start_action (FpiDeviceFt9366 *self, Ft9366Action action)
{
  self->action = action;
  self->ssm = fpi_ssm_new (FP_DEVICE (self), init_run_state, INIT_NUM_STATES);
  fpi_ssm_start (self->ssm, init_complete);
}

/* ---- device lifecycle --------------------------------------------------- */

static void
ft9366_open (FpDevice *dev)
{
  GUsbDevice *usb = fpi_device_get_usb_device (dev);
  GError *error = NULL;

  /* the bridge wedges if a previous user died mid-transfer; the vendor
   * driver resets it on every open too */
  if (!g_usb_device_reset (usb, &error))
    {
      fpi_device_open_complete (dev, error);
      return;
    }

  g_usb_device_claim_interface (usb, 0, 0, &error);
  fpi_device_open_complete (dev, error);
}

static void
ft9366_close (FpDevice *dev)
{
  GError *error = NULL;

  g_usb_device_release_interface (fpi_device_get_usb_device (dev), 0, 0, &error);
  fpi_device_close_complete (dev, error);
}

static void
ft9366_enroll (FpDevice *dev)
{
  FpiDeviceFt9366 *self = FPI_DEVICE_FT9366 (dev);

  self->enroll_images = g_ptr_array_new_with_free_func (g_free);
  self->enroll_dups = 0;
  start_action (self, OP_ENROLL);
}

static void
ft9366_verify (FpDevice *dev)
{
  start_action (FPI_DEVICE_FT9366 (dev), OP_VERIFY);
}

static void
ft9366_identify (FpDevice *dev)
{
  start_action (FPI_DEVICE_FT9366 (dev), OP_IDENTIFY);
}

static const FpIdEntry id_table[] = {
  { .vid = 0x2808, .pid = 0xa658, },
  { .vid = 0, .pid = 0, .driver_data = 0 },
};

static void
fpi_device_ft9366_init (FpiDeviceFt9366 *self)
{
}

static void
fpi_device_ft9366_class_init (FpiDeviceFt9366Class *klass)
{
  FpDeviceClass *dev_class = FP_DEVICE_CLASS (klass);

  dev_class->id = FP_COMPONENT;
  dev_class->full_name = "FocalTech FT9366 (RTS5811)";
  dev_class->type = FP_DEVICE_TYPE_USB;
  dev_class->id_table = id_table;
  dev_class->scan_type = FP_SCAN_TYPE_PRESS;
  /* tiny sensor: every touch sees a different patch, collect more of them */
  dev_class->nr_enroll_stages = FT9366_ENROLL_STAGES;
  /* capacitive, does not heat up (as realtek, focaltech_moc) */
  dev_class->temp_hot_seconds = -1;

  dev_class->open = ft9366_open;
  dev_class->close = ft9366_close;
  dev_class->enroll = ft9366_enroll;
  dev_class->verify = ft9366_verify;
  dev_class->identify = ft9366_identify;

  fpi_device_class_auto_initialize_features (dev_class);
}
