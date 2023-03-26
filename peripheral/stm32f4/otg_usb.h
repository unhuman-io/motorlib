#ifndef UNHUMAN_MOTORLIB_PERIPHERAL_STM32F4_OTG_USB_H_
#define UNHUMAN_MOTORLIB_PERIPHERAL_STM32F4_OTG_USB_H_

#include <algorithm>
#include <cstdint>
#include <cstring>

#include "../st_device.h"
#include "../version.h"
#include "stm32f4xx_hal.h"
#include "stm32f4xx_ll_usb.h"

extern uint8_t go_to_bootloader;

#define USBx USB_OTG_FS
#define USBx_BASE ((uint32_t)USBx)
class USB_OTG {
 public:
  void send_data(uint8_t endpoint, const uint8_t *data, uint16_t length,
                 bool wait = true) {
    send_data32(endpoint, (uint32_t *)data, (length + 3) / 4, length);
  }

  void send_data32(uint8_t endpoint, const uint32_t *data, uint8_t length32,
                   uint8_t length8 = 0);
  void send_string(uint8_t endpoint, const char *str, uint8_t length);

  // receive up to length32 words from endpoint, return number of words read
  int receive_data(uint8_t endpoint, uint8_t *const data, uint8_t length8) {
    if (new_rx_data_[endpoint]) {
      new_rx_data_[endpoint] = false;
      asm("nop");  // addresses unknown bug
      length8 = std::min(length8, count_rx_[endpoint]);
      for (int i = 0; i < length8; i++) {
        data[i] = rx_data_[endpoint][i];
      }
      return length8;
    } else {
      return 0;
    }
  }

  void send_stall(uint8_t endpoint) {
    USBx_INEP(endpoint)->DIEPCTL |= USB_OTG_DIEPCTL_STALL;
  }

  void read_fifo(uint8_t byte_count, uint32_t *data) {
    for (int i = 0; i < (byte_count + 3) / 4; i++) {
      data[i] = USBx_DFIFO(0);
    }
  }

  void interrupt() {
    /* Handle Reset Interrupt */
    if (USBx->GINTSTS & USB_OTG_GINTSTS_USBRST) {
      // flush all fifos
      USBx->GRSTCTL = (USB_OTG_GRSTCTL_TXFFLSH | (uint32_t)(0x10 << 6));
      while (USBx->GRSTCTL & USB_OTG_GRSTCTL_TXFFLSH)
        ;

      for (int i = 0U; i < 4; i++) {
        USBx_INEP(i)->DIEPINT = 0xFFU;
        USBx_OUTEP(i)->DOEPINT = 0xFFU;
      }
      USBx_DEVICE->DAINT = 0xFFFFFFFFU;
      USBx_DEVICE->DAINTMSK |= 0x10001U;

      USBx_DEVICE->DOEPMSK |= (USB_OTG_DOEPMSK_STUPM | USB_OTG_DOEPMSK_XFRCM |
                               USB_OTG_DOEPMSK_EPDM | USB_OTG_DOEPMSK_OTEPSPRM);
      USBx_DEVICE->DIEPMSK |=
          (USB_OTG_DIEPMSK_TOM | USB_OTG_DIEPMSK_XFRCM | USB_OTG_DIEPMSK_EPDM);

      /* Set Default Address to 0 */
      USBx_DEVICE->DCFG = USB_OTG_DCFG_DAD;
      device_address_ = 0;

      /* setup EP0 to receive SETUP packets */
      USBx_OUTEP(0U)->DOEPTSIZ = 0U;
      USBx_OUTEP(0U)->DOEPTSIZ |= (USB_OTG_DOEPTSIZ_PKTCNT & (1U << 19U));
      USBx_OUTEP(0U)->DOEPTSIZ |= (3U * 8U);
      USBx_OUTEP(0U)->DOEPTSIZ |= USB_OTG_DOEPTSIZ_STUPCNT;

      USBx_INEP(1)->DIEPCTL = 0;
      USBx_INEP(2)->DIEPCTL = 0;

      USBx->GINTSTS = USB_OTG_GINTSTS_USBRST;
    }

    if (USBx->GINTSTS & USB_OTG_GINTSTS_USBSUSP) {
      USBx->GINTSTS = USB_OTG_GINTSTS_USBSUSP;
    }

    if (USBx->GINTSTS & USB_OTG_GINTSTS_ENUMDNE) {
      USBx_INEP(0U)->DIEPCTL &= ~USB_OTG_DIEPCTL_MPSIZ;
      USBx_DEVICE->DCTL |= USB_OTG_DCTL_CGINAK;

      USBx->GUSBCFG &= ~USB_OTG_GUSBCFG_TRDT;
      /* hclk Clock Range between 32-180 MHz */
      USBx->GUSBCFG |= (uint32_t)((0x6U << 10U) & USB_OTG_GUSBCFG_TRDT);

      USBx_DEVICE->DAINTMSK |= USB_OTG_DAINTMSK_IEPM & ((1U << (0)));
      USBx_INEP(0)->DIEPCTL |=
          ((64 & USB_OTG_DIEPCTL_MPSIZ) | (0 << 18U) | ((0) << 22U) |
           (USB_OTG_DIEPCTL_SD0PID_SEVNFRM) | (USB_OTG_DIEPCTL_USBAEP));

      USBx_DEVICE->DAINTMSK |= USB_OTG_DAINTMSK_OEPM & ((1U << (0)) << 16U);
      USBx_OUTEP(0)->DOEPCTL |=
          ((64 & USB_OTG_DOEPCTL_MPSIZ) | (0 << 18U) |
           (USB_OTG_DIEPCTL_SD0PID_SEVNFRM) | (USB_OTG_DOEPCTL_USBAEP));

      USBx->GINTSTS = USB_OTG_GINTSTS_ENUMDNE;
    }

    /* Handle RxQLevel Interrupt */
    if (USBx->GINTSTS & USB_OTG_GINTSTS_RXFLVL) {
      uint32_t temp = USBx->GRXSTSP;

      uint8_t ep_number = temp & USB_OTG_GRXSTSP_EPNUM;
      uint8_t byte_count =
          (temp & USB_OTG_GRXSTSP_BCNT) >> USB_OTG_GRXSTSP_BCNT_Pos;
      uint8_t packet_status = (temp & USB_OTG_PKTSTS) >> USB_OTG_PKTSTS_Pos;

      if (packet_status == STS_DATA_UPDT) {
        read_fifo(byte_count, (uint32_t *)rx_data_[ep_number]);
        count_rx_[ep_number] = byte_count;
        new_rx_data_[ep_number] = true;
      }
      if (packet_status == STS_SETUP_UPDT) {
        read_fifo(byte_count, reinterpret_cast<uint32_t *>(setup_data));
      }
      if (ep_number == 1) {
        USBx_OUTEP(1)->DOEPTSIZ = 0x80040;
        USBx_OUTEP(1)->DOEPCTL |= USB_OTG_DOEPCTL_EPENA | USB_OTG_DOEPCTL_CNAK;
      }
      if (ep_number == 2) {
        USBx_OUTEP(2)->DOEPTSIZ = 0x80040;
        USBx_OUTEP(2)->DOEPCTL |= USB_OTG_DOEPCTL_EPENA | USB_OTG_DOEPCTL_CNAK;
      }
    }

    if (USBx->GINTSTS & USB_OTG_GINTSTS_OEPINT) {
      uint16_t out_ep_interrupt =
          (USBx_DEVICE->DAINT & USBx_DEVICE->DAINTMSK) >>
          16u;                     // endpoints with out interrupts
      if (out_ep_interrupt & 1) {  // endpoint 0 interrupt
        if (USBx_OUTEP(0)->DOEPINT & USB_OTG_DOEPINT_XFRC) {
          // transfer complete
        }
        if (USBx_OUTEP(0)->DOEPINT & USB_OTG_DOEPINT_STUP) {
          // setup phase done
          handle_setup_packet(setup_data);
        }
        USBx_OUTEP(0)->DOEPINT = 0xFFFF;
      }
      if (out_ep_interrupt & (1 << 2)) {  // endpoint 2 interrupt
        if (USBx_OUTEP(2)->DOEPINT & USB_OTG_DOEPINT_XFRC) {
          // transfer complete
          // USBx_OUTEP(2)->DOEPTSIZ = 0x80040;
          // USBx_OUTEP(2)->DOEPCTL |= USB_OTG_DOEPCTL_EPENA |
          // USB_OTG_DOEPCTL_CNAK ;
        }
        USBx_OUTEP(2)->DOEPINT = 0xFFFF;
      }
      if (out_ep_interrupt & (1 << 1)) {  // endpoint 1 interrupt
        if (USBx_OUTEP(1)->DOEPINT & USB_OTG_DOEPINT_XFRC) {
          // transfer complete
          // USBx_OUTEP(2)->DOEPTSIZ = 0x80040;
          // USBx_OUTEP(2)->DOEPCTL |= USB_OTG_DOEPCTL_EPENA |
          // USB_OTG_DOEPCTL_CNAK ;
        }
        USBx_OUTEP(1)->DOEPINT = 0xFFFF;
      }
    }

    if (USBx->GINTSTS & USB_OTG_GINTSTS_IEPINT) {
      uint16_t in_ep_interrupt = (USBx_DEVICE->DAINT & USBx_DEVICE->DAINTMSK) &
                                 0xFFFF;  // endpoints with out interrupts
      if (in_ep_interrupt & 1) {          // endpoint 0 interrupt
        if (USBx_INEP(0)->DIEPINT & USB_OTG_DOEPINT_XFRC) {
          // transfer complete
          if (device_address_) {
            // asm("bkpt");
            //  USBx_DEVICE->DCFG &= ~USB_OTG_DCFG_DAD;
            //  USBx_DEVICE->DCFG |= device_address_ << USB_OTG_DCFG_DAD_Pos;
          }
        }
        USBx_INEP(0)->DIEPINT = 0xFFFF;
      }
      if (in_ep_interrupt & (1 << 2)) {
        USBx_INEP(2)->DIEPINT = 0xFFFF;
      }
      if (in_ep_interrupt & (1 << 1)) {
        USBx_INEP(1)->DIEPINT = 0xFFFF;
      }
    }
    // acknowledge all interrupts
    // USBx->GINTSTS = 0xFFFFFFFF;
  }

  // This is the serial number used by the bootloader, 13 bytes with null
  // terminator
  void Get_SerialNum(char *buffer);

  void handle_setup_packet(uint8_t *setup_data);
  bool tx_active(int ep_num) {
    return USBx_INEP(ep_num)->DIEPCTL & USB_OTG_DIEPCTL_EPENA;
  }

  bool new_rx_data(uint8_t endpoint) const { return new_rx_data_[endpoint]; }
  bool tx_data_ack(uint8_t endpoint) { return true; }
  void cancel_transfer(uint8_t endpoint) {}  // todo implement if useful

 private:
  uint8_t device_address_ = 0;
  uint8_t setup_data[64];
  uint16_t interface_ = 0;
  uint8_t rx_data_[4][64] = {};
  int sending_ = 0;
  bool new_rx_data_[4] = {};
  uint8_t count_rx_[4] = {};
  uint32_t error_count_ = 0;

  friend class System;
};

#endif  // UNHUMAN_MOTORLIB_PERIPHERAL_STM32F4_OTG_USB_H_
