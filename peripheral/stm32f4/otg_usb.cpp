#include "otg_usb.h"
#include "bootloader.h"

#include <cstdio>
#include "../..//util.h"

extern const char * const name;


#define LOBYTE(x)  ((uint8_t)(x & 0x00FF))
#define HIBYTE(x)  ((uint8_t)((x & 0xFF00) >>8))
#define  USB_DESC_TYPE_DEVICE                              1
#define  USB_DESC_TYPE_CONFIGURATION                       2
#define  USB_DESC_TYPE_STRING                              3
#define  USB_DESC_TYPE_INTERFACE                           4
#define  USB_DESC_TYPE_ENDPOINT                            5
#define  USB_DESC_TYPE_DEVICE_QUALIFIER                    6
#define  USB_DESC_TYPE_OTHER_SPEED_CONFIGURATION           7
#define  USB_DESC_TYPE_BOS                                 0x0F
#define  USBD_IDX_LANGID_STR                            0x00 
#define  USBD_IDX_MFC_STR                               0x01 
#define  USBD_IDX_PRODUCT_STR                           0x02
#define  USBD_IDX_SERIAL_STR                            0x03 
#define  USBD_IDX_CONFIG_STR                            0x04 
#define  USBD_IDX_INTERFACE_STR                         0x05 
#define USBD_BULK_SIZE                                  64
#define DFU_INTERFACE_NUMBER                            0x01

#define         DEVICE_ID1          (UID_BASE) //(0x1FFF7A10)
#define         DEVICE_ID2          (UID_BASE + 4) 
#define         DEVICE_ID3          (UID_BASE + 8) 

#define USBD_VID     0x3293 // Unhuman VID

const uint8_t USB_DEVICE_DESCIPTOR[]=
{
  0x12,                       /*bLength */
  USB_DESC_TYPE_DEVICE,       /*bDescriptorType*/
  0x00,                       /*bcdUSB */
  0x02,
  0xFF,                       /*bDeviceClass*/
  0x00,                       /*bDeviceSubClass*/
  0x00,                       /*bDeviceProtocol*/
  64,           /*bMaxPacketSize*/
  LOBYTE(USBD_VID),           /*idVendor*/
  HIBYTE(USBD_VID),           /*idVendor*/
  LOBYTE(USBD_PID),        /*idProduct*/
  HIBYTE(USBD_PID),        /*idProduct*/
  0x00,                       /*bcdDevice rel. 2.00*/
  0x02,
  USBD_IDX_MFC_STR,           /*Index of manufacturer  string*/
  USBD_IDX_PRODUCT_STR,       /*Index of product string*/
  USBD_IDX_SERIAL_STR,        /*Index of serial number string*/
  1  /*bNumConfigurations*/
};

const uint8_t USB_CONFIGURATION_DESCRIPTOR[] =
{
  /*Configuration Descriptor*/
  0x09,   /* bLength: Configuration Descriptor size */
  USB_DESC_TYPE_CONFIGURATION,      /* bDescriptorType: Configuration */
  9+9+7+7+7+7+9+9,                /* wTotalLength:no of returned bytes */
  0x00,
  0x02,   /* bNumInterfaces: 2 interface */
  0x01,   /* bConfigurationValue: Configuration value */
  0x04,   /* iConfiguration: Index of string descriptor describing the configuration */
  0xC0,   /* bmAttributes: self powered */
  0x32,   /* MaxPower 0 mA */
  
  /*---------------------------------------------------------------------------*/
  
  /*Interface Descriptor */
  0x09,   /* bLength: Interface Descriptor size */
  USB_DESC_TYPE_INTERFACE,  /* bDescriptorType: Interface */
  /* Interface descriptor type */
  0x00,   /* bInterfaceNumber: Number of Interface */
  0x00,   /* bAlternateSetting: Alternate setting */
  0x04,   /* bNumEndpoints: 4 endpoints used */
  0x00,   /* bInterfaceClass: 0 */
  0x00,   /* bInterfaceSubClass: 0 */
  0x00,   /* bInterfaceProtocol:0 */
  0x05,   /* iInterface: */
  
  // realtime interface
  /*Endpoint 2 Descriptor*/
  0x07,                           /* bLength: Endpoint Descriptor size */
  USB_DESC_TYPE_ENDPOINT,   /* bDescriptorType: Endpoint */
  0x82,                     /* bEndpointAddress */
  0x02,                           /* bmAttributes: Bulk */
  LOBYTE(USBD_BULK_SIZE),     /* wMaxPacketSize: */
  HIBYTE(USBD_BULK_SIZE),
  0x0,                           /* bInterval: */ 

    /*Endpoint 2 Descriptor*/
  0x07,                           /* bLength: Endpoint Descriptor size */
  USB_DESC_TYPE_ENDPOINT,   /* bDescriptorType: Endpoint */
  2,                     /* bEndpointAddress */
  0x02,                           /* bmAttributes: Bulk */
  LOBYTE(USBD_BULK_SIZE),     /* wMaxPacketSize: */
  HIBYTE(USBD_BULK_SIZE),
  0x0,                           /* bInterval: */ 
  /*---------------------------------------------------------------------------*/
  
  // text interface
  /*Endpoint 1 Descriptor*/
  0x07,                           /* bLength: Endpoint Descriptor size */
  USB_DESC_TYPE_ENDPOINT,   /* bDescriptorType: Endpoint */
  0x81,                     /* bEndpointAddress */
  0x02,                           /* bmAttributes: Bulk */
  LOBYTE(USBD_BULK_SIZE),     /* wMaxPacketSize: */
  HIBYTE(USBD_BULK_SIZE),
  0x10,                           /* bInterval: */ 

    /*Endpoint 1 Descriptor*/
  0x07,                           /* bLength: Endpoint Descriptor size */
  USB_DESC_TYPE_ENDPOINT,   /* bDescriptorType: Endpoint */
  0x01,                     /* bEndpointAddress */
  0x02,                           /* bmAttributes: Bulk */
  LOBYTE(USBD_BULK_SIZE),     /* wMaxPacketSize: */
  HIBYTE(USBD_BULK_SIZE),
  0x10,                           /* bInterval: */ 
  /*---------------------------------------------------------------------------*/

// DFU taken from the st dfu mode descriptor change interface protocol 2 to 1
  0x09,
  USB_DESC_TYPE_INTERFACE,
  DFU_INTERFACE_NUMBER,
  0x00,
  0x00,
  0xfe,
  0x01,
  0x01,
  0x06,

  0x09,
  0x21,
  0x0b, 
  0xff,
  0x00,
  0x00,
  0x08,
  0x1a,
  0x01
};

void USB_OTG::send_data32(uint8_t endpoint, const uint32_t *data, uint8_t length32, uint8_t length8) 
{
    if (!(USBx_INEP(endpoint)->DIEPCTL & USB_OTG_DIEPCTL_USBAEP)) {
        return;
    }
    if (endpoint != 0) {
        if (sending_) {
            // endpoint zero has priority, I don't know why but sending on another 
            // endpoint at the same time seems to break the system
            return;
        }
    }
    // if (sending_)
    //     asm("bkpt");
    sending_ = 1;
    if (length8 == 0) {
        length8 = length32*4;
    }
    if (endpoint != 0) {
        if (USBx_INEP(endpoint)->DIEPCTL & USB_OTG_DIEPCTL_EPENA) {  
            int count = 0;
            do {     
                USBx_INEP(endpoint)->DIEPCTL |= USB_OTG_DIEPCTL_SNAK;
                if (count++ > 100) {
                    if (USBx_INEP(endpoint)->DIEPCTL & USB_OTG_DIEPCTL_NAKSTS) {
                        break;
                    } else {
                        sending_ = 0;
                        return;
                    }
                } 
            } while(!(USBx_INEP(endpoint)->DIEPINT & USB_OTG_DIEPINT_INEPNE) && !(USBx_DEVICE->DSTS & USB_OTG_DSTS_SUSPSTS) && !(USBx_INEP(endpoint)->DIEPINT & USB_OTG_DIEPINT_EPDISD));
        while((USBx->GRSTCTL & USB_OTG_GRSTCTL_TXFFLSH) && !(USBx_DEVICE->DSTS & USB_OTG_DSTS_SUSPSTS) && !(USBx->GRSTCTL & USB_OTG_GRSTCTL_AHBIDL));
        USBx->GRSTCTL = ( USB_OTG_GRSTCTL_TXFFLSH |(uint32_t)( 1 << (USB_OTG_GRSTCTL_TXFNUM_Pos + endpoint - 1)));
        while((USBx->GRSTCTL & USB_OTG_GRSTCTL_TXFFLSH) && !(USBx_DEVICE->DSTS & USB_OTG_DSTS_SUSPSTS));     // wait on flush
        }
    } else {
        while(USBx_INEP(endpoint)->DIEPCTL & USB_OTG_DIEPCTL_EPENA) {
            // wait for previous transmission to complete
        }
    }

    USBx_INEP(endpoint)->DIEPTSIZ = 0;  // TODO necessary?
    USBx_INEP(endpoint)->DIEPTSIZ = length8 | (((length8-1)/64 + 1) << USB_OTG_DIEPTSIZ_PKTCNT_Pos);
    USBx_INEP(endpoint)->DIEPCTL |= USB_OTG_DIEPCTL_EPENA | USB_OTG_DIEPCTL_CNAK ;

    for(int i=0; i<length32; i++) {
        USBx_DFIFO(endpoint) = data[i]; 
    }

    sending_ = 0;
}

void USB_OTG::send_string(uint8_t endpoint, const char *str, uint8_t length) {
    uint16_t str_out[length+1] = {};
    uint8_t length_total = 2 + 2*length;
    str_out[0] = length_total | (3 << 8); // header
    for (int i=0; i<length; i++) {
        str_out[i+1] = str[i];
    }
    send_data(endpoint, reinterpret_cast<const uint8_t *>(str_out), length_total);
}

// This is the serial number used by the bootloader, 13 bytes with null terminator
void USB_OTG::Get_SerialNum(char * buffer)
{
    uint32_t deviceserial0, deviceserial1, deviceserial2;

    deviceserial0 = *(uint32_t *)DEVICE_ID1;
    deviceserial1 = *(uint32_t *)DEVICE_ID2;
    deviceserial2 = *(uint32_t *)DEVICE_ID3;

    deviceserial0 += deviceserial2;
    std::sprintf(buffer,"%lX%X",deviceserial0, (uint16_t) (deviceserial1>>16));
}

void USB_OTG::handle_setup_packet(uint8_t *setup_data) {
    switch(setup_data[0]) {
        case 0x80:  // standard request get
            switch (setup_data[1]) {
                case 0x00:  // get status
                    send_data(0, reinterpret_cast<const uint8_t *>("\x1\x0"), 2);  // self powered
                    break;
                case 0x06:  // get descriptor
                    switch (setup_data[3]) {
                        case 0x01:   // device descriptor
                            send_data(0, USB_DEVICE_DESCIPTOR, std::min(static_cast<size_t>(setup_data[6]),sizeof(USB_DEVICE_DESCIPTOR)));
                            break;
                        case 0x02:   // configuration descriptor
                            send_data(0, USB_CONFIGURATION_DESCRIPTOR, std::min(static_cast<size_t>(setup_data[6]),sizeof(USB_CONFIGURATION_DESCRIPTOR)));
                            break;
                        case 0x03:  // string descriptor
                            switch (setup_data[2]) {
                                case 0x00: // language descriptor
                                    send_data(0, reinterpret_cast<const uint8_t *>("\x4\x3\x9\x4"), 4); // english
                                    break;
                                case 0x01:
                                    send_string(0, MANUFACTURER_STRING, std::strlen(MANUFACTURER_STRING));
                                    break;
                                case 0x02:
                                    send_string(0, PRODUCT_STRING, std::strlen(PRODUCT_STRING));
                                    break;
                                case 0x03:
                                { 
                                    char sn_buffer[13];
                                    Get_SerialNum(sn_buffer);
                                    send_string(0, sn_buffer, std::strlen(sn_buffer));
                                    break;
                                }
                                case 0x04:
                                    send_string(0, VERSION " " GIT_HASH " " BUILD_DATETIME, std::strlen(VERSION " " GIT_HASH " " BUILD_DATETIME));
                                    break;
                                case 0x05:
                                    send_string(0, name, std::strlen(name));
                                    break;
                                case 0x06:
                                    send_string(0, "ST DFU mode", std::strlen("ST DFU mode"));
                                    break;
                                default:
                                    send_string(0, "default", std::strlen("default"));
                                    break;
                            }
                            break;
                        default: 
                            send_stall(0);
                            break;
                    }
                    break;
                default:
                    send_stall(0);
                    break;
            }
            break;
        case 0x00:  // standard request set
            switch (setup_data[1]) {
                case 0x05:  // set address
                    device_address_ = setup_data[2];
                    USBx_DEVICE->DCFG &= ~USB_OTG_DCFG_DAD; 
                    USBx_DEVICE->DCFG |= device_address_ << USB_OTG_DCFG_DAD_Pos;
                    send_data(0,0,0); // core seems to know to still send this as address 0
                    break;
                case 0x09: // set configuration
                    // enable endpoint 2 IN (TX)
                    USBx_DEVICE->DAINTMSK |= USB_OTG_DAINTMSK_IEPM & ((1U << (2)));
                    USBx_INEP(2)->DIEPCTL |= ((64 & USB_OTG_DIEPCTL_MPSIZ ) | (2 << 18U) |\
                        ((2) << USB_OTG_DIEPCTL_TXFNUM_Pos) | (USB_OTG_DIEPCTL_SD0PID_SEVNFRM) | (USB_OTG_DIEPCTL_USBAEP)); 
                    
                    // enable endpoint 2 OUT (RX)
                    USBx_DEVICE->DAINTMSK |= USB_OTG_DAINTMSK_OEPM & ((1U << (2)) << 16u);
                    USBx_OUTEP(2)->DOEPCTL |= ((64 & USB_OTG_DOEPCTL_MPSIZ ) | (2 << 18U) |\
                        (USB_OTG_DIEPCTL_SD0PID_SEVNFRM)| (USB_OTG_DOEPCTL_USBAEP));
                    USBx_OUTEP(2)->DOEPTSIZ = 0x80040; 
                    USBx_OUTEP(2)->DOEPCTL |= USB_OTG_DOEPCTL_EPENA | USB_OTG_DOEPCTL_CNAK ;

                    USBx_DEVICE->DAINTMSK |= USB_OTG_DAINTMSK_IEPM & ((1U << (1)));
                    USBx_INEP(1)->DIEPCTL |= ((64 & USB_OTG_DIEPCTL_MPSIZ ) | (2 << 18U) |\
                        ((1) << USB_OTG_DIEPCTL_TXFNUM_Pos) | (USB_OTG_DIEPCTL_SD0PID_SEVNFRM) | (USB_OTG_DIEPCTL_USBAEP)); 
                    
                    // enable endpoint 2 OUT (RX)
                    USBx_DEVICE->DAINTMSK |= USB_OTG_DAINTMSK_OEPM & ((1U << (1)) << 16u);
                    USBx_OUTEP(1)->DOEPCTL |= ((64 & USB_OTG_DOEPCTL_MPSIZ ) | (2 << 18U) |\
                        (USB_OTG_DIEPCTL_SD0PID_SEVNFRM)| (USB_OTG_DOEPCTL_USBAEP));
                    USBx_OUTEP(1)->DOEPTSIZ = 0x80040; 
                    USBx_OUTEP(1)->DOEPCTL |= USB_OTG_DOEPCTL_EPENA | USB_OTG_DOEPCTL_CNAK ;
                    // setup status phase    
                    send_data(0,0,0);
                    break;
                default:
                    send_stall(0);
                    break;
            }
            break;
        case 0x01:  // interface request set
            if (setup_data[1] == 11) { // set inteface request
                interface_ = setup_data[4];
                send_data(0,0,0);
            } else {
                send_stall(0);
            }
            break;
        case 0xa1:  // interface class get
            if ((setup_data[1] == 3) && (interface_ == 1)) { // dfu get_status
                send_data(0,reinterpret_cast<const uint8_t *>("\x00\x00\x00\x00\x00\x00"), 6);
            } else {
                send_stall(0);
            }
            break;
        case 0x21:  // interface class request
            if ((setup_data[1] == 0) && (interface_ == 1)) { // dfu detach
                send_data(0,0,0);
                ms_delay(10);
                reboot_to_bootloader();
            } else {
                send_stall(0);
            }
            break;
        default:
            send_stall(0);
            break;
    }
    USBx_OUTEP(0U)->DOEPTSIZ = 0U;
    USBx_OUTEP(0U)->DOEPTSIZ |= (USB_OTG_DOEPTSIZ_PKTCNT & (1U << 19U)) ;
    USBx_OUTEP(0U)->DOEPTSIZ |= (3U * 8U);
    USBx_OUTEP(0U)->DOEPTSIZ |=  USB_OTG_DOEPTSIZ_STUPCNT;  
    USBx_OUTEP(0)->DOEPCTL |= USB_OTG_DOEPCTL_CNAK | USB_OTG_DOEPCTL_EPENA;
}