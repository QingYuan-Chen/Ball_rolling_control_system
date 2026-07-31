#include "usbd_cdc_if.h"

#include "ball_control_app.h"
#include "usb_device.h"

#define USB_CDC_RX_BUFFER_SIZE 64U
#define USB_CDC_LINE_CODING_SIZE 7U

static uint8_t g_usb_rx_buffer[USB_CDC_RX_BUFFER_SIZE];
static uint8_t g_line_coding[USB_CDC_LINE_CODING_SIZE] = {
    0x00U, 0x08U, 0x07U, 0x00U, 0x00U, 0x00U, 0x08U
};

static int8_t CDC_Init_FS(void)
{
    USBD_CDC_SetTxBuffer(&hUsbDeviceFS, NULL, 0U);
    USBD_CDC_SetRxBuffer(&hUsbDeviceFS, g_usb_rx_buffer);
    BallControl_UsbCdcSetConnected(true);
    return (int8_t) USBD_OK;
}

static int8_t CDC_DeInit_FS(void)
{
    BallControl_UsbCdcSetConnected(false);
    return (int8_t) USBD_OK;
}

static int8_t CDC_Control_FS(uint8_t command, uint8_t *buffer,
                             uint16_t length)
{
    if ((buffer == NULL) && (length != 0U)) {
        return (int8_t) USBD_FAIL;
    }

    if ((command == CDC_SET_LINE_CODING) &&
        (length >= USB_CDC_LINE_CODING_SIZE)) {
        for (uint32_t i = 0U; i < USB_CDC_LINE_CODING_SIZE; ++i) {
            g_line_coding[i] = buffer[i];
        }
    } else if ((command == CDC_GET_LINE_CODING) &&
               (length >= USB_CDC_LINE_CODING_SIZE)) {
        for (uint32_t i = 0U; i < USB_CDC_LINE_CODING_SIZE; ++i) {
            buffer[i] = g_line_coding[i];
        }
    }
    return (int8_t) USBD_OK;
}

static int8_t CDC_Receive_FS(uint8_t *buffer, uint32_t *length)
{
    if ((buffer != NULL) && (length != NULL)) {
        BallControl_UsbCdcReceive(buffer, *length);
    }
    USBD_CDC_SetRxBuffer(&hUsbDeviceFS, g_usb_rx_buffer);
    if (USBD_CDC_ReceivePacket(&hUsbDeviceFS) != USBD_OK) {
        return (int8_t) USBD_FAIL;
    }
    return (int8_t) USBD_OK;
}

static int8_t CDC_TransmitCplt_FS(uint8_t *buffer, uint32_t *length,
                                  uint8_t endpoint)
{
    (void) buffer;
    (void) length;
    (void) endpoint;
    return (int8_t) USBD_OK;
}

USBD_CDC_ItfTypeDef USBD_Interface_fops_FS = {
    CDC_Init_FS,
    CDC_DeInit_FS,
    CDC_Control_FS,
    CDC_Receive_FS,
    CDC_TransmitCplt_FS
};

uint8_t CDC_Transmit_FS(uint8_t *buffer, uint16_t length)
{
    USBD_CDC_HandleTypeDef *cdc =
        (USBD_CDC_HandleTypeDef *) hUsbDeviceFS.pClassData;

    if ((cdc == NULL) || (cdc->TxState != 0U)) {
        return USBD_BUSY;
    }
    USBD_CDC_SetTxBuffer(&hUsbDeviceFS, buffer, length);
    return USBD_CDC_TransmitPacket(&hUsbDeviceFS);
}
