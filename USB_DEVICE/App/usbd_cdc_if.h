#ifndef BALLCONTROL_USBD_CDC_IF_H
#define BALLCONTROL_USBD_CDC_IF_H

#include "usbd_cdc.h"

extern USBD_CDC_ItfTypeDef USBD_Interface_fops_FS;

uint8_t CDC_Transmit_FS(uint8_t *buffer, uint16_t length);

#endif
