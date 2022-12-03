# nucleo h743zi2 - FDCAN prototyping

## Saturday, Sept 03, 2022
- Yong Da work session
- copied most of the magic configuration from Reid
  - https://github.com/spacesys-finch/pay-elec-testing/tree/main/testCAN
- NEW CONTRIBUTION: added the RX functionality to be interrupt based


### usage
equipment:
- 2x H7 devkits
- 2x FDCAN transceivers (TI1044)
  - https://www.ti.com/lit/ds/symlink/tcan1044v-q1.pdf?ts=1662221522598
  - should be in the box called "FDCAN projects", maybe on top of the Banting cabinet
- laptop with 2 USB ports (to monitor UART of both devkits)
- bunch of jumper wires

connectivity:
- connect 5V to VDD on FDCAN module
- connect 3.3V to Vio on FDCAN module
- connect CANH <-> CANH and CANL <-> CANL on the 2x FDCAN modules
- connect FDCAN to MCU TX<->TX and RX<->RX
  - if you used default board configuration, these should be on the lower-left (CN9) on the board

code:
- RX mode is completely done with interrupts, so don't need anything in the main for receiving messages
- TX just puts a message in the TxBuffer, which initiatives the message to be sent
- program 1x MCU with the unmodified program (containing the TX code)
- then comment out the TX super loop (aka only RX interrupt functionaltiy is remaining)
    - and program the 2nd MCU with this modified program

 
### RAM configuration
- if the FDCAN errors out due to something about RAM not allocated, you need to manually setup the RX and TX FIFO buffers in the IOC file
- see section 4.2 - RAM management and 4.3 - RAM sections of AN5348
  - https://www.st.com/resource/en/application_note/an5348-fdcan-peripheral-on-stm32-devices-stmicroelectronics.pdf

- stuff like the below
```
  hfdcan1.Init.RxFifo0ElmtsNbr = 32;
  hfdcan1.Init.RxFifo0ElmtSize = FDCAN_DATA_BYTES_8;
  hfdcan1.Init.RxFifo1ElmtsNbr = 32;
  hfdcan1.Init.RxFifo1ElmtSize = FDCAN_DATA_BYTES_8;
  hfdcan1.Init.RxBuffersNbr = 1;
  hfdcan1.Init.RxBufferSize = FDCAN_DATA_BYTES_8;
  hfdcan1.Init.TxEventsNbr = 4;
  hfdcan1.Init.TxBuffersNbr = 4;
  hfdcan1.Init.TxFifoQueueElmtsNbr = 32;
  hfdcan1.Init.TxFifoQueueMode = FDCAN_TX_FIFO_OPERATION;
  hfdcan1.Init.TxElmtSize = FDCAN_DATA_BYTES_8;
  ```

### RX filter configuration
- see section 4.3.1 of AN5348 for details
  - https://www.st.com/resource/en/application_note/an5348-fdcan-peripheral-on-stm32-devices-stmicroelectronics.pdf
- could not get the buffer mode working
- so just only stick to the FIFO mode
- requires 2 IDs
	- first ID is the message ID
	- second ID is the mask --> aka if it's a 1, then use the those bits
- can also setup the filter using a range or exact match
	- Reid just happened to use this filter mask mode
```  
/* reception filter for Rx FIFO 0 */
  sFilterConfig.IdType = FDCAN_STANDARD_ID;
  sFilterConfig.FilterIndex = 0;
  sFilterConfig.FilterType = FDCAN_FILTER_MASK;
  sFilterConfig.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
  sFilterConfig.FilterID1 = 0x111;	// since all filterID2 is set to 1's, then must be exact match on filterID1
  sFilterConfig.FilterID2 = 0x7FF;
  if (HAL_FDCAN_ConfigFilter(&hfdcan1, &sFilterConfig) != HAL_OK){
	  Error_Handler();
```

### interrupt mode
- see section 31.2.1 - FDCAN Firmware driver API description, how to use this driver from UM2217 rev5
  - https://www.st.com/resource/en/user_manual/um2217-description-of-stm32h7-hal-and-lowlayer-drivers-stmicroelectronics.pdf

---
_start excerpt_

Polling mode operation
1. Reception and transmission states can be monitored via the following functions:
– HAL_FDCAN_IsRxBufferMessageAvailable
– HAL_FDCAN_IsTxBufferMessagePending
– HAL_FDCAN_GetRxFifoFillLevel
– HAL_FDCAN_GetTxFifoFreeLevel

Interrupt mode operation
1. There are two interrupt lines: line 0 and 1. By default, all interrupts are assigned to line 0. Interrupt lines can
be configured using HAL_FDCAN_ConfigInterruptLines function.
2. Notifications are activated using HAL_FDCAN_ActivateNotification function. Then, the process can be
controlled through one of the available user callbacks: HAL_FDCAN_xxxCallback

_end excerpt_

---

- need to enable FDCAN1 interrupt 0 in the NVIC settings on the FDCAN1 IOC page
  - don't need interrupt 1 for this, that's for other FDCAN functionality (I'm not exactly sure what it's for)
- need to put these lines BEFORE you start the FDCAN module:
```
  /* Configure Rx FIFO 0 watermark to 2 */
  HAL_FDCAN_ConfigFifoWatermark(&hfdcan1, FDCAN_CFG_RX_FIFO0, 2);

  /* Activate Rx FIFO 0 watermark notification */
  HAL_FDCAN_ActivateNotification(&hfdcan1, FDCAN_IT_RX_FIFO0_WATERMARK, 0);
```

- in main.c, you need to write your own `void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs)`
  - this function is actually defined in `stm32h7xx_hal_fdcan.h` as a `__weak` function
  - meaning if it's defined somewhere else, the compile is gonna take the non-weak definition
  - otherwise, it uses the weak one

```
void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs){
  if((RxFifo0ITs & FDCAN_IT_RX_FIFO0_WATERMARK) == RESET){
    Error_Handler();
  }

  printf("hit the callback RXFifo0\n\r");

  // https://overiq.com/c-programming-101/local-global-and-static-variables-in-c/
  static int count = 0; // retain the variable value between different function calls

  /* Retrieve Rx message from RX FIFO0 */
  if (HAL_FDCAN_GetRxMessage(&hfdcan1, FDCAN_RX_FIFO0, &RxHeader, RxData) != HAL_OK){
    Error_Handler();
  }

  printf("count=%3d, received RxData[0]=%3d, RxData[1]=%d\r\n", count, RxData[0], RxData[1]);
  count++;
}
```
