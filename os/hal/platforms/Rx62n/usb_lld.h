/*
    ChibiOS/RT - Copyright (C) 2006-2013 Giovanni Di Sirio

    Licensed under the Apache License, Version 2.0 (the "License");
    you may not use this file except in compliance with the License.
    You may obtain a copy of the License at

        http://www.apache.org/licenses/LICENSE-2.0

    Unless required by applicable law or agreed to in writing, software
    distributed under the License is distributed on an "AS IS" BASIS,
    WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
    See the License for the specific language governing permissions and
    limitations under the License.
*/

/**
 * @file    RX62N/usb_lld.h
 * @brief   RX62N USB Driver subsystem low level driver header.
 *
 * @addtogroup USB
 * @{
 */

#ifndef _USB_LLD_H_
#define _USB_LLD_H_

#if HAL_USE_USB || defined(__DOXYGEN__)

#include "iodefine_gcc62n.h"

/*===========================================================================*/
/* Driver constants.                                                         */
/*===========================================================================*/

/**
 * @brief   Maximum endpoint address.
 * @details This value does not include the endpoint 0 which is always present.
 */
#define USB_MAX_ENDPOINTS                   9

/**
 * @brief   Maximum endpoint 0 size.
 */
#define USB_EP0_PACKET_SIZE                 64 /* other values doesn't work ? */

/**
 * @brief   Status stage handling method.
 */
#define USB_EP0_STATUS_STAGE                USB_EP0_STATUS_STAGE_HW

/**
 * @brief   SET_ADDRESS ack handling method.
 */
#define USB_SET_ADDRESS_ACK_HANDLING        USB_SET_ADDRESS_ACK_HW

/**
 * @brief   This device requires the address change after the status packet.
 */
#define USB_SET_ADDRESS_MODE                USB_EARLY_SET_ADDRESS

/**
 * @brief   INTSTS0 bit values.
 */
#define INTSTS0_VBINT                       (1<<15)
#define INTSTS0_RESM                        (1<<14)
#define INTSTS0_SOFR                        (1<<13)
#define INTSTS0_DVST                        (1<<12)
#define INTSTS0_CTRT                        (1<<11)
#define INTSTS0_BEMP                        (1<<10)
#define INTSTS0_NRDY                        (1<<9)
#define INTSTS0_BRDY                        (1<<8)
#define INTSTS0_VBSTS                       (1<<7)
#define INTSTS0_VALID                       (1<<3)
#define INTSTS0_DVSQ(x)                     ((x>>4)&7)
#define INTSTS0_CTSQ(x)                     (x&7)

/**
 * @brief   INTENB0 bit values.
 */
#define INTENB0_VBSE                        (1<<15)
#define INTENB0_RSME                        (1<<14)
#define INTENB0_SOFE                        (1<<13)
#define INTENB0_DVSE                        (1<<12)
#define INTENB0_CTRE                        (1<<11)
#define INTENB0_BEMPE                       (1<<10)
#define INTENB0_NRDYE                       (1<<9)
#define INTENB0_BRDYE                       (1<<8)

/**
 * @brief   PID values.
 */
#define PID_NAK                             0
#define PID_BUF                             1
#define PID_STALL10                         2
#define PID_STALL11                         3
#define PID_MASK                            3

/**
 * @brief   CTSQ state values.
 */
#define CTSQ_IDLE_OR_SETUP                  0
#define CTSQ_READ_DATA                      1
#define CTSQ_READ_STATUS                    2
#define CTSQ_WRITE_DATA                     3
#define CTSQ_WRITE_STATUS                   4
#define CTSQ_NODATA_STATUS                  5
#define CTSQ_SEQ_ERROR                      6

/**
 * @brief   DVSQ state values.
 */
#define DVSQ_POWERED                        0
#define DVSQ_DEFAULT                        1
#define DVSQ_ADDRESS                        2
#define DVSQ_CONFIGURED                     3
#define DVSQ_SUSPENDED_PWR                  4
#define DVSQ_SUSPENDED_DEF                  5
#define DVSQ_SUSPENDED_ADR                  6
#define DVSQ_SUSPENDED_CFG                  7

/**
 * @brief   PIPECFG bit values.
 */
#define PIPECFG_TYPE_BULK                   (1<<14)
#define PIPECFG_TYPE_INT                    (2<<14)
#define PIPECFG_TYPE_ISO                    (3<<14)
#define PIPECFG_DIR_OUT                     (0<<4)
#define PIPECFG_DIR_IN                      (1<<4)

/**
 * @brief   PIPEnCTR bit values.
 */
#define PIPECTR_SQSET                       (1<<7)
#define PIPECTR_SQCLR                       (1<<8)
#define PIPECTR_ACLRM                       (1<<9)
#define PIPECTR_ATREPM                      (1<<10)

/*===========================================================================*/
/* Driver pre-compile time settings.                                         */
/*===========================================================================*/

/*===========================================================================*/
/* Derived constants and error checks.                                       */
/*===========================================================================*/

/*===========================================================================*/
/* Driver data structures and types.                                         */
/*===========================================================================*/

/**
 * @brief   Type of an IN endpoint state structure.
 */
typedef struct {
  /**
   * @brief   Buffer mode, queue or linear.
   */
  bool_t                        txqueued;
  /**
   * @brief   Requested transmit transfer size.
   */
  size_t                        txsize;
  /**
   * @brief   Transmitted bytes so far.
   */
  size_t                        txcnt;
  /**
   * @brief   Transmitted bytes in last transaction (interrupt).
   */
  size_t                        transmitted;
  union {
    struct {
      /**
       * @brief   Pointer to the transmission linear buffer.
       */
      const uint8_t             *txbuf;
    } linear;
    struct {
      /**
       * @brief   Pointer to the output queue.
       */
      OutputQueue               *txqueue;
    } queue;
    /* End of the mandatory fields.*/
  } mode;
} USBInEndpointState;

/**
 * @brief   Type of an OUT endpoint state structure.
 */
typedef struct {
  /**
   * @brief   Buffer mode, queue or linear.
   */
  bool_t                        rxqueued;
  /**
   * @brief   Requested receive transfer size.
   */
  size_t                        rxsize;
  /**
   * @brief   Received bytes so far.
   */
  size_t                        rxcnt;
  union {
    struct {
      /**
       * @brief   Pointer to the receive linear buffer.
       */
      uint8_t                   *rxbuf;
    } linear;
    struct {
      /**
       * @brief   Pointer to the input queue.
       */
      InputQueue               *rxqueue;
    } queue;
  } mode;
  /* End of the mandatory fields.*/
  /**
   * @brief   Number of packets to receive.
   */
  uint16_t                      rxpkts;
  /**
   * @brief   Number of bytes last received.
   */
  uint16_t                      rxlast;
} USBOutEndpointState;

/**
 * @brief   Type of an USB endpoint configuration structure.
 * @note    Platform specific restrictions may apply to endpoints.
 */
typedef struct {
  /**
   * @brief   Type and mode of the endpoint.
   */
  uint32_t                      ep_mode;
  /**
   * @brief   Setup packet notification callback.
   * @details This callback is invoked when a setup packet has been
   *          received.
   * @post    The application must immediately call @p usbReadPacket() in
   *          order to access the received packet.
   * @note    This field is only valid for @p USB_EP_MODE_TYPE_CTRL
   *          endpoints, it should be set to @p NULL for other endpoint
   *          types.
   */
  usbepcallback_t               setup_cb;
  /**
   * @brief   IN endpoint notification callback.
   * @details This field must be set to @p NULL if the IN endpoint is not
   *          used.
   */
  usbepcallback_t               in_cb;
  /**
   * @brief   OUT endpoint notification callback.
   * @details This field must be set to @p NULL if the OUT endpoint is not
   *          used.
   */
  usbepcallback_t               out_cb;
  /**
   * @brief   IN endpoint maximum packet size.
   * @details This field must be set to zero if the IN endpoint is not
   *          used.
   */
  uint16_t                      in_maxsize;
  /**
   * @brief   OUT endpoint maximum packet size.
   * @details This field must be set to zero if the OUT endpoint is not
   *          used.
   */
  uint16_t                      out_maxsize;
  /**
   * @brief   @p USBEndpointState associated to the IN endpoint.
   * @details This structure maintains the state of the IN endpoint.
   */
  USBInEndpointState            *in_state;
  /**
   * @brief   @p USBEndpointState associated to the OUT endpoint.
   * @details This structure maintains the state of the OUT endpoint.
   */
  USBOutEndpointState           *out_state;
  /* End of the mandatory fields.*/
  /**
   * @brief   Reserved field, not currently used.
   * @note    Initialize this field to 1 in order to be forward compatible.
   */
  uint16_t                      ep_buffers;
  /**
   * @brief   Pointer to a buffer for setup packets.
   * @details Setup packets require a dedicated 8-bytes buffer, set this
   *          field to @p NULL for non-control endpoints.
   */
  uint8_t                       *setup_buf;
} USBEndpointConfig;

/**
 * @brief   Type of an USB driver configuration structure.
 */
typedef struct {
  /**
   * @brief   USB events callback.
   * @details This callback is invoked when an USB driver event is registered.
   */
  usbeventcb_t                  event_cb;
  /**
   * @brief   Device GET_DESCRIPTOR request callback.
   * @note    This callback is mandatory and cannot be set to @p NULL.
   */
  usbgetdescriptor_t            get_descriptor_cb;
  /**
   * @brief   Requests hook callback.
   * @details This hook allows to be notified of standard requests or to
   *          handle non standard requests.
   */
  usbreqhandler_t               requests_hook_cb;
  /**
   * @brief   Start Of Frame callback.
   */
  usbcallback_t                 sof_cb;
  /* End of the mandatory fields.*/
} USBConfig;

/**
 * @brief   Structure representing an USB driver.
 */
struct USBDriver {
  /**
   * @brief   Driver state.
   */
  usbstate_t                    state;
  /**
   * @brief   Current configuration data.
   */
  const USBConfig               *config;
  /**
   * @brief   Field available to user, it can be used to associate an
   *          application-defined handler to the USB driver.
   */
  void                          *param;
  /**
   * @brief   Bit map of the transmitting IN endpoints.
   */
  uint16_t                      transmitting;
  /**
   * @brief   Bit map of the receiving OUT endpoints.
   */
  uint16_t                      receiving;
  /**
   * @brief   Active endpoints configurations.
   */
  const USBEndpointConfig       *epc[USB_MAX_ENDPOINTS + 1];
  /**
   * @brief   Fields available to user, it can be used to associate an
   *          application-defined handler to an IN endpoint.
   * @note    The base index is one, the endpoint zero does not have a
   *          reserved element in this array.
   */
  void                          *in_params[USB_MAX_ENDPOINTS];
  /**
   * @brief   Fields available to user, it can be used to associate an
   *          application-defined handler to an OUT endpoint.
   * @note    The base index is one, the endpoint zero does not have a
   *          reserved element in this array.
   */
  void                          *out_params[USB_MAX_ENDPOINTS];
  /**
   * @brief   Endpoint 0 state.
   */
  usbep0state_t                 ep0state;
  /**
   * @brief   Next position in the buffer to be transferred through endpoint 0.
   */
  uint8_t                       *ep0next;
  /**
   * @brief   Number of bytes yet to be transferred through endpoint 0.
   */
  size_t                        ep0n;
  /**
   * @brief   Endpoint 0 end transaction callback.
   */
  usbcallback_t                 ep0endcb;
  /**
   * @brief   Setup packet buffer.
   */
  uint8_t                       setup[8];
  /**
   * @brief   Current USB device status.
   */
  uint16_t                      status;
  /**
   * @brief   Assigned USB address.
   */
  uint8_t                       address;
  /**
   * @brief   Current USB device configuration.
   */
  uint8_t                       configuration;
#if defined(USB_DRIVER_EXT_FIELDS)
  USB_DRIVER_EXT_FIELDS
#endif
  /* End of the mandatory fields.*/
  /**
   * @brief   USB hardware registers pointer.
   */
  volatile struct st_usb0       *usb;
  /**
   * @brief   Last USB device status.
   */
  uint8_t                       last_dvst;
  /**
   * @brief   Last USB control stage.
   */
  uint8_t                       last_ctsq;
  /**
   * @brief   Current FIFO number (D0 or D1).
   */
  uint8_t                       current_fifo;
};

/*===========================================================================*/
/* Driver macros.                                                            */
/*===========================================================================*/

/**
 * @brief   Returns the current frame number.
 *
 * @param[in] usbp      pointer to the @p USBDriver object
 * @return              The current frame number.
 *
 * @notapi
 */
#define usb_lld_get_frame_number(usbp) ((usbp)->usb->FRMNUM.BIT.FRNM)

/**
 * @brief   Returns the exact size of a receive transaction.
 * @details The received size can be different from the size specified in
 *          @p usbStartReceiveI() because the last packet could have a size
 *          different from the expected one.
 * @pre     The OUT endpoint must have been configured in transaction mode
 *          in order to use this function.
 *
 * @param[in] usbp      pointer to the @p USBDriver object
 * @param[in] ep        endpoint number
 * @return              Received data size.
 *
 * @notapi
 */
#define usb_lld_get_transaction_size(usbp, ep)                              \
  ((usbp)->epc[ep]->out_state->rxcnt)

/**
 * @brief   Triggers status stage for control endpoint.
 *
 * @param[in] usbp      pointer to the @p USBDriver object
 * @param[in] ep        endpoint number
 *
 * @notapi
 */
#define usb_lld_end_setup(usbp, ep)                                         \
  ((usbp)->usb->DCPCTR.BIT.CCPL=1)

/**
 * @brief   Returns the exact size of a received packet.
 * @pre     The OUT endpoint must have been configured in packet mode
 *          in order to use this function.
 *
 * @param[in] usbp      pointer to the @p USBDriver object
 * @param[in] ep        endpoint number
 * @return              Received data size.
 *
 * @notapi
 */
#define  usb_lld_get_packet_size(usbp, ep)                                  \
  ((size_t)((usbp)->epc[ep]->out_state->rxlast)


/**
 * @brief   Connects the USB device.
 *
 * @api
 */
#define usb_lld_connect_bus(usbp) ((usbp)->usb->SYSCFG.BIT.DPRPU=1)

/**
 * @brief   Disconnect the USB device.
 *
 * @api
 */
#define usb_lld_disconnect_bus(usbp) ((usbp)->usb->SYSCFG.BIT.DPRPU=0)

/*===========================================================================*/
/* External declarations.                                                    */
/*===========================================================================*/

#if RX62N_USE_USB0 || defined(__DOXYGEN__)
extern USBDriver USBD1;
#endif

#if RX62N_USE_USB1 || defined(__DOXYGEN__)
extern USBDriver USBD2;
#endif

#ifdef __cplusplus
extern "C" {
#endif
  void usb_lld_init(void);
  void usb_lld_start(USBDriver *usbp);
  void usb_lld_stop(USBDriver *usbp);
  void usb_lld_reset(USBDriver *usbp);
  void usb_lld_set_address(USBDriver *usbp);
  void usb_lld_end_configuration(USBDriver *usbp);
  void usb_lld_init_endpoint(USBDriver *usbp, usbep_t ep);
  void usb_lld_disable_endpoints(USBDriver *usbp);
  usbepstatus_t usb_lld_get_status_in(USBDriver *usbp, usbep_t ep);
  usbepstatus_t usb_lld_get_status_out(USBDriver *usbp, usbep_t ep);
  void usb_lld_read_setup(USBDriver *usbp, usbep_t ep, uint8_t *buf);
  void usb_lld_prepare_receive(USBDriver *usbp, usbep_t ep);
  void usb_lld_prepare_transmit(USBDriver *usbp, usbep_t ep);
  void usb_lld_start_out(USBDriver *usbp, usbep_t ep);
  void usb_lld_start_in(USBDriver *usbp, usbep_t ep);
  void usb_lld_stall_out(USBDriver *usbp, usbep_t ep);
  void usb_lld_stall_in(USBDriver *usbp, usbep_t ep);
  void usb_lld_clear_out(USBDriver *usbp, usbep_t ep);
  void usb_lld_clear_in(USBDriver *usbp, usbep_t ep);
#ifdef __cplusplus
}
#endif

#endif /* HAL_USE_USB */

#endif /* _USB_LLD_H_ */

/** @} */
