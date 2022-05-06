/*******************************************************************************
 *				 _ _                                             _ _
				|   |                                           (_ _)
				|   |        _ _     _ _   _ _ _ _ _ _ _ _ _ _   _ _
				|   |       |   |   |   | |    _ _     _ _    | |   |
				|   |       |   |   |   | |   |   |   |   |   | |   |
				|   |       |   |   |   | |   |   |   |   |   | |   |
				|   |_ _ _  |   |_ _|   | |   |   |   |   |   | |   |
				|_ _ _ _ _| |_ _ _ _ _ _| |_ _|   |_ _|   |_ _| |_ _|
								(C)2021 Lumi
 * Copyright (c) 2021
 * Lumi, JSC.
 * All Rights Reserved
 *
 * File name: lumi-types.h
 *
 * Description: This code is used for tranning Lumi IOT member. It is the code form statandard.
 * This is main function in application layer.
 *
 * Author: PhuongNP
 *
 * Last Changed By:  $Author: phuongnp $
 * Revision:         $Revision: $
 * Last Changed:     $Date: $Dec 12, 2021
 *
 * Code sample:
 ******************************************************************************/
#ifndef LUMI_TYPES_H_
#define LUMI_TYPES_H_
/******************************************************************************/
/*                              INCLUDE FILES                                 */
/******************************************************************************/
#include "typedefs.h"
/******************************************************************************/
/*                     EXPORTED TYPES and DEFINITIONS                         */
/******************************************************************************/
/**
 * @brief Defines the possible types of nodes and the roles that a
 * node might play in a network.
 */
typedef u8_t LumiNodeType;
enum
{
  /** The device is not joined. */
  LUMI_UNKNOWN_DEVICE = 0,
  /** Will relay messages and can act as a parent to other nodes. */
  LUMI_COORDINATOR = 1,
  /** Will relay messages and can act as a parent to other nodes. */
  LUMI_ROUTER = 2,
  /** Communicates only with its parent and will not relay messages. */
  LUMI_END_DEVICE = 3,
  /** An end device whose radio can be turned off to save power.
   *  The application must call ::emberPollForData() to receive messages.
   */
  LUMI_SLEEPY_END_DEVICE = 4,
};
/**
 * @brief  Return type for Lumi functions.
 */
#ifndef __LUMISTATUS_TYPE__
#define __LUMISTATUS_TYPE__
typedef u8_t LumiStatus;
#endif // __LUMISTATUS_TYPE__
enum{
	/**
	* @brief The generic "no error" message.
	*/
	LUMI_SUCCESS = 0x00,

	/**
	* @brief The generic "fatal error" message.
	*/
	LUMI_FAILURE = 0x01,

	/**
	* @brief An invalid value was passed as an argument to a function.
	*/
	LUMI_BAD_ARGUMENT = 0x02,

	/**
	* @brief The requested information was not found.
	*/
	LUMI_NOT_FOUND = 0x03,

	/**
	 * @brief Tried to send too much data.
	 */
	LUMI_SERIAL_TX_OVERFLOW = 0x22,

	/**
	* @brief There wasn't enough space to store a received character
	* and the character was dropped.
	*/
	LUMI_SERIAL_RX_OVERFLOW = 0x23,

	/**
	* @brief There is no received data to process.
	*/
	LUMI_SERIAL_RX_EMPTY = 0x26,

	/**
	* @brief The receive interrupt was not handled in time and a
	* character was dropped.
	*/
	LUMI_SERIAL_RX_OVERRUN_ERROR = 0x27,

	/**
	* @brief The stack software has completed initialization and is ready
	* to send and receive packets over the air.
	*/
	LUMI_NETWORK_UP = 0x90,

	/**
	* @brief The network is not operating.
	*/
	LUMI_NETWORK_DOWN = 0x91,

	/**
	* @brief An attempt to join a network failed.
	*/
	LUMI_JOIN_FAILED = 0x94,

	/**
	* @brief After moving, a mobile node's attempt to re-establish contact
	* with the network failed.
	*/
	LUMI_MOVE_FAILED = 0x96,
};

/**
 * @brief A distinguished manufacturer code that is used to indicate the
 * absence of a manufacturer-specific profile, cluster, command, or attribute.
 */
#define LUMI_AF_NULL_MANUFACTURER_CODE 0x0000
/**
 * @brief 16-bit ZigBee network address.
 */
typedef u16_t LumiNodeId;
/**
 * @brief Size of EUI64 (an IEEE address) in bytes (8).
 */
#define EUI64_SIZE 8
/**
 * @brief The maximum channels per page are 27
 * page bits 31...27, channel bits 26...0.
 */
#define LUMI_MAX_CHANNELS_PER_PAGE   27
/**
 * @brief EUI 64-bit ID (an IEEE address).
 */
typedef u8_t LumiEUI64[EUI64_SIZE];

#define LUMI_NULL_U8 				(0xFF)
#define LUMI_NULL_U16 				(0xFFFF)
#define LUMI_NULL_EUI64 			{0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF}
/**
 * @brief A data structure used to describe the time in a human
 * understandable format (as opposed to 32-bit UTC)
 */

typedef struct {
  u16_t year;
  u8_t month;
  u8_t day;
  u8_t hours;
  u8_t minutes;
  u8_t seconds;
} LumiAfTimeStruct;

/**
 * @brief A data structure used to describe the ZCL Date data type
 */

typedef struct {
  u8_t year;
  u8_t month;
  u8_t dayOfMonth;
  u8_t dayOfWeek;
} LumiAfDate;

/** @brief Complete events with a control and a handler procedure.
 *
 * An application typically creates an array of events
 * along with their handlers.
 * The main loop passes the array to ::emberRunEvents() to call
 * the handlers of any events whose time has arrived.
 */
//typedef const struct {
//  /** The control structure for the event. */
//  EmberEventControl *control;
//  /** The procedure to call when the event fires. */
//  EmberEventHandler handler;//void (*handler)(void);
//} EmberEventData;

/**
 * @brief Type for referring to ZCL attribute id
 */
typedef u16_t LumiAfAttributeId;

/**
 * @brief Type for referring to ZCL cluster id
 */
typedef u16_t LumiAfClusterId;

/**
 * @brief Type for referring to ZCL attribute type
 */
typedef u8_t LumiAfAttributeType;

/**
 * @name ZigBee Broadcast Addresses
 *@{
 *  ZigBee specifies three different broadcast addresses that
 *  reach different collections of nodes.  Broadcasts are normally sent only
 *  to routers.  Broadcasts can also be forwarded to end devices, either
 *  all of them or only those that do not sleep.  Broadcasting to end
 *  devices is both significantly more resource-intensive and significantly
 *  less reliable than broadcasting to routers.
 */

/** Broadcast to all routers. */
#define LUMI_BROADCAST_ADDRESS 0xFFFC
/** Broadcast to all non-sleepy devices. */
#define LUMI_RX_ON_WHEN_IDLE_BROADCAST_ADDRESS 0xFFFD
/** Broadcast to all devices, including sleepy end devices. */
#define LUMI_SLEEPY_BROADCAST_ADDRESS 0xFFFF

/** @} END Broadcast Addresses */

// From table 3.51 of 053474r14
// When sending many-to-one route requests, the following
// addresses are used
// 0xFFF9 indicates a non-memory-constrained many-to-one route request
// 0xFFF8 indicates a memory-constrained many-to-one route request
#define LUMI_MIN_BROADCAST_ADDRESS 0xFFF8

/**
 * @brief Returns true if nodeId (short address) is a broadcast address
 */
#define lumiIsZigbeeBroadcastAddress(address) \
  (LUMI_MIN_BROADCAST_ADDRESS <= ((u16_t) (address)))

/**
 * Lumi OTA image type define
 */
enum{
	//Define touch switch
	LUMI_OTA_IMAGE_TYPE_TOUCH_SWITCH_1_BUTTON		=		0x01,
	LUMI_OTA_IMAGE_TYPE_TOUCH_SWITCH_2_BUTTON		=		0x02,
	LUMI_OTA_IMAGE_TYPE_TOUCH_SWITCH_3_BUTTON		=		0x03,
	LUMI_OTA_IMAGE_TYPE_TOUCH_SWITCH_4_BUTTON		=		0x04,
	LUMI_OTA_IMAGE_TYPE_TOUCH_SWITCH_6_BUTTON		=		0x06,
	LUMI_OTA_IMAGE_TYPE_TOUCH_SWITCH_8_BUTTON		=		0x08,
	LUMI_OTA_IMAGE_TYPE_TOUCH_SWITCH_10_BUTTON		=		0x0A,
	LUMI_OTA_IMAGE_TYPE_TOUCH_SWITCH_12_BUTTON		=		0x0C,

	//Define curtain and roll door
	LUMI_OTA_IMAGE_TYPE_TOUCH_SWITCH_CURTAIN_SINGLE	=		0x11,
	LUMI_OTA_IMAGE_TYPE_TOUCH_SWITCH_CURTAIN_DOUBLE	=		0x12,
	LUMI_OTA_IMAGE_TYPE_TOUCH_SWITCH_ROLL_DOOR		=		0x13,

	//Define dimmer
	LUMI_OTA_IMAGE_TYPE_TOUCH_SWITCH_DIMMER_SINGLE	=		0x21,
	LUMI_OTA_IMAGE_TYPE_TOUCH_SWITCH_DIMMER_DOUBLE	=		0x22,

	//Define Fan
	LUMI_OTA_IMAGE_TYPE_TOUCH_SWITCH_FAN			=		0x31,
};
/******************************************************************************/
/*                              PRIVATE DATA                                  */
/******************************************************************************/
/******************************************************************************/
/*                              EXPORTED DATA                                 */
/******************************************************************************/
/******************************************************************************/
/*                            PRIVATE FUNCTIONS                               */
/******************************************************************************/

/******************************************************************************/
/*                            EXPORTED FUNCTIONS                              */
/******************************************************************************/
/******************************************************************************/



#endif /* LUMI_TYPES_H_ */
