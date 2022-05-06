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
 * File name: main.h
 *
 * Description: This file supply the redefination of the general types.
 *
 * Author: PhuongNP
 *
 * Last Changed By:  $Author: phuongnp $
 * Revision:         $Revision: $
 * Last Changed:     $Date: $Jun 9, 2021
 *
 * Code sample:
 ******************************************************************************/
// Enclosing macro to prevent multiple inclusion
#ifndef UTILS_TYPEDEFS_H_
#define UTILS_TYPEDEFS_H_
/******************************************************************************/
/*                              INCLUDE FILES                                 */
/******************************************************************************/

/******************************************************************************/
/*                     EXPORTED TYPES and DEFINITIONS                         */
/******************************************************************************/
#ifndef byte_t
typedef unsigned char           byte_t;       // 1 byte
#endif
#ifndef word_t
typedef unsigned short          word_t;       // 2 bytes
#endif
#ifndef dword_t
typedef unsigned long           dword_t;      // 4 bytes
#endif
#ifndef uint_t
typedef unsigned int            uint_t;       // 4 bytes
#endif
#ifndef int_t
typedef signed int              int_t;
#endif
#ifndef bool_t
typedef unsigned char           bool_t;
#endif
#ifndef char_t
typedef char                    char_t;       // 1 byte
#endif
#ifndef void_t
typedef void                    void_t;
#endif

typedef byte_t*                 byte_p;
typedef word_t*                 word_p;
typedef dword_t*                dword_p;
typedef uint_t*                 uint_p;
typedef char_t*                 char_p;
typedef void_t*                 void_p;

/**
 * Redefine the general variable types
*/
#ifndef u8_t
typedef unsigned char           u8_t;       // 1 byte
#endif
#ifndef u16_t
typedef unsigned short          u16_t;      // 2 bytes
#endif
#ifndef u32_t
typedef unsigned long           u32_t;      // 4 bytes
#endif
#ifndef u64_t
typedef unsigned long long      u64_t;
#endif

#ifndef i8_t
typedef signed char             i8_t;       // 1 byte
#endif
#ifndef i16_t
typedef signed short            i16_t;      // 2 bytes
#endif
#ifndef i32_t
typedef signed long             i32_t;      // 4 bytes
#endif
#ifndef i64_t
typedef signed long long        i64_t;
#endif

#ifndef u8_p
typedef unsigned char*          u8_p;       // 1 byte
#endif
#ifndef u16_p
typedef unsigned short*         u16_p;      // 2 bytes
#endif
#ifndef u32_p
typedef unsigned long*          u32_p;      // 4 bytes
#endif
#ifndef u64_p
typedef unsigned long long*     u64_p;
#endif

#ifndef i8_p
typedef signed char*            i8_p;       // 1 byte
#endif
#ifndef i16_p
typedef signed short*           i16_p;      // 2 bytes
#endif
#ifndef i32_p
typedef signed long*            i32_p;      // 4 bytes
#endif
#ifndef i64_p
typedef signed long long*       i64_p;
#endif

/**
 * Define callback function types 
*/
typedef void (*voidCallbackFunc) (void);
typedef void (*boolCallbackFunc) (bool_t);
typedef void (*byteCallbackFunc) (byte_t);
typedef void (*wordCallbackFunc) (word_t);
typedef void (*uintCallbackFunc) (uint_t);

typedef void (*byte_pCallbackFunc) (byte_p);
typedef void (*byte_pCallbackFunc2Args) (byte_p, byte_t);
typedef void (*word_pCallbackFunc) (word_p);
typedef void (*uint_pCallbackFunc) (uint_p);

#ifndef TRUE
#define TRUE                1
#define FALSE               0
#endif

#ifndef NULL
#define NULL                (0)
#endif

#define MAX_U8              255
#define MAX_U16             65535

#if !defined(WIN32) && !defined(size_t)
typedef uint32_t size_t;
#endif	/*WIN32*/
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
#endif /* UTILS_TYPEDEFS_H_ */