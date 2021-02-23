/* 
 * File:   UserDataTypes.h
 * Author: 
 *
 * Created on 
 */

#ifndef USERDATATYPES_H
#define	USERDATATYPES_H


#ifndef PRIVATE
    #define PRIVATE static
#endif

#ifndef PUBLIC
    #define PUBLIC
#endif
    
/*-----------------------------------------------------------------------------------------
------
------  Compiler Defines
------
-----------------------------------------------------------------------------------------*/ 
#ifndef NULL
#define NULL          0
#endif

/**
FALSE: Will be used for variables from type BOOL */
#ifndef FALSE
#define FALSE                                     0
#endif

/**
TRUE: Will be used for variables from type BOOL  */
#ifndef TRUE
#define TRUE                                      1
#endif

/**
BOOL: Should be adapted to the boolean type of the microcontroller */
#ifndef BOOL
#define BOOL                                      unsigned char
#endif

/**
UINT8: Should be adapted to the unsigned8 type of the microcontroller  */
#ifndef UINT8
#define UINT8                                     unsigned char
#endif

/**
UINT16: Should be adapted to the unsigned16 type of the microcontroller  */
#ifndef UINT16
#define UINT16                                    unsigned short
#endif

/**
UINT32: Should be adapted to the unsigned32 type of the microcontroller  */
#ifndef UINT32
#define UINT32                                    unsigned long
#endif

/**
USHORT: Should be adapted to the unsigned16 type of the microcontroller */
#ifndef USHORT
#define USHORT                                    unsigned short
#endif

/**
INT8: Should be adapted to the integer8 type of the microcontroller */
#ifndef INT8
#define INT8                                      char
#endif

/**
INT16: Should be adapted to the integer16 type of the microcontroller  */
#ifndef INT16
#define INT16                                     short
#endif

/**
INT32: Should be adapted to the integer32 type of the microcontroller */
#ifndef INT32
#define INT32                                     int
#endif

/**
CHAR: Should be adapted to the character type of the microcontroller */
#ifndef CHAR
#define CHAR                                      char
#endif

/**
UCHAR: Should be adapted to the unsigned character type of the microcontroller */
#ifndef UCHAR
#define UCHAR                                     unsigned char
#endif

/**
BYTE: Should be adapted to the character type of the microcontroller */
#ifndef BYTE
#define BYTE                                      char
#endif

/**
WORD: Should be adapted to the WORDacter type of the microcontroller */
#ifndef WORD
#define WORD                                      short
#endif

/**
DWORD: Should be adapted to the character type of the microcontroller */
#ifndef DWORD
#define DWORD                                      int
#endif

/**
REAL32: Should be adapted to the character type of the microcontroller */
#ifndef	REAL32
#define	REAL32                                     float
#endif


typedef union
{
    UINT8 Val;
    struct
    {
        UINT8 b0:1;
        UINT8 b1:1;
        UINT8 b2:1;
        UINT8 b3:1;
        UINT8 b4:1;
        UINT8 b5:1;
        UINT8 b6:1;
        UINT8 b7:1;
    } bits;
} UINT8_VAL, UINT8_BITS;

typedef union
{
    UINT16 Val;
    UINT8 v[2];
    struct
    {
        UINT8 LB;
        UINT8 HB;
    } byte;
    struct
    {
        UINT8 b0:1;
        UINT8 b1:1;
        UINT8 b2:1;
        UINT8 b3:1;
        UINT8 b4:1;
        UINT8 b5:1;
        UINT8 b6:1;
        UINT8 b7:1;
        UINT8 b8:1;
        UINT8 b9:1;
        UINT8 b10:1;
        UINT8 b11:1;
        UINT8 b12:1;
        UINT8 b13:1;
        UINT8 b14:1;
        UINT8 b15:1;
    } bits;
} UINT16_VAL, UINT16_BITS;

typedef union
{
    UINT32 Val;
    UINT16 w[2];
    UINT8  v[4];
    struct
    {
        UINT16 LW;
        UINT16 HW;
    } word;
    struct
    {
        UINT8 LB;
        UINT8 HB;
        UINT8 UB;
        UINT8 MB;
    } byte;
    struct
    {
        UINT16_VAL low;
        UINT16_VAL high;
    }wordUnion;
    struct
    {
        UINT8 b0:1;
        UINT8 b1:1;
        UINT8 b2:1;
        UINT8 b3:1;
        UINT8 b4:1;
        UINT8 b5:1;
        UINT8 b6:1;
        UINT8 b7:1;
        UINT8 b8:1;
        UINT8 b9:1;
        UINT8 b10:1;
        UINT8 b11:1;
        UINT8 b12:1;
        UINT8 b13:1;
        UINT8 b14:1;
        UINT8 b15:1;
        UINT8 b16:1;
        UINT8 b17:1;
        UINT8 b18:1;
        UINT8 b19:1;
        UINT8 b20:1;
        UINT8 b21:1;
        UINT8 b22:1;
        UINT8 b23:1;
        UINT8 b24:1;
        UINT8 b25:1;
        UINT8 b26:1;
        UINT8 b27:1;
        UINT8 b28:1;
        UINT8 b29:1;
        UINT8 b30:1;
        UINT8 b31:1;
    } bits;
} UINT32_VAL;


typedef union
{
    BYTE Val;
    struct
    {
        BYTE b0:1;
        BYTE b1:1;
        BYTE b2:1;
        BYTE b3:1;
        BYTE b4:1;
        BYTE b5:1;
        BYTE b6:1;
        BYTE b7:1;
    } bits;
} BYTE_VAL, BYTE_BITS;

typedef union
{
    WORD Val;
    BYTE v[2];
    struct
    {
        BYTE LB;
        BYTE HB;
    } byte;
    struct
    {
        BYTE b0:1;
        BYTE b1:1;
        BYTE b2:1;
        BYTE b3:1;
        BYTE b4:1;
        BYTE b5:1;
        BYTE b6:1;
        BYTE b7:1;
        BYTE b8:1;
        BYTE b9:1;
        BYTE b10:1;
        BYTE b11:1;
        BYTE b12:1;
        BYTE b13:1;
        BYTE b14:1;
        BYTE b15:1;
    } bits;
} WORD_VAL, WORD_BITS;

typedef union
{
    DWORD Val;
    WORD w[2];
    BYTE v[4];
    struct
    {
        WORD LW;
        WORD HW;
    } word;
    struct
    {
        BYTE LB;
        BYTE HB;
        BYTE UB;
        BYTE MB;
    } byte;
    struct
    {
        WORD_VAL low;
        WORD_VAL high;
    }wordUnion;
    struct
    {
        BYTE b0:1;
        BYTE b1:1;
        BYTE b2:1;
        BYTE b3:1;
        BYTE b4:1;
        BYTE b5:1;
        BYTE b6:1;
        BYTE b7:1;
        BYTE b8:1;
        BYTE b9:1;
        BYTE b10:1;
        BYTE b11:1;
        BYTE b12:1;
        BYTE b13:1;
        BYTE b14:1;
        BYTE b15:1;
        BYTE b16:1;
        BYTE b17:1;
        BYTE b18:1;
        BYTE b19:1;
        BYTE b20:1;
        BYTE b21:1;
        BYTE b22:1;
        BYTE b23:1;
        BYTE b24:1;
        BYTE b25:1;
        BYTE b26:1;
        BYTE b27:1;
        BYTE b28:1;
        BYTE b29:1;
        BYTE b30:1;
        BYTE b31:1;
    } bits;
} DWORD_VAL;

#endif	/* USERDATATYPES_H */

