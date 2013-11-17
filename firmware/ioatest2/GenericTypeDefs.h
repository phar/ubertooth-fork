#ifndef __GTD_H
#define __GTD_H

#include <stdbool.h>

typedef char    INT8;
typedef short   INT16;
typedef long    INT32;
//typedef int	INT;

/* Alternate definitions */
typedef void                    VOID;

typedef char                    CHAR8;
typedef unsigned char           UCHAR8;

typedef unsigned char           BYTE;                           /* 8-bit unsigned  */
typedef unsigned short int      WORD;                           /* 16-bit unsigned */
typedef unsigned long           DWORD;                          /* 32-bit unsigned */
/* MPLAB C Compiler for PIC18 does not support 64-bit integers */
typedef unsigned long long      QWORD;                          /* 64-bit unsigned */
typedef signed char             CHAR;                           /* 8-bit signed    */
typedef signed short int        SHORT;                          /* 16-bit signed   */
typedef signed long             LONG;                           /* 32-bit signed   */
/* MPLAB C Compiler for PIC18 does not support 64-bit integers */
typedef signed long long        LONGLONG;                       /* 64-bit signed   */



typedef unsigned int        UINT;
typedef unsigned char       UINT8;
typedef unsigned short int  UINT16;
typedef unsigned long int   UINT32; 
#define NULL 0
#endif
