/*********************************************************************
*               SEGGER MICROCONTROLLER GmbH & Co. KG                 *
*       Solutions for real time microcontroller applications         *
**********************************************************************
*                                                                    *
*       (c) 2015 - 2016  SEGGER Microcontroller GmbH & Co. KG        *
*                                                                    *
*       www.segger.com     Support: support@segger.com               *
*                                                                    *
**********************************************************************
*                                                                    *
*       SEGGER SystemView * Real-time application analysis           *
*                                                                    *
**********************************************************************
*                                                                    *
* All rights reserved.                                               *
*                                                                    *
* * This software may in its unmodified form be freely redistributed *
*   in source form.                                                  *
* * The source code may be modified, provided the source code        *
*   retains the above copyright notice, this list of conditions and  *
*   the following disclaimer.                                        *
* * Modified versions of this software in source or linkable form    *
*   may not be distributed without prior consent of SEGGER.          *
*                                                                    *
* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS "AS IS" AND     *
* ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,  *
* THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A        *
* PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL               *
* SEGGER Microcontroller BE LIABLE FOR ANY DIRECT, INDIRECT,         *
* INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES           *
* (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS    *
* OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS            *
* INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,       *
* WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING          *
* NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS *
* SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.       *
*                                                                    *
**********************************************************************
*                                                                    *
*       SystemView version: V2.30                                    *
*                                                                    *
**********************************************************************
--------  END-OF-HEADER  ---------------------------------------------

File        : SEGGER_SYSVIEW.c
Purpose     : System visualization API implementation.

Additional information:
  Packet format:
    Packets with IDs 0..23 are standard packets with known structure.
    For efficiency, they do *NOT* contain a length field.
    <ID><Data><TimeStampDelta>

    Packets with IDs 24..31 are standard packets with extendible
    structure and contain a length field.
    <ID><Lenght><Data><TimeStampDelta>

    Packets with IDs >= 32 always contain a length field.
    <ID><Length><Data><TimeStampDelta>

  Packet IDs:
       0..  31 : Standard packets, known by SysViewer.
      32..1023 : OS-definable packets, described in a SystemView description file.
    1024..2047 : User-definable packets, described in a SystemView description file.
    2048..32767: Undefined.

  Data encoding:
    Basic types (int, short, char, ...):
      Basic types are encoded little endian with most-significant bit variant
      encoding.
      Each encoded byte contains 7 data bits [6:0] and the MSB continuation bit.
      The continuation bit indicates whether the next byte belongs to the data
      (bit set) or this is the last byte (bit clear).
      The most significant bits of data are encoded first, proceeding to the
      least significant bits in the final byte (little endian).

      Example encoding:
        Data: 0x1F4 (500)
        Encoded: 0xF4 (First 7 data bits 74 | Continuation bit)
                 0x03 (Second 7 data bits 03, no continuation)

        Data: 0xFFFFFFFF
        Encoded: 0xFF 0xFF 0xFF 0xFF 0x0F

        Data: 0xA2 (162),   0x03 (3), 0x7000
        Encoded: 0xA2 0x01  0x03      0x80 0xE0 0x01

    Byte arrays and strings:
      Byte arrays and strings are encoded as <NumBytes> followed by the raw data.
      NumBytes is encoded as a basic type with a theoretical maximum of 4G.

      Example encoding:
        Data: "Hello World\0" (0x48 0x65 0x6C 0x6C 0x6F 0x20 0x57 0x6F 0x72 0x6C 0x64 0x00)
        Encoded: 0x0B 0x48 0x65 0x6C 0x6C 0x6F 0x20 0x57 0x6F 0x72 0x6C 0x64

  Examples packets:
  01 F4 03 80 80 10 // Overflow packet. Data is a single U32.
                       This packet means: 500 packets lost, Timestamp is 0x40000

  02 0F 50          // ISR(15) Enter. Timestamp 80 (0x50)

  03 20             // ISR Exit. Timestamp 32 (0x20) (Shortest possible packet.)

  Sample code for user defined Packets:
    #define MY_ID   0x400                // Any value between 0x400 and 0x7FF
    void SendMyPacket(unsigned Para0, unsigned Para1, const char* s) {
      U8  aPacket[SEGGER_SYSVIEW_INFO_SIZE + 2 * SEGGER_SYSVIEW_QUANTA_U32 + MAX_STR_LEN + 1];
      U8* pPayload;
      //
      pPayload = SEGGER_SYSVIEW_PPREPARE_PACKET(aPacket);               // Prepare the packet for SystemView
      pPayload = SEGGER_SYSVIEW_EncodeU32(pPayload, Para0);             // Add the first parameter to the packet
      pPayload = SEGGER_SYSVIEW_EncodeU32(pPayload, Para1);             // Add the second parameter to the packet
      pPayload = SEGGER_SYSVIEW_EncodeString(pPayload, s, MAX_STR_LEN); // Add the string to the packet
      //
      SEGGER_SYSVIEW_SendPacket(&aPacket[0], pPayload, MY_ID);          // Send the packet with EventId = MY_ID
    }

    #define MY_ID_1 0x401
    void SendOnePara(unsigned Para0) {
      SEGGER_SYSVIEW_RecordU32(MY_ID_1, Para0);
    }

*/

/*********************************************************************
*
*       #include section
*
**********************************************************************
*/

#include "SEGGER_SYSVIEW_Int.h"
#include "SEGGER_RTT.h"
#include <string.h>
#include <stdlib.h>
#include <stdarg.h>

/*********************************************************************
*
*       Defines, fixed
*
**********************************************************************
*/
#if SEGGER_SYSVIEW_ID_SHIFT
  #define SHRINK_ID(Id)   (((Id) - _SYSVIEW_Globals.RAMBaseAddress) >> SEGGER_SYSVIEW_ID_SHIFT)
#else
  #define SHRINK_ID(Id)   ((Id) - _SYSVIEW_Globals.RAMBaseAddress)
#endif

#if SEGGER_SYSVIEW_RTT_CHANNEL > 0
  #define CHANNEL_ID_UP   SEGGER_SYSVIEW_RTT_CHANNEL
  #define CHANNEL_ID_DOWN SEGGER_SYSVIEW_RTT_CHANNEL
#else
  #define CHANNEL_ID_UP   _SYSVIEW_Globals.UpChannel
  #define CHANNEL_ID_DOWN _SYSVIEW_Globals.DownChannel
#endif

/*********************************************************************
*
*       Defines, configurable
*
**********************************************************************
*/
// Timestamps may be less than full 32-bits, in which case we need to zero
// the unused bits to properly handle overflows.
// Note that this is a quite common scenario, as a 32-bit time such as
// SysTick might be scaled down to reduce bandwith
// or a 16-bit hardware time might be used.
#if SEGGER_SYSVIEW_TIMESTAMP_BITS < 32  // Eliminate unused bits in case hardware timestamps are less than 32 bits
  #define MAKE_DELTA_32BIT(Delta) Delta <<= 32 - SEGGER_SYSVIEW_TIMESTAMP_BITS; \
                                  Delta >>= 32 - SEGGER_SYSVIEW_TIMESTAMP_BITS;
#else
  #define MAKE_DELTA_32BIT(Delta)
#endif


/*********************************************************************
*
*       Defines, fixed
*
**********************************************************************
*/
#define ENABLE_STATE_OFF        0
#define ENABLE_STATE_ON         1
#define ENABLE_STATE_DROPPING   2

#define FORMAT_FLAG_LEFT_JUSTIFY   (1u << 0)
#define FORMAT_FLAG_PAD_ZERO       (1u << 1)
#define FORMAT_FLAG_PRINT_SIGN     (1u << 2)
#define FORMAT_FLAG_ALTERNATE      (1u << 3)

#define MODULE_EVENT_OFFSET        (512)

/*********************************************************************
*
*       Types
*
**********************************************************************
*/
typedef struct {
  U8*       pBuffer;
  U8*       pPayload;
  U8*       pPayloadStart;
  U32       Options;
  unsigned  Cnt;
} SEGGER_SYSVIEW_PRINTF_DESC;

/*********************************************************************
*
*       Static constant data
*
**********************************************************************
*/

/*********************************************************************
*
*       Static data
*
**********************************************************************
*/

static struct {
        U8                      EnableState;   // 0: Disabled, 1: Enabled, 2: Dropping
        U8                      UpChannel;
        U8                      DownChannel;
        U8                      RecursionCnt;
        U32                     SysFreq;
        U32                     CPUFreq;
        U32                     DropCount;
        U32                     LastTxTimeStamp;
        U32                     RAMBaseAddress;
        char                    DownBuffer[8];  // Small, fixed-size buffer, for back-channel comms
        char                    UpBuffer  [SEGGER_SYSVIEW_RTT_BUFFER_SIZE];
  const SEGGER_SYSVIEW_OS_API*  pOSAPI;
        SEGGER_SYSVIEW_SEND_SYS_DESC_FUNC*   pfSendSysDesc;
} _SYSVIEW_Globals;

static SEGGER_SYSVIEW_MODULE* _pFirstModule;
static U8                     _NumModules;

/*********************************************************************
*
*       Static code
*
**********************************************************************
*/

#define ENCODE_U32(pDest, Value) {                                                  \
                                   U8* pSysviewPointer;                             \
                                   U32 SysViewData;                                 \
                                   pSysviewPointer = pDest;                         \
                                   SysViewData = Value;                             \
                                   while(SysViewData > 0x7F) {                      \
                                     *pSysviewPointer++ = (U8)(SysViewData | 0x80); \
                                     SysViewData >>= 7;                             \
                                   };                                               \
                                   *pSysviewPointer++ = (U8)SysViewData;            \
                                   pDest = pSysviewPointer;                         \
                                 };

/*********************************************************************
*
*       _EncodeData()
*
*  Function description
*    Encode a byte buffer in variable-length format.
*
*  Parameters
*    pPayload - Pointer to where string will be encoded.
*    pSrc     - Pointer to data buffer to be encoded.
*    NumBytes - Number of bytes in the buffer to be encoded.
*
*  Return value
*    Pointer to the byte following the value, i.e. the first free
*    byte in the payload and the next position to store payload
*    content.
*
*  Additional information
*    The data is encoded as a count byte followed by the contents
*    of the data buffer.
*    Make sure NumBytes + 1 bytes are free for the payload.
*/
static U8* _EncodeData(U8* pPayload, const char* pSrc, unsigned NumBytes) {
  unsigned n;
  //
  n = 0;
  *pPayload++ = NumBytes;
  while (n < NumBytes) {
    *pPayload++ = *pSrc++;
    n++;
  }
  return pPayload;
}

/*********************************************************************
*
*       _EncodeStr()
*
*  Function description
*    Encode a string in variable-length format.
*
*  Parameters
*    pPayload - Pointer to where string will be encoded.
*    pText    - String to encode.
*    Limit    - Maximum number of characters to encode from string.
*
*  Return value
*    Pointer to the byte following the value, i.e. the first free
*    byte in the payload and the next position to store payload
*    content.
*
*  Additional information
*    The string is encoded as a count byte followed by the contents
*    of the string.
*    No more than 1 + Limit bytes will be encoded to the payload.
*/
static U8 *_EncodeStr(U8 *pPayload, const char *pText, unsigned Limit) {
  unsigned n;
  //
  for (n = 0; n < Limit && *pText; ++n) {
    pPayload[1+n] = *pText++;
  }
  pPayload[0] = n;
  return pPayload + n + 1;
}

/*********************************************************************
*
*       _PreparePacket()
*
*  Function description
*    Prepare a SystemView event packet header.
*
*  Parameters
*    pPacket - Pointer to start of packet to initialize.
*
*  Return value
*    Pointer to first byte of packet payload.
*
*  Additional information
*    The payload length and evnetId are not initialized.
*    PreparePacket only reserves space for them and they are
*    computed and filled in by the sending function.
*/
static U8* _PreparePacket(U8* pPacket) {
  return pPacket + 4;
}

/*********************************************************************
*
*       _HandleIncomingPacket()
*
*  Function description
*    Read an incoming command from the down channel and process it.
*
*  Additional information
*    This function is called each time after sending a packet.
*    Processing incoming packets is done asynchronous. SystemView might
*    already have sent event packets after the host has sent a command.
*/
static void _HandleIncomingPacket(void) {
  U8  Cmd;
  int Status;
  //
  Status = SEGGER_RTT_ReadNoLock(CHANNEL_ID_DOWN, &Cmd, 1);
  if (Status > 0) {
    switch (Cmd) {
    case SEGGER_SYSVIEW_COMMAND_ID_START:
      SEGGER_SYSVIEW_Start();
      break;
    case SEGGER_SYSVIEW_COMMAND_ID_STOP:
      SEGGER_SYSVIEW_Stop();
      break;
    case SEGGER_SYSVIEW_COMMAND_ID_GET_SYSTIME:
      SEGGER_SYSVIEW_RecordSystime();
      break;
    case SEGGER_SYSVIEW_COMMAND_ID_GET_TASKLIST:
      SEGGER_SYSVIEW_SendTaskList();
      break;
    case SEGGER_SYSVIEW_COMMAND_ID_GET_SYSDESC:
      SEGGER_SYSVIEW_GetSysDesc();
      break;
    case SEGGER_SYSVIEW_COMMAND_ID_GET_NUMMODULES:
      SEGGER_SYSVIEW_SendNumModules();
      break;
    case SEGGER_SYSVIEW_COMMAND_ID_GET_MODULEDESC:
      SEGGER_SYSVIEW_SendModuleDescription();
      break;
    case SEGGER_SYSVIEW_COMMAND_ID_GET_MODULE:
      Status = SEGGER_RTT_ReadNoLock(CHANNEL_ID_DOWN, &Cmd, 1);
      if (Status > 0) {
        SEGGER_SYSVIEW_SendModule(Cmd);
      }
      break;
    default:
      if (Cmd >= 128) { // Unknown extended command. Dummy read its parameter.
        SEGGER_RTT_ReadNoLock(CHANNEL_ID_DOWN, &Cmd, 1);
      }
      break;
    }
  }
}

/*********************************************************************
*
*       _TrySendOverflowPacket()
*
*  Function description
*    Try to transmit an SystemView Overflow packet containing the
*    number of dropped packets.
*
*  Additional information
*    Format as follows:
*      01 <DropCnt><TimeStamp>  Max. packet len is 1 + 5 + 5 = 11
*
*    Example packets sent
*      01 20 40
*
*  Return value
*    !=0:  Success, Message sent (stored in RTT-Buffer)
*    ==0:  Buffer full, Message *NOT* stored
*
*/
static int _TrySendOverflowPacket(void) {
  U32 TimeStamp;
  S32 Delta;
  int Status;
  U8  aPacket[11];
  U8* pPayload;

  aPacket[0] = SEGGER_SYSVIEW_EVENT_ID_OVERFLOW;      // 1
  pPayload   = &aPacket[1];
  ENCODE_U32(pPayload, _SYSVIEW_Globals.DropCount);
  //
  // Compute time stamp delta and append it to packet.
  //
  TimeStamp  = SEGGER_SYSVIEW_GET_TIMESTAMP();
  Delta = TimeStamp - _SYSVIEW_Globals.LastTxTimeStamp;
  MAKE_DELTA_32BIT(Delta);
  ENCODE_U32(pPayload, Delta);
  //
  // Try to store packet in RTT buffer and update time stamp when this was successful
  //
  Status = SEGGER_RTT_WriteSkipNoLock(CHANNEL_ID_UP, aPacket, pPayload - aPacket);
  if (Status) {
    _SYSVIEW_Globals.LastTxTimeStamp = TimeStamp;
    _SYSVIEW_Globals.EnableState--; // EnableState has been 2, will be 1. Always.
  } else {
    _SYSVIEW_Globals.DropCount++;
  }
  //
  return Status;
}

/*********************************************************************
*
*       _SendPacket()
*
*  Function description
*    Send a SystemView packet over RTT. RTT channel and mode are
*    configured by macros when the SysView component is initialized.
*    This function takes care of maintaining the packet drop count
*    and sending overflow packets when necessary.
*    The packet must be passed without Id and Length because this
*    function prepends it to the packet before transmission.
*
*  Parameters
*    pStartPacket - Pointer to start of packet payload.
*                   There must be at least 4 bytes free to prepend Id and Length.
*    pEndPacket   - Pointer to end of packet payload.
*    EventId      - Id of the event to send.
*
*/
static void _SendPacket(U8* pStartPacket, U8* pEndPacket, unsigned EventId) {
  unsigned  NumBytes;
  U32 TimeStamp;
  S32 Delta;
  int Status;

  SEGGER_SYSVIEW_LOCK();
  if (_SYSVIEW_Globals.EnableState == 1) {  // Enabled, no dropped packets remaining
    goto Send;
  }
  if (_SYSVIEW_Globals.EnableState == 0) {
    goto SendDone;
  }
  //
  // Handle buffer full situations:
  // Have packets been dropped before because buffer was full?
  // In this case try to send and overflow packet.
  //
  if (_SYSVIEW_Globals.EnableState == 2) {
    _TrySendOverflowPacket();
    if (_SYSVIEW_Globals.EnableState != 1) {
      goto SendDone;
    }
  }
Send:
  //
  // Prepare actual packet.
  // If it is a known packet, prepend eventId only,
  // otherwise prepend packet length and eventId.
  //
  if (EventId < 24) {
    *--pStartPacket = EventId;
  } else {
    NumBytes = pEndPacket - pStartPacket;
    if (NumBytes > 127) {
      *--pStartPacket = (NumBytes >> 7);
      *--pStartPacket = NumBytes | 0x80;
    } else {
      *--pStartPacket = NumBytes;
    }
    if (EventId > 127) {
      *--pStartPacket = (EventId >> 7);
      *--pStartPacket = EventId | 0x80;
    } else {
      *--pStartPacket = EventId;
    }
  }
  //
  // Compute time stamp delta and append it to packet.
  //
  TimeStamp  = SEGGER_SYSVIEW_GET_TIMESTAMP();
  Delta = TimeStamp - _SYSVIEW_Globals.LastTxTimeStamp;
  MAKE_DELTA_32BIT(Delta);
  ENCODE_U32(pEndPacket, Delta);
  //
  // Try to store packet in RTT buffer and update time stamp when this was successful
  //
  Status = SEGGER_RTT_WriteSkipNoLock(CHANNEL_ID_UP, pStartPacket, pEndPacket - pStartPacket);
  if (Status) {
    _SYSVIEW_Globals.LastTxTimeStamp = TimeStamp;
  } else {
    _SYSVIEW_Globals.EnableState++; // EnableState has been 1, will be 2. Always.
  }

  //
  // Check if host is sending data which needs to be processed.
  // Note that since this code is called for every packet, it is very time critical, so we do
  // only what is really needed here, which is checking if there is any data
  //
SendDone:
  if (SEGGER_RTT_HASDATA(CHANNEL_ID_DOWN)) {
    if (_SYSVIEW_Globals.RecursionCnt == 0) {   // Avoid uncontrolled nesting. This way, this routine can call itself once, but no more often than that.
      _SYSVIEW_Globals.RecursionCnt = 1;
      _HandleIncomingPacket();
      _SYSVIEW_Globals.RecursionCnt = 0;
    }
  }
  //
  SEGGER_SYSVIEW_UNLOCK();  // We are done. Unlock and return
}

#ifndef SEGGER_SYSVIEW_EXCLUDE_PRINTF // Define in project to avoid warnings about variable parameter list
/*********************************************************************
*
*       _APrintHost()
*
*  Function description
*    Prepares a string and its parameters to be formatted on the host.
*
*  Parameters
*    s            Pointer to format string.
*    Options      Options to be sent to the host.
*    pArguments   Pointer to array of arguments for the format string.
*    NumArguments Number of arguments in the array.
*/
static void _APrintHost(const char* s, U32 Options, U32* pArguments, U32 NumArguments) {
  U8 aPacket[SEGGER_SYSVIEW_INFO_SIZE + SEGGER_SYSVIEW_MAX_STRING_LEN + 2 * SEGGER_SYSVIEW_QUANTA_U32 + SEGGER_SYSVIEW_MAX_ARGUMENTS * SEGGER_SYSVIEW_QUANTA_U32];
  U8* pPayload;
  U8* pPayloadStart;

  pPayloadStart = _PreparePacket(aPacket);
  pPayload = _EncodeStr(pPayloadStart, s, SEGGER_SYSVIEW_MAX_STRING_LEN);
  ENCODE_U32(pPayload, Options);
  ENCODE_U32(pPayload, NumArguments);
  while (NumArguments--) {
    ENCODE_U32(pPayload, (*pArguments++));
  }
  _SendPacket(pPayloadStart, pPayload, SEGGER_SYSVIEW_EVENT_ID_PRINT_FORMATTED);
}

/*********************************************************************
*
*       _VPrintHost()
*
*  Function description
*    Prepares a string and its parameters to be formatted on the host.
*
*  Parameters
*    s            Pointer to format string.
*    Options      Options to be sent to the host.
*    pParamList   Pointer to the list of arguments for the format string.
*/
static void _VPrintHost(const char* s, U32 Options, va_list* pParamList) {
  U32 aParas[SEGGER_SYSVIEW_MAX_ARGUMENTS];
  U32 NumArguments;
  const char* p;
  
  p = s;
  NumArguments = 0;
  while (*p) {
    if (*p == '%') {
      aParas[NumArguments++] = va_arg(*pParamList, int);
      if (NumArguments == SEGGER_SYSVIEW_MAX_ARGUMENTS) {
        break;
      }
    }
    p++;
  }
  _APrintHost(s, Options, aParas, NumArguments);
}

/*********************************************************************
*
*       _StoreChar()
*
*  Parameters
*    p            Pointer to the buffer description.
*    c            Character to be printed.
*/
static void _StoreChar(SEGGER_SYSVIEW_PRINTF_DESC * p, char c) {
  unsigned Cnt;
  U8* pPayload;
  U32 Options;

  Cnt = p->Cnt;
  if ((Cnt + 1u) <= SEGGER_SYSVIEW_MAX_STRING_LEN) {
    *(p->pPayload++) = c;
    p->Cnt = Cnt + 1u;
  }
  //
  // Write part of string, when the buffer is full
  //
  if (p->Cnt == SEGGER_SYSVIEW_MAX_STRING_LEN) {
    *(p->pPayloadStart) = p->Cnt;
    pPayload = p->pPayload;
    Options = p->Options;
    ENCODE_U32(pPayload, Options);
    ENCODE_U32(pPayload, 0);
    _SendPacket(p->pPayloadStart, pPayload, SEGGER_SYSVIEW_EVENT_ID_PRINT_FORMATTED);
    p->pPayloadStart = _PreparePacket(p->pBuffer);
    p->pPayload = p->pPayloadStart + 1u;
    p->Cnt = 0u;
  }
}

/*********************************************************************
*
*       _PrintUnsigned()
*
*  Parameters
*    pBufferDesc  Pointer to the buffer description.
*    v            Value to be printed.
*    Base         Base of the value.
*    NumDigits    Number of digits to be printed.
*    FieldWidth   Width of the printed field.
*    FormatFlags  Flags for formatting the value.
*/
static void _PrintUnsigned(SEGGER_SYSVIEW_PRINTF_DESC * pBufferDesc, unsigned v, unsigned Base, unsigned NumDigits, unsigned FieldWidth, unsigned FormatFlags) {
  static const char _aV2C[16] = {'0', '1', '2', '3', '4', '5', '6', '7', '8', '9', 'A', 'B', 'C', 'D', 'E', 'F' };
  unsigned Div;
  unsigned Digit;
  unsigned Number;
  unsigned Width;
  char c;

  Number = v;
  Digit = 1u;
  //
  // Get actual field width
  //
  Width = 1u;
  while (Number >= Base) {
    Number = (Number / Base);
    Width++;
  }
  if (NumDigits > Width) {
    Width = NumDigits;
  }
  //
  // Print leading chars if necessary
  //
  if ((FormatFlags & FORMAT_FLAG_LEFT_JUSTIFY) == 0u) {
    if (FieldWidth != 0u) {
      if (((FormatFlags & FORMAT_FLAG_PAD_ZERO) == FORMAT_FLAG_PAD_ZERO) && (NumDigits == 0u)) {
        c = '0';
      } else {
        c = ' ';
      }
      while ((FieldWidth != 0u) && (Width < FieldWidth)) {
        FieldWidth--;
        _StoreChar(pBufferDesc, c);
      }
    }
  }
  //
  // Compute Digit.
  // Loop until Digit has the value of the highest digit required.
  // Example: If the output is 345 (Base 10), loop 2 times until Digit is 100.
  //
  while (1) {
    if (NumDigits > 1u) {       // User specified a min number of digits to print? => Make sure we loop at least that often, before checking anything else (> 1 check avoids problems with NumDigits being signed / unsigned)
      NumDigits--;
    } else {
      Div = v / Digit;
      if (Div < Base) {        // Is our divider big enough to extract the highest digit from value? => Done
        break;
      }
    }
    Digit *= Base;
  }
  //
  // Output digits
  //
  do {
    Div = v / Digit;
    v -= Div * Digit;
    _StoreChar(pBufferDesc, _aV2C[Div]);
    Digit /= Base;
  } while (Digit);
  //
  // Print trailing spaces if necessary
  //
  if ((FormatFlags & FORMAT_FLAG_LEFT_JUSTIFY) == FORMAT_FLAG_LEFT_JUSTIFY) {
    if (FieldWidth != 0u) {
      while ((FieldWidth != 0u) && (Width < FieldWidth)) {
        FieldWidth--;
        _StoreChar(pBufferDesc, ' ');
      }
    }
  }
}

/*********************************************************************
*
*       _PrintInt()
*
*  Parameters
*    pBufferDesc  Pointer to the buffer description.
*    v            Value to be printed.
*    Base         Base of the value.
*    NumDigits    Number of digits to be printed.
*    FieldWidth   Width of the printed field.
*    FormatFlags  Flags for formatting the value.
*/
static void _PrintInt(SEGGER_SYSVIEW_PRINTF_DESC * pBufferDesc, int v, unsigned Base, unsigned NumDigits, unsigned FieldWidth, unsigned FormatFlags) {
  unsigned Width;
  int Number;

  Number = (v < 0) ? -v : v;

  //
  // Get actual field width
  //
  Width = 1u;
  while (Number >= (int)Base) {
    Number = (Number / (int)Base);
    Width++;
  }
  if (NumDigits > Width) {
    Width = NumDigits;
  }
  if ((FieldWidth > 0u) && ((v < 0) || ((FormatFlags & FORMAT_FLAG_PRINT_SIGN) == FORMAT_FLAG_PRINT_SIGN))) {
    FieldWidth--;
  }

  //
  // Print leading spaces if necessary
  //
  if ((((FormatFlags & FORMAT_FLAG_PAD_ZERO) == 0u) || (NumDigits != 0u)) && ((FormatFlags & FORMAT_FLAG_LEFT_JUSTIFY) == 0u)) {
    if (FieldWidth != 0u) {
      while ((FieldWidth != 0u) && (Width < FieldWidth)) {
        FieldWidth--;
        _StoreChar(pBufferDesc, ' ');
      }
    }
  }
  //
  // Print sign if necessary
  //
  if (v < 0) {
    v = -v;
    _StoreChar(pBufferDesc, '-');
  } else if ((FormatFlags & FORMAT_FLAG_PRINT_SIGN) == FORMAT_FLAG_PRINT_SIGN) {
    _StoreChar(pBufferDesc, '+');
  } else {

  }
  //
  // Print leading zeros if necessary
  //
  if (((FormatFlags & FORMAT_FLAG_PAD_ZERO) == FORMAT_FLAG_PAD_ZERO) && ((FormatFlags & FORMAT_FLAG_LEFT_JUSTIFY) == 0u) && (NumDigits == 0u)) {
    if (FieldWidth != 0u) {
      while ((FieldWidth != 0u) && (Width < FieldWidth)) {
        FieldWidth--;
        _StoreChar(pBufferDesc, '0');
      }
    }
  }
  //
  // Print number without sign
  //
  _PrintUnsigned(pBufferDesc, (unsigned)v, Base, NumDigits, FieldWidth, FormatFlags);
}

/*********************************************************************
*
*       _VPrintTarget()
*
*  Function description
*    Stores a formatted string.
*    This data is read by the host.
*
*  Parameters
*    sFormat      Pointer to format string.
*    Options      Options to be sent to the host.
*    pParamList   Pointer to the list of arguments for the format string.
*/
static void _VPrintTarget(const char* sFormat, U32 Options, va_list* pParamList) {
  char c;
  SEGGER_SYSVIEW_PRINTF_DESC BufferDesc;
  int v;
  unsigned NumDigits;
  unsigned FormatFlags;
  unsigned FieldWidth;
  U8 aPacket[SEGGER_SYSVIEW_INFO_SIZE + SEGGER_SYSVIEW_MAX_STRING_LEN + 1 + 2 * SEGGER_SYSVIEW_QUANTA_U32];

  SEGGER_SYSVIEW_LOCK();

  BufferDesc.pBuffer        = aPacket;
  BufferDesc.Cnt            = 0u;
  BufferDesc.pPayloadStart  = _PreparePacket(aPacket);
  BufferDesc.pPayload       = BufferDesc.pPayloadStart + 1u;
  BufferDesc.Options        =  Options;

  do {
    c = *sFormat;
    sFormat++;
    if (c == 0u) {
      break;
    }
    if (c == '%') {
      //
      // Filter out flags
      //
      FormatFlags = 0u;
      v = 1;
      do {
        c = *sFormat;
        switch (c) {
        case '-': FormatFlags |= FORMAT_FLAG_LEFT_JUSTIFY; sFormat++; break;
        case '0': FormatFlags |= FORMAT_FLAG_PAD_ZERO;     sFormat++; break;
        case '+': FormatFlags |= FORMAT_FLAG_PRINT_SIGN;   sFormat++; break;
        case '#': FormatFlags |= FORMAT_FLAG_ALTERNATE;    sFormat++; break;
        default:  v = 0; break;
        }
      } while (v);
      //
      // filter out field with
      //
      FieldWidth = 0u;
      do {
        c = *sFormat;
        if ((c < '0') || (c > '9')) {
          break;
        }
        sFormat++;
        FieldWidth = (FieldWidth * 10u) + ((unsigned)c - '0');
      } while (1);

      //
      // Filter out precision (number of digits to display)
      //
      NumDigits = 0u;
      c = *sFormat;
      if (c == '.') {
        sFormat++;
        do {
          c = *sFormat;
          if ((c < '0') || (c > '9')) {
            break;
          }
          sFormat++;
          NumDigits = NumDigits * 10u + ((unsigned)c - '0');
        } while (1);
      }
      //
      // Filter out length modifier
      //
      c = *sFormat;
      do {
        if ((c == 'l') || (c == 'h')) {
          c = *sFormat;
          sFormat++;
        } else {
          break;
        }
      } while (1);
      //
      // Handle specifiers
      //
      switch (c) {
      case 'c': {
        char c0;
        v = va_arg(*pParamList, int);
        c0 = (char)v;
        _StoreChar(&BufferDesc, c0);
        break;
      }
      case 'd':
        v = va_arg(*pParamList, int);
        _PrintInt(&BufferDesc, v, 10u, NumDigits, FieldWidth, FormatFlags);
        break;
      case 'u':
        v = va_arg(*pParamList, int);
        _PrintUnsigned(&BufferDesc, (unsigned)v, 10u, NumDigits, FieldWidth, FormatFlags);
        break;
      case 'x':
      case 'X':
        v = va_arg(*pParamList, int);
        _PrintUnsigned(&BufferDesc, (unsigned)v, 16u, NumDigits, FieldWidth, FormatFlags);
        break;
      case 'p':
        v = va_arg(*pParamList, int);
        _PrintUnsigned(&BufferDesc, (unsigned)v, 16u, 8u, 8u, 0u);
        break;
      case '%':
        _StoreChar(&BufferDesc, '%');
        break;
      default:
        break;
      }
      sFormat++;
    } else {
      _StoreChar(&BufferDesc, c);
    }
  } while (*sFormat);

  //
  // Write remaining data, if any
  //
  if (BufferDesc.Cnt != 0u) {
    *(BufferDesc.pPayloadStart) = BufferDesc.Cnt;
    ENCODE_U32(BufferDesc.pPayload, BufferDesc.Options);
    ENCODE_U32(BufferDesc.pPayload, 0);
    _SendPacket(BufferDesc.pPayloadStart, BufferDesc.pPayload, SEGGER_SYSVIEW_EVENT_ID_PRINT_FORMATTED);
  }
  SEGGER_SYSVIEW_UNLOCK();
}
#endif // SEGGER_SYSVIEW_EXCLUDE_PRINTF

/*********************************************************************
*
*       Public functions
*
**********************************************************************
*/

/*********************************************************************
*
*       SEGGER_SYSVIEW_Init()
*
*  Function description
*    Initializes the SYSVIEW module.
*    Must be called before SystemViewer attaches to the system.
*
*  Parameters
*    SysFreq        - Frequency of timestamp, i.e. CPU core clock frequency.
*    CPUFreq        - CPU core clock frequency.
*    pOSAPI         - Pointer to the API structure for OS-specific functions.
*    pfSendSysDesc  - Pointer to SendSysDesc callback function.
*
*  Additional information
*    This function initializes the RTT channel used to transport 
*    SEGGER SystemView packets. 
*    The channel is assigned the label "SysView" for client software 
*    to identify the SystemView channel.
*
*  Notes
*    The channel is configured by the macro SEGGER_SYSVIEW_RTT_CHANNEL.
*/
void SEGGER_SYSVIEW_Init(U32 SysFreq, U32 CPUFreq, const SEGGER_SYSVIEW_OS_API *pOSAPI, SEGGER_SYSVIEW_SEND_SYS_DESC_FUNC pfSendSysDesc) {
#if SEGGER_SYSVIEW_RTT_CHANNEL > 0
  SEGGER_RTT_ConfigUpBuffer   (SEGGER_SYSVIEW_RTT_CHANNEL, "SysView", &_SYSVIEW_Globals.UpBuffer[0],   sizeof(_SYSVIEW_Globals.UpBuffer),   SEGGER_RTT_MODE_NO_BLOCK_SKIP);
  SEGGER_RTT_ConfigDownBuffer (SEGGER_SYSVIEW_RTT_CHANNEL, "SysView", &_SYSVIEW_Globals.DownBuffer[0], sizeof(_SYSVIEW_Globals.DownBuffer), SEGGER_RTT_MODE_NO_BLOCK_SKIP);
#else
  _SYSVIEW_Globals.UpChannel = SEGGER_RTT_AllocUpBuffer  ("SysView", &_SYSVIEW_Globals.UpBuffer[0],   sizeof(_SYSVIEW_Globals.UpBuffer),   SEGGER_RTT_MODE_NO_BLOCK_SKIP);
  //
  // TODO: Use SEGGER_RTT_AllocDownBuffer when SysViewer is able to handle another Down Channel than Up Channel.
  //
  _SYSVIEW_Globals.DownChannel = _SYSVIEW_Globals.UpChannel;
  SEGGER_RTT_ConfigDownBuffer (_SYSVIEW_Globals.DownChannel, "SysView", &_SYSVIEW_Globals.DownBuffer[0], sizeof(_SYSVIEW_Globals.DownBuffer), SEGGER_RTT_MODE_NO_BLOCK_SKIP);
#endif
  _SYSVIEW_Globals.RAMBaseAddress  = SEGGER_SYSVIEW_ID_BASE;
  _SYSVIEW_Globals.LastTxTimeStamp = SEGGER_SYSVIEW_GET_TIMESTAMP();
  _SYSVIEW_Globals.pOSAPI          = pOSAPI;
  _SYSVIEW_Globals.SysFreq         = SysFreq;
  _SYSVIEW_Globals.CPUFreq         = CPUFreq;
  _SYSVIEW_Globals.pfSendSysDesc   = pfSendSysDesc;
  _SYSVIEW_Globals.EnableState     = 0;
}

/*********************************************************************
*
*       SEGGER_SYSVIEW_SetRAMBase()
*
*  Function description
*    Sets the RAM base address, which is subtracted from IDs in order
*     to save bandwidth.
*
*  Parameters
*    RAMBaseAddress - Lowest RAM Address. (i.e. 0x20000000 on most Cortex-M)
*/
void SEGGER_SYSVIEW_SetRAMBase(U32 RAMBaseAddress) {
  _SYSVIEW_Globals.RAMBaseAddress = RAMBaseAddress;
}

/*********************************************************************
*
*       SEGGER_SYSVIEW_RecordVoid()
*
*  Function description
*    Formats and sends a SystemView packet with an empty payload.
*
*  Parameters
*    EventID - SystemView event ID.
*/
void SEGGER_SYSVIEW_RecordVoid(unsigned EventID) {
  U8  aPacket[SEGGER_SYSVIEW_INFO_SIZE];
  U8* pPayloadStart;

  pPayloadStart = _PreparePacket(aPacket);
  _SendPacket(pPayloadStart, pPayloadStart, EventID);
}

/*********************************************************************
*
*       SEGGER_SYSVIEW_RecordU32()
*
*  Function description
*    Formats and sends a SystemView packet containing a single U32
*    parameter payload.
*
*  Parameters
*    EventID - SystemView event ID.
*    Value   - The 32-bit parameter encoded to SystemView packet payload.
*/
void SEGGER_SYSVIEW_RecordU32(unsigned EventID, U32 Value) {
  U8  aPacket[SEGGER_SYSVIEW_INFO_SIZE + SEGGER_SYSVIEW_QUANTA_U32];
  U8* pPayload;
  U8* pPayloadStart;

  pPayloadStart = _PreparePacket(aPacket);
  pPayload = pPayloadStart;
  ENCODE_U32(pPayload, Value);
  _SendPacket(pPayloadStart, pPayload, EventID);
}

/*********************************************************************
*
*       SEGGER_SYSVIEW_RecordU32x2()
*
*  Function description
*    Formats and sends a SystemView packet containing 2 U32 parameter payload.
*
*  Parameters
*    EventID - SystemView event ID.
*    Para0   - The 32-bit parameter encoded to SystemView packet payload.
*    Para1   - The 32-bit parameter encoded to SystemView packet payload.
*/
void SEGGER_SYSVIEW_RecordU32x2(unsigned EventID, U32 Para0, U32 Para1) {
  U8  aPacket[SEGGER_SYSVIEW_INFO_SIZE + 2 * SEGGER_SYSVIEW_QUANTA_U32];
  U8* pPayload;
  U8* pPayloadStart;
  //
  pPayloadStart = _PreparePacket(aPacket);
  pPayload = pPayloadStart;
  ENCODE_U32(pPayload, Para0);
  ENCODE_U32(pPayload, Para1);
  _SendPacket(pPayloadStart, pPayload, EventID);
}

/*********************************************************************
*
*       SEGGER_SYSVIEW_RecordU32x3()
*
*  Function description
*    Formats and sends a SystemView packet containing 3 U32 parameter payload.
*
*  Parameters
*    EventID - SystemView event ID.
*    Para0   - The 32-bit parameter encoded to SystemView packet payload.
*    Para1   - The 32-bit parameter encoded to SystemView packet payload.
*    Para2   - The 32-bit parameter encoded to SystemView packet payload.
*/
void SEGGER_SYSVIEW_RecordU32x3(unsigned EventID, U32 Para0, U32 Para1, U32 Para2) {
  U8  aPacket[SEGGER_SYSVIEW_INFO_SIZE + 3 * SEGGER_SYSVIEW_QUANTA_U32];
  U8* pPayload;
  U8* pPayloadStart;
  //
  pPayloadStart = _PreparePacket(aPacket);
  pPayload = pPayloadStart;
  ENCODE_U32(pPayload, Para0);
  ENCODE_U32(pPayload, Para1);
  ENCODE_U32(pPayload, Para2);
  _SendPacket(pPayloadStart, pPayload, EventID);
}

/*********************************************************************
*
*       SEGGER_SYSVIEW_Start()
*
*  Function description
*    Start recording SystemView events.
*    This function is triggered by the host application.
*
*  Additional information
*    This function enables transmission of SystemView packets recorded
*    by subsequent trace calls and records a SystemView Start event.
*
*    As part of start, a SystemView Init packet is sent, containing the system
*    frequency. The list of current tasks, the current system time and the
*    system description string is sent, too.
*
*  Notes
*    SEGGER_SYSVIEW_Start and SEGGER_SYSVIEW_Stop do not nest.
*/
void SEGGER_SYSVIEW_Start(void) {
  U8  aPacket[SEGGER_SYSVIEW_INFO_SIZE + 4 * SEGGER_SYSVIEW_QUANTA_U32];
  U8* pPayload;
  U8* pPayloadStart;

  if (_SYSVIEW_Globals.EnableState == 0) {
    _SYSVIEW_Globals.EnableState = 1;
    SEGGER_SYSVIEW_RecordVoid(SEGGER_SYSVIEW_EVENT_ID_TRACE_START);
    pPayloadStart = _PreparePacket(aPacket);
    pPayload = pPayloadStart;
    ENCODE_U32(pPayload, _SYSVIEW_Globals.SysFreq);
    ENCODE_U32(pPayload, _SYSVIEW_Globals.CPUFreq);
    ENCODE_U32(pPayload, _SYSVIEW_Globals.RAMBaseAddress);
    ENCODE_U32(pPayload, SEGGER_SYSVIEW_ID_SHIFT);
    _SendPacket(pPayloadStart, pPayload, SEGGER_SYSVIEW_EVENT_ID_INIT);
    if (_SYSVIEW_Globals.pfSendSysDesc) {
      _SYSVIEW_Globals.pfSendSysDesc();
    }
    SEGGER_SYSVIEW_RecordSystime();
    SEGGER_SYSVIEW_SendTaskList();
    SEGGER_SYSVIEW_SendNumModules();
  }
}

/*********************************************************************
*
*       SEGGER_SYSVIEW_Stop()
*
*  Function description
*    Stop recording SystemView events.
*
*  Additional information
*    This function disables transmission of SystemView packets recorded
*    by subsequent trace calls.  If transmission is enabled when
*    this function is called, a single SystemView Stop event is recorded
*    to the trace, send, and then trace transmission is halted.
*/
void SEGGER_SYSVIEW_Stop(void) {
  U8  aPacket[SEGGER_SYSVIEW_INFO_SIZE];
  U8* pPayloadStart;

  if (_SYSVIEW_Globals.EnableState) {
    pPayloadStart = _PreparePacket(aPacket);
    _SendPacket(pPayloadStart, pPayloadStart, SEGGER_SYSVIEW_EVENT_ID_TRACE_STOP);
    _SYSVIEW_Globals.EnableState = 0;
  }
}

/*********************************************************************
*
*       SEGGER_SYSVIEW_GetSysDesc()
*
*  Function description
*    Triggers a send of the system information and description.
*
*/
void SEGGER_SYSVIEW_GetSysDesc(void) {
  U8  aPacket[SEGGER_SYSVIEW_INFO_SIZE + 4 * SEGGER_SYSVIEW_QUANTA_U32];
  U8* pPayload;
  U8* pPayloadStart;
  
  pPayloadStart = _PreparePacket(aPacket);
  pPayload = pPayloadStart;
  ENCODE_U32(pPayload, _SYSVIEW_Globals.SysFreq);
  ENCODE_U32(pPayload, _SYSVIEW_Globals.CPUFreq);
  ENCODE_U32(pPayload, _SYSVIEW_Globals.RAMBaseAddress);
  ENCODE_U32(pPayload, SEGGER_SYSVIEW_ID_SHIFT);
  _SendPacket(pPayloadStart, pPayload, SEGGER_SYSVIEW_EVENT_ID_INIT);
  if (_SYSVIEW_Globals.pfSendSysDesc) {
    _SYSVIEW_Globals.pfSendSysDesc();
  }
}

/*********************************************************************
*
*       SEGGER_SYSVIEW_SendTaskInfo()
*
*  Function description
*    Send a Task Info Packet, containing TaskId for identification,
*    task priority and task name.
*
*  Parameters
*    pInfo - Pointer to task information to send.
*/
void SEGGER_SYSVIEW_SendTaskInfo(const SEGGER_SYSVIEW_TASKINFO *pInfo) {
  U8  aPacket[SEGGER_SYSVIEW_INFO_SIZE + SEGGER_SYSVIEW_QUANTA_U32 + 1 + 32];
  U8* pPayload;
  U8* pPayloadStart;
  //
  pPayloadStart = _PreparePacket(aPacket);
  pPayload = pPayloadStart;
  ENCODE_U32(pPayload, SHRINK_ID(pInfo->TaskID));
  ENCODE_U32(pPayload, pInfo->Prio);
  pPayload = _EncodeStr(pPayload, pInfo->sName, 32);
  _SendPacket(pPayloadStart, pPayload, SEGGER_SYSVIEW_EVENT_ID_TASK_INFO);
  pPayloadStart = _PreparePacket(aPacket);
  pPayload = pPayloadStart;
  ENCODE_U32(pPayload, SHRINK_ID(pInfo->TaskID));
  ENCODE_U32(pPayload, pInfo->StackBase);
  ENCODE_U32(pPayload, pInfo->StackSize);
  ENCODE_U32(pPayload, 0); // Stack End, future use
  _SendPacket(pPayloadStart, pPayload, SEGGER_SYSVIEW_EVENT_ID_STACK_INFO);
}

/*********************************************************************
*
*       SEGGER_SYSVIEW_SendTaskList()
*
*  Function description
*    Send all tasks descriptors to the host.
*/
void SEGGER_SYSVIEW_SendTaskList(void) {
  if (_SYSVIEW_Globals.pOSAPI && _SYSVIEW_Globals.pOSAPI->pfSendTaskList) {
    _SYSVIEW_Globals.pOSAPI->pfSendTaskList();
  }
}

/*********************************************************************
*
*       SEGGER_SYSVIEW_SendSysDesc()
*
*  Function description
*    Send the system description string to the host.
*    The system description is used by SysViewer to identify the
*    current application and handle events accordingly.
*
*  Parameters
*    sSysDesc - Pointer to the 0-terminated system description string.
*
*  Additional information
*    One system description string may not exceed SEGGER_SYSVIEW_MAX_STRING_LEN characters.
*
*    The Following items can be described in a system description string.
*    Each item is identified by its identifier, followed by '=' and the value.
*    Items are separated by ','.
*/
void SEGGER_SYSVIEW_SendSysDesc(const char *sSysDesc) {
  U8  aPacket[SEGGER_SYSVIEW_INFO_SIZE + 1 + SEGGER_SYSVIEW_MAX_STRING_LEN];
  U8* pPayload;
  U8* pPayloadStart;

  pPayloadStart = _PreparePacket(aPacket);
  pPayload = _EncodeStr(pPayloadStart, sSysDesc, SEGGER_SYSVIEW_MAX_STRING_LEN);
  _SendPacket(pPayloadStart, pPayload, SEGGER_SYSVIEW_EVENT_ID_SYSDESC);
}

/*********************************************************************
*
*       SEGGER_SYSVIEW_RecordSystime()
*
*  Function description
*    Formats and sends a SystemView Systime containing a single U64 or U32
*    parameter payload.
*/
void SEGGER_SYSVIEW_RecordSystime(void) {
  U64 Systime;

  if (_SYSVIEW_Globals.pOSAPI && _SYSVIEW_Globals.pOSAPI->pfGetTime) {
    Systime = _SYSVIEW_Globals.pOSAPI->pfGetTime();
    SEGGER_SYSVIEW_RecordU32x2(SEGGER_SYSVIEW_EVENT_ID_SYSTIME_US,
                               (U32)(Systime),
                               (U32)(Systime >> 32));
  } else {
    SEGGER_SYSVIEW_RecordU32(SEGGER_SYSVIEW_EVENT_ID_SYSTIME_CYCLES, SEGGER_SYSVIEW_GET_TIMESTAMP());
  }
}

/*********************************************************************
*
*       SEGGER_SYSVIEW_RecordEnterISR()
*
*  Function description
*    Format and send an ISR entry event.
*
*  Additional information
*    Example packets sent
*      02 0F 50              // ISR(15) Enter. Timestamp is 80 (0x50)
*/
void SEGGER_SYSVIEW_RecordEnterISR(void) {
  U8  aPacket[SEGGER_SYSVIEW_INFO_SIZE + SEGGER_SYSVIEW_QUANTA_U32];
  U8* pPayload;
  U8* pPayloadStart;
  unsigned v;
  
  v = SEGGER_SYSVIEW_GET_INTERRUPT_ID();
  pPayloadStart = _PreparePacket(aPacket);
  pPayload = pPayloadStart;
  ENCODE_U32(pPayload, v);
  _SendPacket(pPayloadStart, pPayload, SEGGER_SYSVIEW_EVENT_ID_ISR_ENTER);
}

/*********************************************************************
*
*       SEGGER_SYSVIEW_RecordExitISR()
*
*  Function description
*    Format and send an ISR exit event.
*
*  Additional information
*    Format as follows:
*      03 <TimeStamp>        // Max. packet len is 6
*
*    Example packets sent
*      03 20                // ISR Exit. Timestamp is 32 (0x20)
*/
void SEGGER_SYSVIEW_RecordExitISR(void) {
  U8  aPacket[SEGGER_SYSVIEW_INFO_SIZE];
  U8* pPayloadStart;
  
  pPayloadStart = _PreparePacket(aPacket);
  _SendPacket(pPayloadStart, pPayloadStart, SEGGER_SYSVIEW_EVENT_ID_ISR_EXIT);
}

/*********************************************************************
*
*       SEGGER_SYSVIEW_RecordExitISRToScheduler()
*
*  Function description
*    Format and send an ISR exit into scheduler event.
*
*  Additional information
*    Format as follows:
*      18 <TimeStamp>        // Max. packet len is 6
*
*    Example packets sent
*      18 20                // ISR Exit to Scheduler. Timestamp is 32 (0x20)
*/
void SEGGER_SYSVIEW_RecordExitISRToScheduler(void) {
  U8  aPacket[SEGGER_SYSVIEW_INFO_SIZE];
  U8* pPayloadStart;

  pPayloadStart = _PreparePacket(aPacket);
  _SendPacket(pPayloadStart, pPayloadStart, SEGGER_SYSVIEW_EVENT_ID_ISR_TO_SCHEDULER);
}

/*********************************************************************
*
*       SEGGER_SYSVIEW_RecordEnterTimer()
*
*  Function description
*    Format and send a Timer entry event.
*  
*  Parameters
*    TimerId - Id of the timer which starts.
*/
void SEGGER_SYSVIEW_RecordEnterTimer(unsigned TimerId) {
  U8  aPacket[SEGGER_SYSVIEW_INFO_SIZE + SEGGER_SYSVIEW_QUANTA_U32];
  U8* pPayload;
  U8* pPayloadStart;

  pPayloadStart = _PreparePacket(aPacket);
  pPayload = pPayloadStart;
  ENCODE_U32(pPayload, SHRINK_ID(TimerId));
  _SendPacket(pPayloadStart, pPayload, SEGGER_SYSVIEW_EVENT_ID_TIMER_ENTER);
}

/*********************************************************************
*
*       SEGGER_SYSVIEW_RecordExitTimer()
*
*  Function description
*    Format and send a Timer exit event.
*/
void SEGGER_SYSVIEW_RecordExitTimer(void) {
  U8  aPacket[SEGGER_SYSVIEW_INFO_SIZE];
  U8* pPayloadStart;

  pPayloadStart = _PreparePacket(aPacket);
  _SendPacket(pPayloadStart, pPayloadStart, SEGGER_SYSVIEW_EVENT_ID_TIMER_EXIT);
}

/*********************************************************************
*
*       SEGGER_SYSVIEW_OnIdle()
*
*  Function description
*    Record an Idle event.
*/
void SEGGER_SYSVIEW_OnIdle(void) {
  U8  aPacket[SEGGER_SYSVIEW_INFO_SIZE];
  U8* pPayloadStart;

  pPayloadStart = _PreparePacket(aPacket);
  _SendPacket(pPayloadStart, pPayloadStart, SEGGER_SYSVIEW_EVENT_ID_IDLE);
}

/*********************************************************************
*
*       SEGGER_SYSVIEW_OnTaskCreate()
*
*  Function description
*    Record a Task Create event.  The Task Create event corresponds
*    to creating a task in the OS.
*
*  Parameters
*    TaskId        - Task ID of created task.
*/
void SEGGER_SYSVIEW_OnTaskCreate(unsigned TaskId) {
  U8  aPacket[SEGGER_SYSVIEW_INFO_SIZE + SEGGER_SYSVIEW_QUANTA_U32];
  U8* pPayload;
  U8* pPayloadStart;

  TaskId = SHRINK_ID(TaskId);
  pPayload = pPayloadStart = _PreparePacket(aPacket);
  ENCODE_U32(pPayload, TaskId);
  _SendPacket(pPayloadStart, pPayload, SEGGER_SYSVIEW_EVENT_ID_TASK_CREATE);
}

/*********************************************************************
*
*       SEGGER_SYSVIEW_OnTaskStartExec()
*
*  Function description
*    Record a Task Start Execution event.  The Task Start event
*    corresponds to when a task has started to execute rather than
*    when it is ready to execute.
*
*  Parameters
*    TaskId - Task ID of task that started to execute.
*/
void SEGGER_SYSVIEW_OnTaskStartExec(unsigned TaskId) {
  U8  aPacket[SEGGER_SYSVIEW_INFO_SIZE + SEGGER_SYSVIEW_QUANTA_U32];
  U8* pPayload;
  U8* pPayloadStart;

  TaskId = SHRINK_ID(TaskId);
  pPayload = pPayloadStart = _PreparePacket(aPacket);
  ENCODE_U32(pPayload, TaskId);
  _SendPacket(pPayloadStart, pPayload, SEGGER_SYSVIEW_EVENT_ID_TASK_START_EXEC);
}

/*********************************************************************
*
*       SEGGER_SYSVIEW_OnTaskStopExec()
*
*  Function description
*    Record a Task Stop Execution event.  The Task Stop event
*    corresponds to when a task stops executing.
*/
void SEGGER_SYSVIEW_OnTaskStopExec(void) {
  U8  aPacket[SEGGER_SYSVIEW_INFO_SIZE + SEGGER_SYSVIEW_QUANTA_U32];
  U8* pPayloadStart;

  pPayloadStart = _PreparePacket(aPacket);
  _SendPacket(pPayloadStart, pPayloadStart, SEGGER_SYSVIEW_EVENT_ID_TASK_STOP_EXEC);
}

/*********************************************************************
*
*       SEGGER_SYSVIEW_OnTaskStartReady()
*
*  Function description
*    Record a Task Start Ready event.
*
*  Parameters
*    TaskId - Task ID of task that started to execute.
*/
void SEGGER_SYSVIEW_OnTaskStartReady(unsigned TaskId) {
  U8  aPacket[SEGGER_SYSVIEW_INFO_SIZE + SEGGER_SYSVIEW_QUANTA_U32];
  U8* pPayload;
  U8* pPayloadStart;

  TaskId = SHRINK_ID(TaskId);
  pPayload = pPayloadStart = _PreparePacket(aPacket);
  ENCODE_U32(pPayload, TaskId);
  _SendPacket(pPayloadStart, pPayload, SEGGER_SYSVIEW_EVENT_ID_TASK_START_READY);
}

/*********************************************************************
*
*       SEGGER_SYSVIEW_OnTaskStopReady()
*
*  Function description
*    Record a Task Stop Ready event.
*
*  Parameters
*    TaskId - Task ID of task that completed execution.
*    Cause  - Reason for task to stop (i.e. Idle/Sleep)
*/
void SEGGER_SYSVIEW_OnTaskStopReady(unsigned TaskId, unsigned Cause) {
  U8  aPacket[SEGGER_SYSVIEW_INFO_SIZE + 2 * SEGGER_SYSVIEW_QUANTA_U32];
  U8* pPayload;
  U8* pPayloadStart;

  TaskId = SHRINK_ID(TaskId);
  pPayload = pPayloadStart = _PreparePacket(aPacket);
  ENCODE_U32(pPayload, TaskId);
  ENCODE_U32(pPayload, Cause);
  _SendPacket(pPayloadStart, pPayload, SEGGER_SYSVIEW_EVENT_ID_TASK_STOP_READY);
}

/*********************************************************************
*
*       SEGGER_SYSVIEW_OnUserStart()
*
*  Function description
*    Send a user event start, such as start of a subroutine for profiling.
*
*  Parameters
*    UserId  - User defined ID for the event.
*/
void SEGGER_SYSVIEW_OnUserStart(unsigned UserId) {
  U8  aPacket[SEGGER_SYSVIEW_INFO_SIZE + SEGGER_SYSVIEW_QUANTA_U32];
  U8* pPayload;
  U8* pPayloadStart;

  pPayload = pPayloadStart = _PreparePacket(aPacket);
  ENCODE_U32(pPayload, UserId);
  _SendPacket(pPayloadStart, pPayload, SEGGER_SYSVIEW_EVENT_ID_USER_START);
}

/*********************************************************************
*
*       SEGGER_SYSVIEW_OnUserStop()
*
*  Function description
*    Send a user event stop, such as return of a subroutine for profiling.
*
*  Parameters
*    UserId  - User defined ID for the event.
*/
void SEGGER_SYSVIEW_OnUserStop(unsigned UserId) {
  U8   aPacket[SEGGER_SYSVIEW_INFO_SIZE + SEGGER_SYSVIEW_QUANTA_U32];
  U8 * pPayload;
  U8 * pPayloadStart;

  pPayload = pPayloadStart = _PreparePacket(aPacket);
  ENCODE_U32(pPayload, UserId);
  _SendPacket(pPayloadStart, pPayload, SEGGER_SYSVIEW_EVENT_ID_USER_STOP);
}

/*********************************************************************
*
*       SEGGER_SYSVIEW_NameResource()
*
*  Function description
*    Send the name of a resource to be displayed in SysViewer.
*
*  Parameters
*    ResourceId - Id of the resource to be named. i.e. its address.
*    sName      - Pointer to the resource name. (Max. SEGGER_SYSVIEW_MAX_STRING_LEN Bytes)
*/
void SEGGER_SYSVIEW_NameResource(U32 ResourceId, const char* sName) {
  U8  aPacket[SEGGER_SYSVIEW_INFO_SIZE + SEGGER_SYSVIEW_QUANTA_U32 + 1 + SEGGER_SYSVIEW_MAX_STRING_LEN];
  U8* pPayload;
  U8* pPayloadStart;

  pPayload = pPayloadStart = _PreparePacket(aPacket);
  ENCODE_U32(pPayload, SHRINK_ID(ResourceId));
  pPayload = _EncodeStr(pPayload, sName, SEGGER_SYSVIEW_MAX_STRING_LEN);
  _SendPacket(pPayloadStart, pPayload, SEGGER_SYSVIEW_EVENT_ID_NAME_RESOURCE);
}

/*********************************************************************
*
*       SEGGER_SYSVIEW_SendPacket()
*
*  Function description
*    Send an event packet.
*
*  Parameters
*    pPacket      - Pointer to the start of the packet.
*    pPayloadEnd  - Pointer to the end of the payload.
*                   Make sure there are at least 5 bytes free after the payload.
*    EventId      - Id of the event packet.
*
*  Return value
*    !=0:  Success, Message sent.
*    ==0:  Buffer full, Message *NOT* sent.
*/
int SEGGER_SYSVIEW_SendPacket(U8* pPacket, U8* pPayloadEnd, unsigned EventId) {
  _SendPacket(pPacket + 4, pPayloadEnd, EventId);
  return 0;
}

/*********************************************************************
*
*       SEGGER_SYSVIEW_EncodeU32()
*
*  Function description
*    Encode a U32 in variable-length format.
*
*  Parameters
*    pPayload - Pointer to where U32 will be encoded.
*    Value    - The 32-bit value to be encoded.
*
*  Return value
*    Pointer to the byte following the value, i.e. the first free
*    byte in the payload and the next position to store payload
*    content.
*/
U8* SEGGER_SYSVIEW_EncodeU32(U8* pPayload, unsigned Value) {
  ENCODE_U32(pPayload, Value);
  return pPayload;
}

/*********************************************************************
*
*       SEGGER_SYSVIEW_EncodeString()
*
*  Function description
*    Encode a string in variable-length format.
*
*  Parameters
*    pPayload - Pointer to where string will be encoded.
*    s        - String to encode.
*    MaxLen   - Maximum number of characters to encode from string.
*
*  Return value
*    Pointer to the byte following the value, i.e. the first free
*    byte in the payload and the next position to store payload
*    content.
*
*  Additional information
*    The string is encoded as a count byte followed by the contents
*    of the string.
*    No more than 1 + MaxLen bytes will be encoded to the payload.
*/
U8* SEGGER_SYSVIEW_EncodeString(U8* pPayload, const char* s, unsigned MaxLen) {
  return _EncodeStr(pPayload, s, MaxLen);
}

/*********************************************************************
*
*       SEGGER_SYSVIEW_EncodeData()
*
*  Function description
*    Encode a byte buffer in variable-length format.
*
*  Parameters
*    pPayload - Pointer to where string will be encoded.
*    pSrc     - Pointer to data buffer to be encoded.
*    NumBytes - Number of bytes in the buffer to be encoded.
*
*  Return value
*    Pointer to the byte following the value, i.e. the first free
*    byte in the payload and the next position to store payload
*    content.
*
*  Additional information
*    The data is encoded as a count byte followed by the contents
*    of the data buffer.
*    Make sure NumBytes + 1 bytes are free for the payload.
*/
U8* SEGGER_SYSVIEW_EncodeData(U8 *pPayload, const char* pSrc, unsigned NumBytes) {
  return _EncodeData(pPayload, pSrc, NumBytes);
}

/*********************************************************************
*
*       SEGGER_SYSVIEW_EncodeId()
*
*  Function description
*    Encode a 32-bit Id in shrunken variable-length format.
*
*  Parameters
*    pPayload - Pointer to where the Id will be encoded.
*    Id       - The 32-bit value to be encoded.
*
*  Return value
*    Pointer to the byte following the value, i.e. the first free
*    byte in the payload and the next position to store payload
*    content.
*
*  Additional information
*    The parameters to shrink an Id can be configured in
*    SEGGER_SYSVIEW_Conf.h and via SEGGER_SYSVIEW_SetRAMBase().
*     SEGGER_SYSVIEW_ID_BASE: Lowest Id reported by the application.
*       (i.e. 0x20000000 when all Ids are an address in this RAM)
*     SEGGER_SYSVIEW_ID_SHIFT: Number of bits to shift the Id to
*       save bandwidth. (i.e. 2 when Ids are 4 byte aligned)
*/
U8* SEGGER_SYSVIEW_EncodeId(U8* pPayload, unsigned Id) {
  Id = SHRINK_ID(Id);
  ENCODE_U32(pPayload, Id);
  return pPayload;
}

/*********************************************************************
*
*       SEGGER_SYSVIEW_ShrinkId()
*
*  Function description
*    Get the shrunken value of an Id for further processing like in
*    SEGGER_SYSVIEW_NameResource().
*
*  Parameters
*    Id       - The 32-bit value to be shrunken.
*
*  Return value
*    Shrunken Id.
*
*  Additional information
*    The parameters to shrink an Id can be configured in
*    SEGGER_SYSVIEW_Conf.h and via SEGGER_SYSVIEW_SetRAMBase().
*     SEGGER_SYSVIEW_ID_BASE: Lowest Id reported by the application.
*       (i.e. 0x20000000 when all Ids are an address in this RAM)
*     SEGGER_SYSVIEW_ID_SHIFT: Number of bits to shift the Id to
*       save bandwidth. (i.e. 2 when Ids are 4 byte aligned)
*/
unsigned SEGGER_SYSVIEW_ShrinkId(unsigned Id) {
  return SHRINK_ID(Id);
}

/*********************************************************************
*
*       SEGGER_SYSVIEW_RegisterModule()
*
*  Function description
*    Register a middleware module for recording its events.
*
*  Parameters
*    pModule  - The middleware module information.
*
*  Additional information
*    pModule->sDescription      - Pointer to a string containing the module
*                                 name and optionally the module event 
*                                 description.
*    pModule->NumEvents         - Number of events the module wants to 
*                                 register.
*    pModule->EventOffset       - Offset to be added to the event Ids.
*                                 Out parameter, set by this function.
*                                 Do not modify after calling this function.
*    pModule->pfSendModuleDesc  - Callback function pointer to send 
*                                 more detailed module description
*                                 to SystemViewer.
*    pModule->pNext             - Pointer to next registered module.
*                                 Out parameter, set by this function.
*                                 Do not modify after calling this function.
*/
void SEGGER_SYSVIEW_RegisterModule(SEGGER_SYSVIEW_MODULE* pModule) {
  SEGGER_SYSVIEW_LOCK();
  if (_pFirstModule == 0) {
    //
    // No module registered, yet.
    // Start list with new module.
    // EventOffset is the base offset for modules
    //
    pModule->EventOffset = MODULE_EVENT_OFFSET;
    pModule->pNext = 0;
    _pFirstModule = pModule;
    _NumModules = 1;
  } else {
    //
    // Registreded module(s) present.
    // Prepend new module in list.
    // EventOffset set from number of events and offset of previous module.
    //
    pModule->EventOffset = _pFirstModule->EventOffset + _pFirstModule->NumEvents;
    pModule->pNext = _pFirstModule;
    _pFirstModule = pModule;
    _NumModules++;
  }
  SEGGER_SYSVIEW_SendModule(0);
  if (pModule->pfSendModuleDesc) {
    pModule->pfSendModuleDesc();
  }
  SEGGER_SYSVIEW_UNLOCK();
}

/*********************************************************************
*
*       SEGGER_SYSVIEW_RecordModuleDescription()
*
*  Function description
*    Sends detailed information of a registered module to the host.
*
*  Parameters
*    pModule      - Pointer to the described module.
*    sDescription - Pointer to a description string.
*/
void SEGGER_SYSVIEW_RecordModuleDescription(const SEGGER_SYSVIEW_MODULE* pModule, const char* sDescription) {
  U8  aPacket[SEGGER_SYSVIEW_INFO_SIZE + 2 * SEGGER_SYSVIEW_QUANTA_U32 + 1 + SEGGER_SYSVIEW_MAX_STRING_LEN];
  U8* pPayload;
  U8* pPayloadStart;
  U8  ModuleId;
  SEGGER_SYSVIEW_MODULE* p;

  p = _pFirstModule;
  ModuleId = 0;
  do {
    if (p == pModule) {
      break;
    }
    ModuleId++;
    p = p->pNext;
  } while (p);
  //
  // Send module description
  // Send event offset and number of events
  //
  pPayload = pPayloadStart = _PreparePacket(aPacket);
  ENCODE_U32(pPayload, ModuleId);
  ENCODE_U32(pPayload, (pModule->EventOffset));
  pPayload = _EncodeStr(pPayload, sDescription, SEGGER_SYSVIEW_MAX_STRING_LEN);
  _SendPacket(pPayloadStart, pPayload, SEGGER_SYSVIEW_EVENT_ID_MODULEDESC);
}

/*********************************************************************
*
*       SEGGER_SYSVIEW_SendModule()
*
*  Function description
*    Sends the information of a registered module to the host.
*
*  Parameters
*    ModuleId   - Id of the requested module.
*/
void SEGGER_SYSVIEW_SendModule(U8 ModuleId) {
  SEGGER_SYSVIEW_MODULE* pModule;
  U32 n;

  if (_pFirstModule != 0) {
    pModule = _pFirstModule;
    for (n = 0; n < ModuleId; n++) {
      pModule = pModule->pNext;
      if (pModule == 0) {
        break;
      }
    }
    if (pModule != 0) {
      //
      // Send module description
      // Send event offset and number of events
      //
      U8  aPacket[SEGGER_SYSVIEW_INFO_SIZE + 2 * SEGGER_SYSVIEW_QUANTA_U32 + 1 + SEGGER_SYSVIEW_MAX_STRING_LEN];
      U8* pPayload;
      U8* pPayloadStart;

      pPayload = pPayloadStart = _PreparePacket(aPacket);
      ENCODE_U32(pPayload, ModuleId);
      ENCODE_U32(pPayload, (pModule->EventOffset));
      pPayload = _EncodeStr(pPayload, pModule->sModule, SEGGER_SYSVIEW_MAX_STRING_LEN);
      _SendPacket(pPayloadStart, pPayload, SEGGER_SYSVIEW_EVENT_ID_MODULEDESC);
    }
  }
}

/*********************************************************************
*
*       SEGGER_SYSVIEW_SendModuleDescription()
*
*  Function description
*    Triggers a send of the registered module descriptions.
*
*/
void SEGGER_SYSVIEW_SendModuleDescription(void) {
  SEGGER_SYSVIEW_MODULE* pModule;

  if (_pFirstModule != 0) {
    pModule = _pFirstModule;
    do {
      if (pModule->pfSendModuleDesc) {
        pModule->pfSendModuleDesc();
      }
      pModule = pModule->pNext;
    } while (pModule);
  }
}

/*********************************************************************
*
*       SEGGER_SYSVIEW_SendNumModules()
*
*  Function description
*    Send the number of registered modules to the host.
*/
void SEGGER_SYSVIEW_SendNumModules(void) {
    U8  aPacket[SEGGER_SYSVIEW_INFO_SIZE + 2*SEGGER_SYSVIEW_QUANTA_U32];
    U8* pPayload;
    U8* pPayloadStart;

    pPayload = pPayloadStart = _PreparePacket(aPacket);
    ENCODE_U32(pPayload, _NumModules);
    _SendPacket(pPayloadStart, pPayload, SEGGER_SYSVIEW_EVENT_ID_NUMMODULES);
}

#ifndef SEGGER_SYSVIEW_EXCLUDE_PRINTF // Define in project to avoid warnings about variable parameter list

/*********************************************************************
*
*       SEGGER_SYSVIEW_PrintfHostEx()
*
*  Function description
*    Print a string which is formatted on the host by SystemViewer
*    with Additional information.
*
*  Parameters
*    s        - String to be formatted.
*    Options  - Options for the string. i.e. Log level.
*
*  Additional information
*    All format arguments are treated as 32-bit scalar values.
*/
void SEGGER_SYSVIEW_PrintfHostEx(const char* s, U32 Options, ...) {
  va_list ParamList;

  va_start(ParamList, Options);
  _VPrintHost(s, Options, &ParamList);
  va_end(ParamList);
}

/*********************************************************************
*
*       SEGGER_SYSVIEW_PrintfHost()
*
*  Function description
*    Print a string which is formatted on the host by SystemViewer.
*
*  Parameters
*    s        - String to be formatted.
*
*  Additional information
*    All format arguments are treated as 32-bit scalar values.
*/
void SEGGER_SYSVIEW_PrintfHost(const char* s, ...) {
  va_list ParamList;

  va_start(ParamList, s);
  _VPrintHost(s, SEGGER_SYSVIEW_LOG, &ParamList);
  va_end(ParamList);
}

/*********************************************************************
*
*       SEGGER_SYSVIEW_WarnfHost()
*
*  Function description
*    Print a warnin string which is formatted on the host by 
*    SystemViewer.
*
*  Parameters
*    s        - String to be formatted.
*
*  Additional information
*    All format arguments are treated as 32-bit scalar values.
*/
void SEGGER_SYSVIEW_WarnfHost(const char* s, ...) {
  va_list ParamList;

  va_start(ParamList, s);
  _VPrintHost(s, SEGGER_SYSVIEW_WARNING, &ParamList);
  va_end(ParamList);
}

/*********************************************************************
*
*       SEGGER_SYSVIEW_ErrorfHost()
*
*  Function description
*    Print an error string which is formatted on the host by 
*    SystemViewer.
*
*  Parameters
*    s        - String to be formatted.
*
*  Additional information
*    All format arguments are treated as 32-bit scalar values.
*/
void SEGGER_SYSVIEW_ErrorfHost(const char* s, ...) {
  va_list ParamList;

  va_start(ParamList, s);
  _VPrintHost(s, SEGGER_SYSVIEW_ERROR, &ParamList);
  va_end(ParamList);
}

/*********************************************************************
*
*       SEGGER_SYSVIEW_PrintfTargetEx()
*
*  Function description
*    Print a string which is formatted on the target before sent to 
*    the host with Additional information.
*
*  Parameters
*    s        - String to be formatted.
*    Options  - Options for the string. i.e. Log level.
*/
void SEGGER_SYSVIEW_PrintfTargetEx(const char* s, U32 Options, ...) {
  va_list ParamList;

  va_start(ParamList, Options);
  _VPrintTarget(s, Options, &ParamList);
  va_end(ParamList);
}

/*********************************************************************
*
*       SEGGER_SYSVIEW_PrintfTarget()
*
*  Function description
*    Print a string which is formatted on the target before sent to 
*    the host.
*
*  Parameters
*    s        - String to be formatted.
*/
void SEGGER_SYSVIEW_PrintfTarget(const char* s, ...) {
  va_list ParamList;

  va_start(ParamList, s);
  _VPrintTarget(s, SEGGER_SYSVIEW_LOG, &ParamList);
  va_end(ParamList);
}

/*********************************************************************
*
*       SEGGER_SYSVIEW_WarnfTarget()
*
*  Function description
*    Print a warning string which is formatted on the target before
*    sent to the host.
*
*  Parameters
*    s        - String to be formatted.
*/
void SEGGER_SYSVIEW_WarnfTarget(const char* s, ...) {
  va_list ParamList;

  va_start(ParamList, s);
  _VPrintTarget(s, SEGGER_SYSVIEW_WARNING, &ParamList);
  va_end(ParamList);
}

/*********************************************************************
*
*       SEGGER_SYSVIEW_ErrorfTarget()
*
*  Function description
*    Print an error string which is formatted on the target before
*    sent to the host.
*
*  Parameters
*    s        - String to be formatted.
*/
void SEGGER_SYSVIEW_ErrorfTarget(const char* s, ...) {
  va_list ParamList;

  va_start(ParamList, s);
  _VPrintTarget(s, SEGGER_SYSVIEW_ERROR, &ParamList);
  va_end(ParamList);
}
#endif // SEGGER_SYSVIEW_EXCLUDE_PRINTF

/*********************************************************************
*
*       SEGGER_SYSVIEW_Print()
*
*  Function description
*    Print a string to the host.
*
*  Parameters
*    s        - String to sent.
*/
void SEGGER_SYSVIEW_Print(const char* s) {
  U8  aPacket[SEGGER_SYSVIEW_INFO_SIZE + 2 * SEGGER_SYSVIEW_QUANTA_U32 + SEGGER_SYSVIEW_MAX_STRING_LEN];
  U8* pPayload;
  U8* pPayloadStart;

  pPayloadStart = _PreparePacket(aPacket);
  pPayload = _EncodeStr(pPayloadStart, s, SEGGER_SYSVIEW_MAX_STRING_LEN);
  ENCODE_U32(pPayload, SEGGER_SYSVIEW_LOG);
  ENCODE_U32(pPayload, 0);
  _SendPacket(pPayloadStart, pPayload, SEGGER_SYSVIEW_EVENT_ID_PRINT_FORMATTED);
}

/*********************************************************************
*
*       SEGGER_SYSVIEW_Warn()
*
*  Function description
*    Print a warning string to the host.
*
*  Parameters
*    s        - String to sent.
*/
void SEGGER_SYSVIEW_Warn(const char* s) {
  U8  aPacket[SEGGER_SYSVIEW_INFO_SIZE + 2 * SEGGER_SYSVIEW_QUANTA_U32 + SEGGER_SYSVIEW_MAX_STRING_LEN];
  U8* pPayload;
  U8* pPayloadStart;

  pPayloadStart = _PreparePacket(aPacket);
  pPayload = _EncodeStr(pPayloadStart, s, SEGGER_SYSVIEW_MAX_STRING_LEN);
  ENCODE_U32(pPayload, SEGGER_SYSVIEW_WARNING);
  ENCODE_U32(pPayload, 0);
  _SendPacket(pPayloadStart, pPayload, SEGGER_SYSVIEW_EVENT_ID_PRINT_FORMATTED);
}

/*********************************************************************
*
*       SEGGER_SYSVIEW_Error()
*
*  Function description
*    Print an error string to the host.
*
*  Parameters
*    s        - String to sent.
*/
void SEGGER_SYSVIEW_Error(const char* s) {
  U8  aPacket[SEGGER_SYSVIEW_INFO_SIZE + 2 * SEGGER_SYSVIEW_QUANTA_U32 + SEGGER_SYSVIEW_MAX_STRING_LEN];
  U8* pPayload;
  U8* pPayloadStart;

  pPayloadStart = _PreparePacket(aPacket);
  pPayload = _EncodeStr(pPayloadStart, s, SEGGER_SYSVIEW_MAX_STRING_LEN);
  ENCODE_U32(pPayload, SEGGER_SYSVIEW_ERROR);
  ENCODE_U32(pPayload, 0);
  _SendPacket(pPayloadStart, pPayload, SEGGER_SYSVIEW_EVENT_ID_PRINT_FORMATTED);
}

/****** End Of File *************************************************/
