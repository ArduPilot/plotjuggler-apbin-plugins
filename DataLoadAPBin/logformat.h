/**
 * @file
 * @author Pierre Kancir <pierre.kancir.emn@gmail.com>
 * @author Jonas Withelm <IAV GmbH>
 * 
 * @section DESCRIPTION
 *
 * ArduPilot DataFlash binaries loader for Plotjuggler.
 * This load and decode ArduPilot DataFlash binaries to extract logging information in readable and plotable state.
 * The logic is derived from Dronekit-La software (https://github.com/dronekit/dronekit-la).
 *
 */

#pragma once
#include <cstdint>


/*
The contents of this file were mainly taken from the ArduPilot source code:
  - https://github.com/ArduPilot/ardupilot/

For more details please have a look at:
  - https://github.com/ArduPilot/ardupilot/tree/master/libraries/AP_Logger

If the ArduPilot source code for the logging mechanism is changed it may be necessary
to update the content of this file!
*/



/*
An ArduPilot logfile consists of messages.
The format message (FMT) is the base message which defines the content of any other message.
A message is structured as follows:

  Each message begins with a header
    - The header consists of the header identification sequence and the message id
      - identification sequence:  HEAD_BYTE1, HEAD_BYTE2
      - message id:               unique number of the message (uint8_t, therefore a maximum of 255 different message types)
    - A message can contain up to 16 fields, in which the logging data is stored
    - Usually, the first field of a message is time
*/



/*
  The format message (FMT) has a hardcoded message id of 128
    - file: libraries/AP_Logger/LogStructure.h (commit: b80cc9a)
    - line: 1446
*/
static constexpr uint8_t LOG_FORMAT_MSG = 128;



/*
  These are two bytes for the header identification sequence
    - file: libraries/AP_Logger/LogStructure.h (commit: b80cc9a)
    - line: 119 - 120
*/
static constexpr uint8_t HEAD_BYTE1 = 0xA3;    // Decimal 163
static constexpr uint8_t HEAD_BYTE2 = 0x95;    // Decimal 149



/*
  Message header definition
    - file: libraries/AP_Logger/LogStructure.h (commit: b80cc9a)
    - line: 113 - 115
*/
#define LOG_PACKET_HEADER	       uint8_t head1, head2, msgid;
//#define LOG_PACKET_HEADER_INIT(id) head1 : HEAD_BYTE1, head2 : HEAD_BYTE2, msgid : id
#define LOG_PACKET_HEADER_LEN 3 // bytes required for LOG_PACKET_HEADER



/*
  If you need to change this section, please also fix dataload_apbin.cpp (handle_message_received)!
  AP_Logger: Format Types (https://github.com/ArduPilot/ardupilot/tree/master/libraries/AP_Logger#format-types)
    - file: libraries/AP_Logger/LogStructure.h (commit: b80cc9a)
    - line: 9 - 28
*/
const std::map<char, uint16_t> format_types =
{
  {'a', sizeof(int16_t[32])},
  {'b', sizeof(int8_t)},
  {'B', sizeof(uint8_t)},
  {'h', sizeof(int16_t)},
  {'H', sizeof(uint16_t)},
  {'i', sizeof(int32_t)},
  {'I', sizeof(uint32_t)},
  {'f', sizeof(float)},
  {'d', sizeof(double)},
  {'n', sizeof(char[4])},
  {'N', sizeof(char[16])},
  {'Z', sizeof(char[64])},
  {'c', sizeof(int16_t)},
  {'C', sizeof(uint16_t)},
  {'e', sizeof(int32_t)},  
  {'E', sizeof(uint32_t)},
  {'L', sizeof(int32_t)},   // latitude/longitude
  {'M', sizeof(uint8_t)},   // flight mode  
  {'q', sizeof(int64_t)},
  {'Q', sizeof(uint64_t)}
};


/*
  Based on:
    - file: libraries/AP_Logger/LogStructure.h (commit: b80cc9a)
    - line: 150 - 155
*/
static constexpr uint8_t MAX_NAME_SIZE = 5-1;           // message name, max 4 chars
static constexpr uint8_t MAX_FORMAT_SIZE = 17-1;        // max 16 fields per message
static constexpr uint8_t MAX_LABELS_SIZE = 65-1;        // max 16 fields per message, field names max 64 chars in total
static constexpr uint8_t MAX_UNITS_SIZE = 17-1;         // max 16 fields per message
static constexpr uint8_t MAX_MULTIPLIERS_SIZE = 17-1;   // max 16 fields per message



/*
  FMT - format
  - The FMT-message is the base message which defines the content of any other message
  - The FMT-message has a hardcoded message id of LOG_FORMAT_MSG (128)
  - message name and message id are defined here
    - file: libraries/AP_Logger/LogStructure.h (commit: b80cc9a)
    - line: 160 - 167
*/
struct log_Format {
  LOG_PACKET_HEADER;
  uint8_t type;                   // message id
  uint8_t length;
  char name[MAX_NAME_SIZE];       // message name           example: "PIDR"
  char format[MAX_FORMAT_SIZE];   // format                 example: "QfffffffffB"
  char labels[MAX_LABELS_SIZE];   // label (field names)    example: "TimeUS,Tar,Act,Err,P,I,D,FF,Dmod,SRate,Limit"
} __attribute__((packed));



/*
  FMTU - format unit
  - The FMTU-message defines units and multipliers for the fields of a message
  - units and multipliers are encoded as characters
    - file: libraries/AP_Logger/LogStructure.h (commit: b80cc9a)
    - line: 183 - 188
*/
struct log_Format_Units {
  LOG_PACKET_HEADER;
  uint64_t time_us;
  uint8_t format_type;
  char units[MAX_UNITS_SIZE];               // units        example: "s----------"
  char multipliers[MAX_MULTIPLIERS_SIZE];   // multipliers  example: "F----------"
} __attribute__((packed));