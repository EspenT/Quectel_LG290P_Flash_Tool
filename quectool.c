/*
  quectool.c

  February 23, 2026
  SparkFun Electronics
  Nathan Seidle

  quectool is a command-line tool to update LG290P GNSS firmware.

  To compile for Windows: gcc quectool.c -o quectool.exe -lserialport -lsetupapi -lcfgmgr32 -static
  To compile for macOS:   make   (requires: brew install libserialport)
  To compile for Linux:   gcc quectool.c -o quectool -lserialport
*/

#define TOOL_VERSION "0.0.1"

#include <errno.h>
#include <fcntl.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

/* libserialport provides a cross‑platform serial API that works on Linux,
   macOS and Windows.  The old termios/select code has been replaced with
   calls into this library. */
#include <libserialport.h>

//----------------------------------------
// New types
//----------------------------------------

typedef struct _COMMAND_OPTION
{
    bool *_optionBoolean;
    const char *_optionString;
    const char *_helpText;
} COMMAND_OPTION;

//----------------------------------------
// Constants
//----------------------------------------

#define MAX_PACKET_SIZE 4096

#define BAIL_WITH_SUCCESS 0x8000000

// Firmware upload states
enum GNSS_FIRMWARE_UPLOAD_STATES
{
    DCFUS_POWER_ON = 0,
    DCFUS_SYNC,
    DCFUS_BOOT_VERSION,
    DCFUS_FIRMWARE_INFO,
    DCFUS_FIRMWARE_ERASE,
    DCFUS_FIRMWARE_UPLOAD,
    DCFUS_FIRMWARE_RESET,

    // Add new states above this line
    DCFUS_MAX
};

#define nullptr NULL

const uint32_t crc32_table[256] = {
    0x00000000, 0x77073096, 0xee0e612c, 0x990951ba, 0x076dc419, 0x706af48f, 0xe963a535, 0x9e6495a3, 0x0edb8832,
    0x79dcb8a4, 0xe0d5e91e, 0x97d2d988, 0x09b64c2b, 0x7eb17cbd, 0xe7b82d07, 0x90bf1d91, 0x1db71064, 0x6ab020f2,
    0xf3b97148, 0x84be41de, 0x1adad47d, 0x6ddde4eb, 0xf4d4b551, 0x83d385c7, 0x136c9856, 0x646ba8c0, 0xfd62f97a,
    0x8a65c9ec, 0x14015c4f, 0x63066cd9, 0xfa0f3d63, 0x8d080df5, 0x3b6e20c8, 0x4c69105e, 0xd56041e4, 0xa2677172,
    0x3c03e4d1, 0x4b04d447, 0xd20d85fd, 0xa50ab56b, 0x35b5a8fa, 0x42b2986c, 0xdbbbc9d6, 0xacbcf940, 0x32d86ce3,
    0x45df5c75, 0xdcd60dcf, 0xabd13d59, 0x26d930ac, 0x51de003a, 0xc8d75180, 0xbfd06116, 0x21b4f4b5, 0x56b3c423,
    0xcfba9599, 0xb8bda50f, 0x2802b89e, 0x5f058808, 0xc60cd9b2, 0xb10be924, 0x2f6f7c87, 0x58684c11, 0xc1611dab,
    0xb6662d3d, 0x76dc4190, 0x01db7106, 0x98d220bc, 0xefd5102a, 0x71b18589, 0x06b6b51f, 0x9fbfe4a5, 0xe8b8d433,
    0x7807c9a2, 0x0f00f934, 0x9609a88e, 0xe10e9818, 0x7f6a0dbb, 0x086d3d2d, 0x91646c97, 0xe6635c01, 0x6b6b51f4,
    0x1c6c6162, 0x856530d8, 0xf262004e, 0x6c0695ed, 0x1b01a57b, 0x8208f4c1, 0xf50fc457, 0x65b0d9c6, 0x12b7e950,
    0x8bbeb8ea, 0xfcb9887c, 0x62dd1ddf, 0x15da2d49, 0x8cd37cf3, 0xfbd44c65, 0x4db26158, 0x3ab551ce, 0xa3bc0074,
    0xd4bb30e2, 0x4adfa541, 0x3dd895d7, 0xa4d1c46d, 0xd3d6f4fb, 0x4369e96a, 0x346ed9fc, 0xad678846, 0xda60b8d0,
    0x44042d73, 0x33031de5, 0xaa0a4c5f, 0xdd0d7cc9, 0x5005713c, 0x270241aa, 0xbe0b1010, 0xc90c2086, 0x5768b525,
    0x206f85b3, 0xb966d409, 0xce61e49f, 0x5edef90e, 0x29d9c998, 0xb0d09822, 0xc7d7a8b4, 0x59b33d17, 0x2eb40d81,
    0xb7bd5c3b, 0xc0ba6cad, 0xedb88320, 0x9abfb3b6, 0x03b6e20c, 0x74b1d29a, 0xead54739, 0x9dd277af, 0x04db2615,
    0x73dc1683, 0xe3630b12, 0x94643b84, 0x0d6d6a3e, 0x7a6a5aa8, 0xe40ecf0b, 0x9309ff9d, 0x0a00ae27, 0x7d079eb1,
    0xf00f9344, 0x8708a3d2, 0x1e01f268, 0x6906c2fe, 0xf762575d, 0x806567cb, 0x196c3671, 0x6e6b06e7, 0xfed41b76,
    0x89d32be0, 0x10da7a5a, 0x67dd4acc, 0xf9b9df6f, 0x8ebeeff9, 0x17b7be43, 0x60b08ed5, 0xd6d6a3e8, 0xa1d1937e,
    0x38d8c2c4, 0x4fdff252, 0xd1bb67f1, 0xa6bc5767, 0x3fb506dd, 0x48b2364b, 0xd80d2bda, 0xaf0a1b4c, 0x36034af6,
    0x41047a60, 0xdf60efc3, 0xa867df55, 0x316e8eef, 0x4669be79, 0xcb61b38c, 0xbc66831a, 0x256fd2a0, 0x5268e236,
    0xcc0c7795, 0xbb0b4703, 0x220216b9, 0x5505262f, 0xc5ba3bbe, 0xb2bd0b28, 0x2bb45a92, 0x5cb36a04, 0xc2d7ffa7,
    0xb5d0cf31, 0x2cd99e8b, 0x5bdeae1d, 0x9b64c2b0, 0xec63f226, 0x756aa39c, 0x026d930a, 0x9c0906a9, 0xeb0e363f,
    0x72076785, 0x05005713, 0x95bf4a82, 0xe2b87a14, 0x7bb12bae, 0x0cb61b38, 0x92d28e9b, 0xe5d5be0d, 0x7cdcefb7,
    0x0bdbdf21, 0x86d3d2d4, 0xf1d4e242, 0x68ddb3f8, 0x1fda836e, 0x81be16cd, 0xf6b9265b, 0x6fb077e1, 0x18b74777,
    0x88085ae6, 0xff0f6a70, 0x66063bca, 0x11010b5c, 0x8f659eff, 0xf862ae69, 0x616bffd3, 0x166ccf45, 0xa00ae278,
    0xd70dd2ee, 0x4e048354, 0x3903b3c2, 0xa7672661, 0xd06016f7, 0x4969474d, 0x3e6e77db, 0xaed16a4a, 0xd9d65adc,
    0x40df0b66, 0x37d83bf0, 0xa9bcae53, 0xdebb9ec5, 0x47b2cf7f, 0x30b5ffe9, 0xbdbdf21c, 0xcabac28a, 0x53b39330,
    0x24b4a3a6, 0xbad03605, 0xcdd70693, 0x54de5729, 0x23d967bf, 0xb3667a2e, 0xc4614ab8, 0x5d681b02, 0x2a6f2b94,
    0xb40bbe37, 0xc30c8ea1, 0x5a05df1b, 0x2d02ef8d};

//----------------------------------------
// Globals
//----------------------------------------

size_t commandResponseLength;

struct sp_port *comPort; // Handle for the libserialport port object

static int peek_byte_available = 0;
static uint8_t peek_byte;

bool eraseOnly;
int firmware;
uint32_t firmwareCrc32;
off_t firmwareLength;
uint8_t *firmwarePackage;
bool firmwareUpdateEnabled = true;
int32_t packetCount;
int32_t packetNumber;
uint32_t pollTimeoutUsec;
uint8_t response[8192];
size_t responseLength;
bool skipVersionCheck;
int state;
int timeoutCount;
char *timeoutMessage;

//----------------------------------------
// Helper routines
//----------------------------------------

//----------------------------------------
// Print a simple textual progress bar (current and total are 1-based counts)
//----------------------------------------
static void printProgress(int current, int total)
{
    const int width = 40; // Characters in bar
    int filled = (current * width) / total;
    int percent = (current * 100) / total;

    // Print progress bar on a new line
    printf("[");
    for (int i = 0; i < filled; i++)
        putchar('=');
    for (int i = filled; i < width; i++)
        putchar(' ');
    printf("] %3d%%\r\n", percent);
    fflush(stdout);
}

//-------------------------------------------------------------------------
// Configure the serial port
// baudRate is an actual baud (115200, 460800, etc.)
//-------------------------------------------------------------------------
int configureComPort(int baudRate)
{
    enum sp_return r;

    // Set baud rate
    r = sp_set_baudrate(comPort, baudRate);
    if (r != SP_OK)
    {
        fprintf(stderr, "ERROR: failed to set baud rate %d (%s)\n", baudRate, sp_last_error_message());
        return -1;
    }

    sp_set_bits(comPort, 8);
    sp_set_parity(comPort, SP_PARITY_NONE);
    sp_set_stopbits(comPort, 1);
    sp_set_flowcontrol(comPort, SP_FLOWCONTROL_NONE);

    return 0;
}

//-------------------------------------------------------------------------
// Write some data to the GNSS device
//-------------------------------------------------------------------------
int writeData(const uint8_t *command, ssize_t length)
{
    int written;
    unsigned int timeoutMs = 1000;

    while (length > 0)
    {
        written = sp_blocking_write(comPort, command, length, timeoutMs);
        if (written < 0)
            return errno;
        if (written == 0)
            break; // Timeout
        length -= written;
        command += written;
    }
    return 0;
}

//----------------------------------------
// Helper routines for serial I/O
//----------------------------------------
// These wrap libserialport blocking calls and provide a one-byte
// push-back buffer.

// Wait for a byte to arrive (or timeout).  Returns 1 on success,
// 0 for timeout, -1 on error (errno set).
int serialWaitByte(uint8_t *byte, unsigned int timeoutMs)
{
    int r;

    r = sp_blocking_read(comPort, byte, 1, timeoutMs);
    if (r < 0)
        return -1;
    if (r == 0)
        return 0;
    return 1;
}

// Read a byte, using the peek buffer if necessary.  Same return
// semantics as serialWaitByte.
ssize_t readSerialByte(uint8_t *byte)
{
    if (peek_byte_available)
    {
        peek_byte_available = 0;
        *byte = peek_byte;
        return 1;
    }
    unsigned int timeoutMs = (pollTimeoutUsec + 999) / 1000;
    int r = serialWaitByte(byte, timeoutMs);
    if (r <= 0)
        return r;
    return 1;
}

// Push a byte back so the next readSerialByte() will return it.
void pushByteBack(uint8_t b)
{
    peek_byte_available = 1;
    peek_byte = b;
}

//----------------------------------------
// Send a command to the GNSS device
//----------------------------------------
int writeCommand(const char *command)
{
    // Send the command to the gnssDevice
    if (writeData((const uint8_t *)command, strlen(command)))
        return errno;
    return writeData((const uint8_t *)"\r\n", 2);
}

//----------------------------------------
// Read a response from the gnssDevice
//----------------------------------------
bool getResponse()
{
    bool gotResponse;

    errno = 0;
    gotResponse = false;
    do
    {
        // Read data from the gnssDevice
        ssize_t bytesRead = readSerialByte(&response[responseLength]);

        // Handle the errors
        if (bytesRead <= 0)
            break;

        // Done when CR or LF received
        if ((response[responseLength] == '\r') || (response[responseLength] == '\n'))
        {
            gotResponse = (responseLength != 0);
            break;
        }

        // Buffer the response
        responseLength += 1;
    } while (0);

    // Zero terminate the string
    response[responseLength] = 0;

    // Start a new response if necessary
    if (gotResponse)
    {
        responseLength = 0;
    }

    // Tell the caller of the response
    return gotResponse;
}

//----------------------------------------
// Extract the status from the command response
//----------------------------------------
uint16_t getCommandStatus(const uint8_t *data, const char **message)
{
    uint16_t status;

    // Get the command status value
    status = (((uint16_t)data[0]) << 8) | data[1];

    // Return the status message
    if (message)
    {
        switch (status)
        {
        default:
            *message = "Unknown status value";
            break;

        case 0:
            *message = "Message received and executed successfully.";
            break;

        case 1:
            *message = "Unknow error.";
            break;

        case 2:
            *message = "CRC32 checksum error.";
            break;

        case 3:
            *message = "Timeout.";
            break;

        case 4:
            *message = "Unsupported message.";
            break;

        case 5:
            *message = "Message package error.";
            break;

        case 0x20:
            *message = "Firmware area erase error.";
            break;

        case 0x21:
            *message = "Firmware write to Flash error.";
            break;
        }
    }
    return status;
}

//----------------------------------------
// Read a command response from the gnssDevice
//----------------------------------------
bool getCommandResponse()
{
    bool gotResponse;
    size_t messageBytes;
    uint16_t packetLength;

    errno = 0;
    gotResponse = false;
    do
    {
        // Read data from the gnssDevice
        ssize_t bytesRead = readSerialByte(&response[responseLength]);

        // Handle the errors
        if (bytesRead <= 0)
            break;

        // Wait for 0xAA to start the binary response
        if ((responseLength == 0) && (response[0] != 0xaa))
            break;

        // Buffer the response
        responseLength += 1;

        // Ignore the class and message ID values and upper portion of length
        if (responseLength <= 4)
            break;

        // Get the payload length
        packetLength = (((uint16_t)response[3]) << 8) | response[4];
        messageBytes = 1                   // Head
                       + 1                 // Class ID
                       + 1                 // Message ID
                       + packetLength + 4; // CRC32

        // Skip over the payload and CRC
        if (responseLength <= messageBytes)
            break;

        // Done when 0x55 is in the buffer
        if (response[responseLength - 1] == 0x55)
            gotResponse = true;
    } while (0);

    // Start a new response if necessary
    if (gotResponse)
    {
        commandResponseLength = responseLength;
        responseLength = 0;
    }
    return gotResponse;
}

//----------------------------------------
// Compute the CRC32 for the specified data region
//----------------------------------------

uint32_t computeCrc32(uint32_t initialValue, const uint8_t *data, size_t length)
{
    uint32_t crc;
    const uint8_t *end;

    // Compute the CRC for the data buffer
    crc = initialValue ^ 0xffffffff;
    end = &data[length];
    while (data < end)
        crc = crc32_table[(crc ^ *data++) & 0xFF] ^ (crc >> 8);
    crc ^= 0xffffffff;
    return crc;
}

//----------------------------------------
// Write a 32-bit value in big endian format
//----------------------------------------
void insertBigEndian(uint32_t value, uint8_t *data)
{
    data[0] = (value >> 24) & 0xff;
    data[1] = (value >> 16) & 0xff;
    data[2] = (value >> 8) & 0xff;
    data[3] = value & 0xff;
}

//----------------------------------------
// Write a 32-bit value in little endian format
//----------------------------------------
void insertLittleEndian(uint32_t value, uint8_t *data)
{
    data[0] = value & 0xff;
    data[1] = (value >> 8) & 0xff;
    data[2] = (value >> 16) & 0xff;
    data[3] = (value >> 24) & 0xff;
}

//----------------------------------------
// Display the binary command
//----------------------------------------

//----------------------------------------
// Send the boot firmware version command to the gnssDevice
//----------------------------------------
uint32_t getBootLoaderVersion()
{
    uint8_t command[10];
    uint32_t crc32;

    // Construct the command
    command[0] = 0xaa;
    command[1] = 2;
    command[2] = 0x71;
    command[3] = 0;
    command[4] = 0;
    command[9] = 0x55;

    // Compute the CRC
    crc32 = computeCrc32(0, &command[1], 4);
    insertBigEndian(crc32, &command[5]);

    // Send the command to the gnssDevice
    return writeData(command, sizeof(command));
}

//----------------------------------------
// Send the firmware erase command to the gnssDevice
//----------------------------------------
uint32_t sendFirmwareErase()
{
    uint8_t command[10];
    uint32_t crc32;

    // Construct the command
    command[0] = 0xaa; // Head
    command[1] = 2;    // Class ID
    command[2] = 3;    // Message ID
    command[3] = 0;    // Payload length (big endian)
    command[4] = 0;
    command[9] = 0x55; // Tail

    // Compute the CRC
    crc32 = computeCrc32(0, &command[1], 4);
    insertBigEndian(crc32, &command[5]);

    // Send the command to the gnssDevice
    return writeData(command, sizeof(command));
}

//----------------------------------------
// Send the firmware infomation command to the gnssDevice
//----------------------------------------
uint32_t sendFirmwareInfo()
{
    uint8_t command[26];
    uint32_t crc32;

    // Construct the command
    command[0] = 0xaa; // Head
    command[1] = 2;    // Class ID
    command[2] = 2;    // Message ID
    command[3] = 0;    // Payload length (big endian)
    command[4] = 0x10;
    command[25] = 0x55; // Tail

    // Payload
    insertBigEndian(firmwareLength, &command[5]); // Firmware length in bytes
    insertBigEndian(firmwareCrc32, &command[9]);  // Firmware CRC
    insertBigEndian(0, &command[13]);             // Base address
    insertBigEndian(0, &command[17]);             // Reversed

    // Compute the CRC
    crc32 = computeCrc32(0, &command[1], sizeof(command) - 1 - 4 - 1);
    insertBigEndian(crc32, &command[21]);

    // Send the command to the gnssDevice
    return writeData(command, sizeof(command));
}

//----------------------------------------
// Send a firmware packet to the gnssDevice
//----------------------------------------
uint32_t sendFirmwarePacket()
{
    static uint8_t command[1 + 1 + 1 + 2 + 4 + MAX_PACKET_SIZE + 4 + 1];
    size_t commandLength;
    uint32_t crc32;
    uint8_t *firmwarePayload;
    size_t lengthFirmware;
    size_t lengthPayload;
    size_t offset;

    // Determine the payload length
    offset = packetNumber * MAX_PACKET_SIZE;
    firmwarePayload = &firmwarePackage[offset];
    lengthFirmware = firmwareLength - offset;
    if (lengthFirmware > MAX_PACKET_SIZE)
        lengthFirmware = MAX_PACKET_SIZE;
    lengthPayload = 4 + lengthFirmware;

    // Construct the command
    command[0] = 0xaa;               // Head
    command[1] = 2;                  // Class ID
    command[2] = 4;                  // Message ID
    command[3] = lengthPayload >> 8; // Payload length (big endian)
    command[4] = lengthPayload & 0xff;
    command[5 + lengthPayload + 4] = 0x55; // Tail

    // Add the payload
    insertBigEndian(packetNumber, &command[5]);           // Packet number (big endian)
    memcpy(&command[9], firmwarePayload, lengthFirmware); // Next block of firmware data

    // Compute the CRC
    commandLength = 1 + 1 + 1 + 2 + lengthPayload + 4 + 1;
    crc32 = computeCrc32(0, &command[1], commandLength - 1 - 4 - 1);
    insertBigEndian(crc32, &command[commandLength - 4 - 1]);

    // Send the command to the gnssDevice
    return writeData(command, commandLength);
}

//----------------------------------------
// Send the firmware reset command to the gnssDevice
//----------------------------------------
uint32_t sendFirmwareReset()
{
    uint8_t command[10];
    uint32_t crc32;

    // Construct the command
    command[0] = 0xaa; // Head
    command[1] = 2;    // Class ID
    command[2] = 0x31; // Message ID
    command[3] = 0;    // Payload length (big endian)
    command[4] = 0;
    command[9] = 0x55; // Tail

    // Compute the CRC
    crc32 = computeCrc32(0, &command[1], 4);
    insertBigEndian(crc32, &command[5]);

    // Send the command to the gnssDevice
    return writeData(command, sizeof(command));
}

// Upload the firmware image through a directly connected UART
//----------------------------------------
int directFirmwareUpload(bool timeout)
{
    ssize_t bytesRead;
    uint16_t commandStatus;
    uint32_t crc;
    int exitStatus;
    static bool getResponse;
    bool gotResponse;
    int length;
    const char *message;

    // Handle timeouts
    if (timeout && timeoutMessage && (state < DCFUS_MAX))
    {
        printf("%s\r\n", timeoutMessage);
        timeoutMessage = nullptr;
        return -1;
    }

    // Get the response
    exitStatus = 0;
    gotResponse = false;
    if (getResponse && (timeout == false))
    {
        // Get the binary response
        gotResponse = getCommandResponse();

        // Handle the any response errors
        exitStatus = errno;
        if (exitStatus)
            return exitStatus;
    }

    // Process the state
    switch (state)
    {
    // Default case for development
    default:
        // Display the error
        printf("ERROR: Unknown state %d\r\n", state);
        if (timeout)
            printf("Timeout!\r\n");
        exitStatus = -1;
        timeoutMessage = nullptr;
        break;

    // Wait for response to SYNC WORD 1
    case DCFUS_POWER_ON:
        // Resend the SYNC WORD 1 every 250 mSec
        if (timeout)
        {
            uint8_t data[4];

            // Account for this timeout
            timeoutCount += 1;

            // Exceeded 20 seconds
            if (timeoutCount > (1000 / 250))
            {
                // Display the timeout error
                printf("ERROR: Response to SYNC_WORD1 not received!\r\n");
                printf("Please power cycle or manually reset the LG290P module (by connecting RST to GND)\r\n");
                exitStatus = -1;
                break;
            }

            // Send SYNC_WORD1 0x514C1309 (little endian) every 20 mSec
            data[0] = 0x09;
            data[1] = 0x13;
            data[2] = 0x4c;
            data[3] = 0x51;
            exitStatus = writeData(data, sizeof(data));

            // Start receiving a new response
            responseLength = 0;
        }
        else
        {
            // Read data from the GNSS
            bytesRead = readSerialByte(&response[responseLength]);

            // Handle the errors
            if (bytesRead <= 0)
            {
                exitStatus = errno;
                break;
            }
            if (bytesRead)
            {
                // Account for this byte
                responseLength += 1;
            }

            // Expecting RSP_WORD1 0xAAFC3A4D (little endian)
            if (((responseLength >= 1) && (response[0] != 0x4d)) || ((responseLength >= 2) && (response[1] != 0x3a)) ||
                ((responseLength >= 3) && (response[2] != 0xfc)) || ((responseLength == 4) && (response[3] != 0xaa)))
            {
                // Error, discard any received data and try again
                responseLength = 0;
            }

            // Successfully received RSP_WORD1 0xAAFC3A4D (little endian)
            else if (responseLength == 4)
            {
                uint8_t data[4];

                // Send SYNC_WORD2 0x1203A504 (little endian)
                data[0] = 0x04;
                data[1] = 0xa5;
                data[2] = 0x03;
                data[3] = 0x12;
                exitStatus = writeData(data, sizeof(data));
                responseLength = 0;
                timeoutMessage = "ERROR: Failed to receive RSP_WORD2";
                state = DCFUS_SYNC;
            }
        }
        break;

    // Waiting for RSP_WORD2 0x55FD5BA0 (little endian)
    case DCFUS_SYNC:
        // Read data from the GNSS
        bytesRead = readSerialByte(&response[responseLength]);
        // Handle the errors
        if (bytesRead <= 0)
        {
            exitStatus = errno;
            break;
        }
        if (bytesRead)
        {
            // Account for this byte
            responseLength += 1;
        }

        // Expecting RSP_WORD2 0x55FD5BA0 (little endian)
        if (((responseLength >= 1) && (response[0] != 0xa0)) || ((responseLength >= 2) && (response[1] != 0x5b)) ||
            ((responseLength >= 3) && (response[2] != 0xfd)) || ((responseLength == 4) && (response[3] != 0x55)))
        {
            // Error, discard any received data and try again
            responseLength = 0;
        }

        // Successfully received RSP_WORD2 0x55FD5BA0 (little endian)
        else if (responseLength == 4)
        {
            // Send the boot version command
            exitStatus = getBootLoaderVersion();
            getResponse = true;
            responseLength = 0;
            pollTimeoutUsec = 500 * 1000;
            timeoutMessage = "ERROR: Timeout getting bootloader version command!\r\n";
            state = DCFUS_BOOT_VERSION;
        }
        break;

    case DCFUS_BOOT_VERSION:
        if (gotResponse)
        {
            // Process the response
            crc = computeCrc32(0, &response[1], commandResponseLength - 1 - 4 - 1);

            // Check for an error
            if ((response[0] != 0xaa)                       // Head
                || (response[1] != 2)                       // Class ID
                || (response[2] != 0)                       // Message ID
                || (response[3] != 0)                       // Payload length (big endian)
                || (response[4] != 7) || (response[5] != 2) // Class ID
                || (response[6] != 0x71)                    // Message ID
                || (response[12] != ((crc >> 24) & 0xff))   // CRC (big endian)
                || (response[13] != ((crc >> 16) & 0xff)) || (response[14] != ((crc >> 8) & 0xff)) ||
                (response[15] != (crc & 0xff)) || (response[16] != 0x55)) // Tail
            {
                printf("ERROR: Boot loader version error\r\n");
                exitStatus = -1;
                break;
            }

            // Verify the command status
            commandStatus = getCommandStatus(&response[7], &message);
            if (commandStatus)
            {
                printf("ERROR: Failed to get boot loader version!\r\n");
                exitStatus = -1;
                break;
            }

            // Display the boot loader version
            printf("Bootloader version: %d.%d.%d\r\n", response[9], response[10], response[11]);

            // Send the firmware information
            exitStatus = sendFirmwareInfo();
            getResponse = true;
            responseLength = 0;
            pollTimeoutUsec = 500 * 1000;
            timeoutMessage = "ERROR: Timeout waiting for firmware info command response!\r\n";
            state = DCFUS_FIRMWARE_INFO;
        }
        break;

    // Wait for the firmware information response
    case DCFUS_FIRMWARE_INFO:
        if (gotResponse)
        {
            // Process the response
            crc = computeCrc32(0, &response[1], commandResponseLength - 1 - 4 - 1);

            // Check for an error
            if ((response[0] != 0xaa)                         // Head
                || (response[1] != 2)                         // Class ID
                || (response[2] != 0)                         // Message ID
                || (response[3] != 0)                         // Payload length (big endian)
                || (response[4] != 4) || (response[5] != 2)   // Class ID
                || ((response[6] != 2) && (response[6] != 0)) // Message ID
                || (response[9] != ((crc >> 24) & 0xff))      // CRC (big endian)
                || (response[10] != ((crc >> 16) & 0xff)) || (response[11] != ((crc >> 8) & 0xff)) ||
                (response[12] != (crc & 0xff)) || (response[13] != 0x55)) // Tail
            {
                printf("ERROR: Firmware information error\r\n");
                exitStatus = -1;
                break;
            }

            // Verify the command status
            commandStatus = getCommandStatus(&response[7], &message);
            if (commandStatus)
            {
                printf("ERROR: Failed to set the firmware information!\r\n");
                exitStatus = -1;
                break;
            }

            // Determine if firmware updates are enabled
            if (firmwareUpdateEnabled == false)
            {
                exitStatus = BAIL_WITH_SUCCESS;
                break;
            }

            // Start the firmware erase
            exitStatus = sendFirmwareErase();
            pollTimeoutUsec = 30 * 1000 * 1000;
            timeoutMessage = "ERROR: Timeout waiting for erase command response!\r\n";
            state = DCFUS_FIRMWARE_ERASE;
        }
        break;

    // Wait for the firmware erase response
    case DCFUS_FIRMWARE_ERASE:
        if (gotResponse)
        {
            // Process the response
            crc = computeCrc32(0, &response[1], commandResponseLength - 1 - 4 - 1);

            // Check for an error
            if ((response[0] != 0xaa)                         // Head
                || (response[1] != 2)                         // Class ID
                || (response[2] != 0)                         // Message ID
                || (response[3] != 0)                         // Payload length (big endian)
                || (response[4] != 4) || (response[5] != 2)   // Class ID
                || ((response[6] != 3) && (response[6] != 0)) // Message ID
                || (response[9] != ((crc >> 24) & 0xff))      // CRC (big endian)
                || (response[10] != ((crc >> 16) & 0xff)) || (response[11] != ((crc >> 8) & 0xff)) ||
                (response[12] != (crc & 0xff)) || (response[13] != 0x55)) // Tail
            {
                printf("ERROR: Firmware erase error\r\n");
                exitStatus = -1;
                break;
            }

            // Verify the command status
            commandStatus = getCommandStatus(&response[7], &message);
            if (commandStatus)
            {
                printf("ERROR: Failed to erase the GNSS firmware!\r\n");
                exitStatus = -1;
                break;
            }

            // Determine if just erasing the flash
            if (eraseOnly)
            {
                exitStatus = BAIL_WITH_SUCCESS;
                break;
            }

            // Start the firmware upload
            packetNumber = 0;
            timeoutMessage = "ERROR: Timeout waiting for firmware packet response!\r\n";
            exitStatus = writeCommand("Command Packet");
            if (!exitStatus)
                exitStatus = sendFirmwarePacket();
            state = DCFUS_FIRMWARE_UPLOAD;
        }
        break;

    // Wait for the firmware upload response
    case DCFUS_FIRMWARE_UPLOAD:
        if (gotResponse)
        {
            // Process the response
            crc = computeCrc32(0, &response[1], commandResponseLength - 1 - 4 - 1);

            // Check for an error
            if ((response[0] != 0xaa)                         // Head
                || (response[1] != 2)                         // Class ID
                || (response[2] != 0)                         // Message ID
                || (response[3] != 0)                         // Payload length (big endian)
                || (response[4] != 4) || (response[5] != 2)   // Class ID
                || ((response[6] != 4) && (response[6] != 0)) // Message ID
                || (response[9] != ((crc >> 24) & 0xff))      // CRC (big endian)
                || (response[10] != ((crc >> 16) & 0xff)) || (response[11] != ((crc >> 8) & 0xff)) ||
                (response[12] != (crc & 0xff)) || (response[13] != 0x55)) // Tail
            {
                // Show a progress bar so user knows how far we got
                printf("ERROR: Firmware upload error at packet %d\r\n", packetNumber);
                printProgress(packetNumber, packetCount);
                exitStatus = -1;
                break;
            }

            // Verify the command status
            commandStatus = getCommandStatus(&response[7], &message);
            if (commandStatus)
            {
                printf("ERROR: Failed firmware upload at packet %d!\r\n", packetNumber);
                exitStatus = -1;
                break;
            }

            // Display progress bar only when progress increases by 1%
            static int lastPercent = -1;
            int currentPercent = ((packetNumber + 1) * 100) / packetCount;
            if (currentPercent != lastPercent && currentPercent < 100)
            {
                printProgress(packetNumber + 1, packetCount);
                lastPercent = currentPercent;
            }

            // Account for this packet
            packetNumber += 1;

            // Print a message before the module starts processing the firmware (which can take about 20 seconds)
            if (packetNumber == packetCount - 1)
            {
                printf("[========================================] 100%%\r\n");
                printf("Module processing and verifying firmware...\r\n");
            }

            if (packetNumber < packetCount)
            {
                exitStatus = sendFirmwarePacket();
                state = DCFUS_FIRMWARE_UPLOAD;
                break;
            }

            // Send the GNSS reset command
            exitStatus = sendFirmwareReset();
            printf("Firmware update successful\r\n");
            exitStatus = BAIL_WITH_SUCCESS;
            return exitStatus;
        }
        break;

    // Wait for the GNSS reset response
    case DCFUS_FIRMWARE_RESET:
        if (gotResponse)
        {
            // Process the response
            crc = computeCrc32(0, &response[1], commandResponseLength - 1 - 4 - 1);

            // Check for an error
            if ((response[0] != 0xaa)                       // Head
                || (response[1] != 2)                       // Class ID
                || (response[2] != 0)                       // Message ID
                || (response[3] != 0)                       // Payload length (big endian)
                || (response[4] != 4) || (response[5] != 2) // Class ID
                || (response[6] != 0x31)                    // Message ID
                || (response[9] != ((crc >> 24) & 0xff))    // CRC (big endian)
                || (response[10] != ((crc >> 16) & 0xff)) || (response[11] != ((crc >> 8) & 0xff)) ||
                (response[12] != (crc & 0xff)) || (response[13] != 0x55)) // Tail
            {
                printf("ERROR: Firmware information error\r\n");
                exitStatus = -1;
                break;
            }

            // Verify the command status
            commandStatus = getCommandStatus(&response[7], &message);
            if (commandStatus)
            {
                printf("ERROR: Failed GNSS reset\r\n");
                exitStatus = -1;
                break;
            }

            // All done
            exitStatus = BAIL_WITH_SUCCESS;
        }
        break;
    }

    return exitStatus;
}

//----------------------------------------
// Handle data flow with the COM Port
//----------------------------------------
int handleComPort()
{
    int exitStatus = 0;
    unsigned int timeoutMs;
    uint8_t b;
    int r;

    timeoutMessage = nullptr;

    while (exitStatus == 0)
    {
        timeoutMs = (pollTimeoutUsec + 999) / 1000; // convert usec -> ms

        r = serialWaitByte(&b, timeoutMs);
        if (r < 0)
        {
            exitStatus = errno;
            fprintf(stderr, "ERROR: Target disconnected (serial port communication failed)\r\n");
            break;
        }

        if (r == 1)
        {
            // A byte is waiting, push it back and run state handler
            pushByteBack(b);
            exitStatus = directFirmwareUpload(false);
        }
        else
        {
            // Timeout
            exitStatus = directFirmwareUpload(true);
        }
    }

    return exitStatus;
}

//----------------------------------------
// Use a given port to do the GNSS firmware upgrade
//----------------------------------------
int gnssUartFirmwareUpgrade(const char *portName)
{
    int exitStatus;
    int comStatus = 0;

    exitStatus = 0;
    do
    {
        // Wait for the COM Port to become available
        printf("Waiting for Port %s to become available\r\n", portName);
        do
        {
            // Attempt to open the COM Port
            // Keep looping until we can open the port successfully
            if (comPort)
            {
                sp_close(comPort);
                sp_free_port(comPort);
                comPort = NULL;
            }
            if (sp_get_port_by_name(portName, &comPort) != SP_OK)
            {
                comPort = NULL;
            }
            else
            {
                if (sp_open(comPort, SP_MODE_READ_WRITE) != SP_OK)
                {
                    sp_free_port(comPort);
                    comPort = NULL;
                }
            }
        } while (comPort == NULL);

        // The LG290P bootloader is hard wired at 460800 baud
        exitStatus = configureComPort(460800);
        if (exitStatus)
            break;

        // Send reboot command to the GNSS module
        printf("Sending reboot command to GNSS module...\r\n");
        exitStatus = writeData((const uint8_t *)"$PQTMSRR*4B\r\n", 13);
        if (exitStatus)
        {
            printf("ERROR: Failed to send reboot command\r\n");
            break;
        }

        // Wait for reboot command response
        printf("Waiting for reboot command acknowledgment...\r\n");
        responseLength = 0;
        pollTimeoutUsec = 100 * 1000; // 100ms timeout per byte
        int tries = 0;
        bool gotRebootResponse = false;

        while (tries < 5) // Wait up to 500ms total
        {
            // Handle incoming serial data during reboot wait
            comStatus = handleComPort();
            if (comStatus <= 0)
            {
                // Exit loop immediately on fatal error
                exitStatus = comStatus;
                return (exitStatus);
            }

            if (comStatus == BAIL_WITH_SUCCESS)
            {
                // Exit loop immediately on firmware update complete
                exitStatus = comStatus;
                return (exitStatus);
            }

            unsigned int timeoutMs = 100;
            uint8_t b;
            int r = serialWaitByte(&b, timeoutMs);

            if (r == 1)
            {
                response[responseLength++] = b;

                // Check if we got a complete line (CR or LF)
                if ((b == '\r') || (b == '\n'))
                {
                    response[responseLength] = 0;
                    printf("Reboot response received: %s\r\n", (char *)response);
                    gotRebootResponse = true;
                    break;
                }

                // Safety check - don't overflow buffer
                if (responseLength >= sizeof(response) - 1)
                {
                    response[responseLength] = 0;
                    printf("Reboot response: %s\r\n", (char *)response);
                    gotRebootResponse = true;
                    break;
                }
                tries = 0; // Reset tries on data received
            }
            else
            {
                tries++;
            }
        }

        // Upload the firmware image
        pollTimeoutUsec = 250 * 1000;
        timeoutCount = 0;
        exitStatus = handleComPort();
    } while (0);

    return exitStatus;
}

//----------------------------------------
// Application to update the firmware on a Quectel GNSS device
//----------------------------------------
int main(int argc, char **argv)
{
    int argCount;
    int argOffset;
    int exitStatus;
    const char *fileName;
    int index;
    bool showVersion = false;
    const COMMAND_OPTION options[] = {
        {&firmwareUpdateEnabled, "--firmware-update-enabled", "Enable firmware updates (default)"},
        {&eraseOnly, "--erase-only", "Perform the flash erase and then exit"},
        {&skipVersionCheck, "--skip-version-check", "Don't display current firmware version"},
        {&showVersion, "--version", "Print the version of this tool and exit"},
    };
    const int optionCount = sizeof(options) / sizeof(options[0]);
    const char *portName;

    exitStatus = -1;
    do
    {
        // Get the options
        argCount = 0;
        argOffset = 1;
        portName = "";
        fileName = "";
        while (argc - argOffset)
        {
            bool match;

            // Check for an option
            match = false;
            if (strncmp(argv[argOffset], "--", 2) == 0)
            {
                // Walk the list of options
                for (index = 0; index < optionCount; index++)
                {
                    match = (strcmp(argv[argOffset], options[index]._optionString) == 0);
                    if (match)
                    {
                        *options[index]._optionBoolean = true;
                        if (options[index]._optionBoolean == &showVersion)
                        {
                            printf("%s\r\n", TOOL_VERSION);
                            return 0;
                        }
                        argOffset += 1;
                        break;
                    }
                }
            }

            // Check for a valid argument
            else if (argCount < 2)
            {
                if (argCount == 0)
                    // Save the terminal port name
                    portName = argv[argOffset++];
                else
                    // Save the file name
                    fileName = argv[argOffset++];
                match = true;
                argCount += 1;
            }

            // Handle the error
            if (match == false)
            {
                // Display the help message
                printf("Invalid argument or option: %s\r\n", argv[argOffset]);
                break;
            }
        }

        // Display the help text
        if ((argCount < 2) || (argOffset != argc))
        {
            printf("%s   [options]   <COM_Port%s%s>   <Firmware_File>\r\n", argv[0], (argc == 2) ? ": " : "",
                   (argc == 2) ? argv[1] : "");
            printf("    COM_PORT: Example COM3 or /dev/ttyACM0\r\n");
            printf("    Firmware_File: Example LG290P03AANR02A01S.pkg\r\n");
            printf("Options:\r\n");
            for (index = 0; index < optionCount; index++)
                printf("    %s: %s\r\n", options[index]._optionString, options[index]._helpText);
            printf("\r\n");
            printf("A program to update the firmware on the Quectel LG290P GNSS receiver.\r\n");
            break;
        }

        // Attempt to open the firmware file
        // Open firmware in binary mode when available to avoid CRLF translation
#ifdef _WIN32
        firmware = open(fileName, O_RDONLY | O_BINARY, 0);
#else
        firmware = open(fileName, O_RDONLY, 0);
#endif
        if (firmware < 0)
        {
            exitStatus = errno;
            printf("ERROR: Failed to open file %s\r\n", fileName);
            perror("");
            break;
        }

        // Determine the firmware length
        firmwareLength = lseek(firmware, 0, SEEK_END);
        if (firmwareLength == (off_t)-1)
        {
            exitStatus = errno;
            perror("ERROR: Failed to get the file length");
            break;
        }
        printf("Firmware length: %ld bytes\r\n", firmwareLength);

        // Start at the beginning of the file
        if (lseek(firmware, 0, SEEK_SET) == (off_t)-1)
        {
            exitStatus = errno;
            perror("ERROR: Failed to set the start of the file");
            break;
        }

        // Allocate a buffer and read the firmware image into memory
        firmwarePackage = malloc((size_t)firmwareLength);
        if (firmwarePackage == NULL)
        {
            exitStatus = errno;
            perror("ERROR: Failed to allocate memory for firmware");
            break;
        }
        // rewind file just in case
        if (lseek(firmware, 0, SEEK_SET) == (off_t)-1)
        {
            exitStatus = errno;
            perror("ERROR: Failed to rewind firmware file");
            break;
        }
        {
            /* read() may return fewer bytes than requested, so loop until we've
               consumed the entire file or hit an error. */
            ssize_t totalRead = 0;
            while ((off_t)totalRead < firmwareLength)
            {
                ssize_t r = read(firmware, firmwarePackage + totalRead, firmwareLength - totalRead);
                if (r < 0)
                {
                    exitStatus = errno;
                    perror("ERROR: Failed to read firmware file");
                    break;
                }
                if (r == 0)
                {
                    // unexpected EOF
                    exitStatus = EIO;
                    fprintf(stderr, "ERROR: Unexpected end of firmware file (read %zd of %ld bytes)\n", totalRead,
                            (long)firmwareLength);
                    break;
                }
                totalRead += r;
            }
            if ((off_t)totalRead != firmwareLength)
                break;
        }

        // Determine the packet count
        packetCount = (firmwareLength + MAX_PACKET_SIZE - 1) / MAX_PACKET_SIZE;

        // Compute the firmware CRC
        uint8_t bytes[4];
        insertLittleEndian(firmwareLength, bytes);
        firmwareCrc32 = computeCrc32(0, bytes, sizeof(bytes));
        firmwareCrc32 = computeCrc32(firmwareCrc32, firmwarePackage, firmwareLength);
        printf("Firmware CRC32: 0x%08x\r\n", firmwareCrc32);

        // Attempt to upgrade the GNSS firmware
        packetNumber = -1;
        exitStatus = gnssUartFirmwareUpgrade(portName);
    } while (0);

    // Done with the COM port
    if (comPort)
    {
        sp_close(comPort);
        sp_free_port(comPort);
        comPort = NULL;
    }
    // Release the firmware buffer
    if (firmwarePackage)
        free(firmwarePackage);

    // Done with the firmware file
    if (firmware)
        close(firmware);

    // Convert the exitStatus value if necessary
    if (exitStatus == BAIL_WITH_SUCCESS)
        exitStatus = 0;

    return exitStatus;
}
