#ifndef IOHC_PACKET_H
#define IOHC_PACKET_H

#include <array>
#include <vector>
#include <string>

#include <board_config.h>
#include <cstring>
#include <esp_timer.h>
#include <unordered_map>

#include "esp32-hal-gpio.h"

// Only one radio now: the SX1276 branch pointed at a header deleted with the
// 868 MHz support.
#include <rf231_helpers.h>

#define RESET_AFTER_LAST_MSG_US 15000
#define MAX_FRAME_LEN 32
#define IOHC_INBOUND_MAX_PACKETS 255  // Maximum Inbound packets buffer
#define IOHC_OUTBOUND_MAX_PACKETS 20  // Maximum Outbound packets

namespace IOHC {
class iohcDevice;
// Simplify this
using address = uint8_t[3];
typedef std::array<uint8_t, 3> Address;

struct CB1 {
    uint8_t MsgLen : 5;      //1
    uint8_t Protocol : 1;    //2
    uint8_t StartFrame : 1;  //3
    uint8_t EndFrame : 1;    //4
};

struct CB2 {
    uint8_t Version : 2;
    uint8_t Prio : 1;
    uint8_t Unk2 : 1;
    uint8_t Unk3 : 1;
    uint8_t LPM : 1;
    uint8_t Routed : 1;
    uint8_t Beacon : 1;
};

union CtrlByte1Union {
    uint8_t asByte;
    CB1 asStruct;
};

union CtrlByte2Union {
    uint8_t asByte;
    CB2 asStruct;
};

/// Common frame header
struct _header {
    CtrlByte1Union CtrlByte1;  //1
    CtrlByte2Union CtrlByte2;  //1
    address target;            //3
    address source;            //3
    uint8_t cmd;               //1
};

struct Acei {
    uint8_t isvalid : 1;
    uint8_t extended : 2;
    uint8_t service : 2;
    uint8_t level : 3;
};

union AceiUnion {
    uint8_t asByte;
    Acei asStruct;
};

//61 0110 0001
//83 1000 0011
inline void setAcei(AceiUnion &acei, uint8_t value) {
    acei.asByte = value;
}

struct _p0x00_16 {
    uint8_t origin;
    AceiUnion acei;
    uint8_t main[2];
    uint8_t fp1;
    uint8_t fp2;
    uint8_t data[2];
    uint8_t sequence[2];
    uint8_t hmac[6];
    void setAcei(uint8_t as_byte) { acei.asByte = as_byte; }
};

struct _p0x00_14 {
    uint8_t origin;
    AceiUnion acei;
    uint8_t main[2];
    uint8_t fp1;
    uint8_t fp2;
    uint8_t sequence[2];
    uint8_t hmac[6];
    void setAcei(uint8_t as_byte) { acei.asByte = as_byte; }
};

struct _p0x01_13 {
    uint8_t origin;
    AceiUnion acei;
    uint8_t main;  //[2];
    uint8_t fp1;
    uint8_t fp2;
    uint8_t sequence[2];
    uint8_t hmac[6];
    void setAcei(uint8_t as_byte) { acei.asByte = as_byte; }
};

struct _p0x20_15 {
    uint8_t origin;
    AceiUnion acei;
    uint8_t main[2];
    uint8_t fp1;
    uint8_t fp2;
    uint8_t fp3;
    uint8_t sequence[2];
    uint8_t hmac[6];
    void setAcei(uint8_t as_byte) { acei.asByte = as_byte; }
};

struct _p0x20_13 {
    uint8_t origin;
    AceiUnion acei;
    uint8_t main[2];
    uint8_t fp1;
    uint8_t sequence[2];
    uint8_t hmac[6];
    void setAcei(uint8_t as_byte) { acei.asByte = as_byte; }
};

struct _p0x20_16 {
    uint8_t origin;
    AceiUnion acei;
    uint8_t main[2];
    uint8_t fp1;
    uint8_t fp2;
    uint8_t data[2];
    uint8_t sequence[2];
    uint8_t hmac[6];
    void setAcei(uint8_t as_byte) { acei.asByte = as_byte; }
};

struct _p0x2b {
    uint8_t actuator[2];
    address backbone;
    uint8_t manufacturer;
    uint8_t info;
    uint8_t tstamp[2];
};
/// 1) 1W PAIR
struct _p0x2e {
    uint8_t data;
    uint8_t sequence[2];
    uint8_t hmac[6];
};
/// 3) 1W ADD**
struct _p0x30 {
    uint8_t enc_key[16];
    uint8_t man_id;
    uint8_t data;
    uint8_t sequence[2];
};

union _msg {
    _p0x01_13 p0x01_13;
    _p0x00_14 p0x00_14;
    _p0x20_15 p0x20_15;
    _p0x20_13 p0x20_13;
    _p0x00_16 p0x00_16;
    _p0x20_16 p0x20_16;
    _p0x2b p0x29;
    _p0x2b p0x2b;
    _p0x2e p0x2e;
    _p0x30 p0x30;
    _p0x2e p0x39;  // same format of 2e
};

struct _packet {
    _header header;
    _msg msg;
};

typedef union payload {
    uint8_t buffer[MAX_FRAME_LEN];
    _packet packet;
} Payload;

/* keep the size of variable lenght of data */
typedef struct {
    uint8_t memorizedCmd;
    std::vector<uint8_t> memorizedData;
} Memorize;

inline unsigned long packetStamp = 0L;
inline unsigned long relStamp = 0L;
/* lastCmd and lastData used to live here as bare globals. They were written
 * by three tasks and read by two, with no synchronisation, and assigning the
 * vector freed its buffer under whoever was copying it. They are now behind
 * iohc_last_command.h. */
/**
    Class implementing the IOHC packet received/sent, including some additional members useful to track/control Radio parameters and timings
    */
class iohcPacket {
  public:
    iohcPacket() {
        // Common Flags
        // size is 8 if protocol version is 0 else 10
        this->payload.packet.header.CtrlByte1.asStruct.MsgLen = sizeof(_header) - 1;
        this->payload.packet.header.CtrlByte2.asByte = 0;

        this->repeatTime = 25;
        this->repeat = 0;
        this->lock = false;
    };

    ~iohcPacket() = default;

    void reset() {
        payload = {};
        buffer_length = 0;
        frequency = 0;  // = CHANNEL2; // Both 1W & 2W
        repeatTime = 25;
        repeat = 0;
        lock = false;

        afc = {};   // AFC freq correction applied
        snr = {};   // in dB
        rssi = {};  // -RSSI*2 of last packet received
        lna = {};   // LNA attenuation in dB
    };

    Payload payload{};
    uint8_t buffer_length = 0;
    uint32_t frequency = 0;  // = CHANNEL2; // Both 1W & 2W
    unsigned long repeatTime = 13L;
    uint8_t repeat = 0;
    bool lock = false;

    double afc{};   // AFC freq correction applied
    uint8_t snr{};  // in dB
    float rssi{};   // -RSSI*2 of last packet received
    uint8_t lna{};  // LNA attenuation in dB

    void decode(bool verbosity = false);

    bool operator==(const iohcPacket &other) const {
        return (memcmp(this->payload.buffer + 2, other.payload.buffer + 2, 16) == 0);
    }

    bool is1W() const { return this->payload.packet.header.CtrlByte1.asStruct.Protocol; }
    uint8_t cmd() const { return this->payload.packet.header.cmd; }
    std::vector<uint8_t> data() const {
        return std::vector<uint8_t>(this->payload.buffer + 9,
                                    this->payload.buffer + this->buffer_length);
    }
};
}  // namespace IOHC
#endif
