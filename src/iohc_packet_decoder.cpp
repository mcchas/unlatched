#include <cstdio>
#include <cstring>
#include <esp_attr.h>
#include <iohc_device.h>
#include <iohc_last_command.h>
#include <iohc_packet.h>
#include <utils.h>
#include <rom/ets_sys.h>

#include "interact.h"

namespace IOHC {

static bool sameDiscussion;  // = false;
static uint8_t countDiscussions;
static Memorize discussions;
static std::array<uint8_t, 6> challengeAsked = {};
static std::array<uint8_t, 6> challengeAnswer = {};

void IRAM_ATTR iohcPacket::decode(bool verbosity) {
    sameDiscussion = true;
    countDiscussions++;
    if (countDiscussions > 4) {
        sameDiscussion = false;
        countDiscussions = 1;
    }

    if (packetStamp - relStamp > 500000L) {
        ets_printf("\n");
        relStamp = packetStamp;  // - this->relStamp;
        sameDiscussion = false;
        countDiscussions = 1;
    }
    char _dir[3] = {};

    if (this->is1W()) {
        _dir[0] = '>';
    }

    else if (this->payload.packet.header.CtrlByte1.asStruct.StartFrame &&
             !this->payload.packet.header.CtrlByte1.asStruct.EndFrame)
        _dir[0] = '>';
    else if (!this->payload.packet.header.CtrlByte1.asStruct.StartFrame &&
             this->payload.packet.header.CtrlByte1.asStruct.EndFrame)
        _dir[0] = '<';
    else
        _dir[0] = ' ';

    ets_printf("(%2.2u) %1xW S %s E %s ", this->payload.packet.header.CtrlByte1.asStruct.MsgLen,
               this->is1W() ? 1 : 2,
               this->payload.packet.header.CtrlByte1.asStruct.StartFrame ? "1" : "0",
               this->payload.packet.header.CtrlByte1.asStruct.EndFrame ? "1" : "0");

    if (this->payload.packet.header.CtrlByte2.asStruct.LPM) ets_printf("[LPM]");
    if (this->payload.packet.header.CtrlByte2.asStruct.Beacon) ets_printf("[B]");
    if (this->payload.packet.header.CtrlByte2.asStruct.Routed) ets_printf("[R]");
    if (this->payload.packet.header.CtrlByte2.asStruct.Prio) ets_printf("[PRIO]");
    if (this->payload.packet.header.CtrlByte2.asStruct.Unk2) ets_printf("[U2]");
    if (this->payload.packet.header.CtrlByte2.asStruct.Unk3) ets_printf("[U3]");
    if (this->payload.packet.header.CtrlByte2.asStruct.Version)
        ets_printf("[V]%u", this->payload.packet.header.CtrlByte2.asStruct.Version);

    ets_printf("\tFROM %2.2X%2.2X%2.2X TO %2.2X%2.2X%2.2X CMD %2.2X",
               this->payload.packet.header.source[0], this->payload.packet.header.source[1],
               this->payload.packet.header.source[2], this->payload.packet.header.target[0],
               this->payload.packet.header.target[1], this->payload.packet.header.target[2],
               this->payload.packet.header.cmd);

    if (verbosity) ets_printf(" +%03d\t", static_cast<int>((packetStamp - relStamp) / 1000.0));
    ets_printf(" %s ", _dir);

    uint8_t dataLen = this->buffer_length - 9;
    ets_printf(" DATA(%2.2u) ", dataLen);

    /* A 1W decode branch lived here. Every frame this rig sends or reads is 2W --
     * 1W is the one-way 868 MHz protocol whose remote was deleted -- and the raw
     * `FRAME t=... <hex>` trace line still shows a 1W frame if a neighbour's
     * device ever sends one. Only the prettier decode of someone else's traffic
     * is lost. is1W() stays: it reads CtrlByte1.Protocol, which is in every
     * header and is how a frame is known to be 2W at all. */
    {
        // 2W fields
        if (dataLen != 0) {
            std::string msg_data = bitrow_to_hex_string(this->payload.buffer + 9, dataLen);
            ets_printf(" %s ", msg_data.c_str());
            /*Private Atlantic/Sauter/Thermor*/
            if (this->cmd() == 0x20) {
            }
            if (this->cmd() == iohcDevice::DISCOVER_ANSWER_0x29) {
                ets_printf("2W Device want to be paired Waiting for 0x2C (or 0x38) ");
                // Node type and subtype (2 bytes): type on 10 bits and subtype on the remainer
                // Node type = (field >> 6) & 1023
                // Node subtype = field & 63
                // Node address (3 bytes)
                // Manufacturer ID (1 byte)
                // Multiinfo (1 byte)
                // Timestamp (2 bytes)

                std::vector<uint8_t> deviceAsked;
                deviceAsked.assign(this->payload.buffer + 9, this->payload.buffer + 18);
                int type = ((deviceAsked[0] << 8) | deviceAsked[1]);
                type = (type >> 6) & 0x3FF;
                int subtype = type & 0x3F;
                ets_printf("Type %03X Subtype %d Manu %02X Addr ", type, subtype, deviceAsked[6]);
                for (uint8_t i = 2; i < 5; i++) {
                    ets_printf("%02X", deviceAsked[i]);
                }
            }
            if (this->cmd() == iohcDevice::DISCOVER_REMOTE_ANSWER_0x2B) {
                ets_printf("2W Remote want to be paired ");
                std::vector<uint8_t> deviceAsked;
                deviceAsked.assign(this->payload.buffer + 9, this->payload.buffer + 18);
                int type = ((deviceAsked[0] << 8) | deviceAsked[1]);
                type = (type >> 6) & 0x3FF;
                int subtype = type & 0x3F;
                ets_printf("Type %03X Subtype %d Manu %02X Addr ", type, subtype, deviceAsked[6]);
                for (uint8_t i = 2; i < 5; i++) {
                    ets_printf("%02X", deviceAsked[i]);
                }
            }
            // get set name
            if (this->cmd() == 0x04) {  // answer of 0x00 or 0x03 maybe 0x01
                /* Split msg_data by 4, 4, 4, 4, 6, 6 array parts -- but only as
                 * far as the payload actually goes.
                 *
                 * These offsets describe the LONG form, 14 data bytes, which is
                 * what a position read (0x03) draws. A movement command draws a
                 * short one: six data bytes, twelve characters here. The last
                 * two fields then start beyond the end of the string, and
                 * std::string::substr throws std::out_of_range for a position
                 * past the end -- which, with exceptions disabled as they are in
                 * this build, is an abort() and a panic.
                 *
                 * That made every `open`, `close` and `stop` a coin toss: the
                 * actuator acks, the decoder reads past the ack, the rig
                 * reboots. It went unnoticed for as long as commands were typed
                 * one at a time into a console and mostly followed by a status
                 * read; a front end that sends a movement command per tap hits
                 * it immediately.
                 */
                const auto field = [&msg_data](const size_t pos, const size_t len) -> std::string {
                    if (pos >= msg_data.size()) return std::string();
                    return msg_data.substr(pos, len);
                };
                ets_printf("%s %s %s %s %s %s %s ", field(0, 2).c_str(),
                           field(2, 2).c_str(), field(4, 4).c_str(),    // Asked state
                           field(8, 4).c_str(), field(12, 4).c_str(),   // Actual states
                           field(16, 6).c_str(), field(22, 2).c_str()  // From
                );
                // At least 2 types: simple state or extended state
                // simple state OPEN CLOSE etc... c800 c800 0000 - d100 0000 0000 - d200 3796
                // actualized states ... 0c00 0a3f 0000 - 6000 6011 0000 - 7000 7011 0000
            }
            if (this->cmd() == 0x37) {
                ets_printf("This is my address ");
            }
            if (this->cmd() == 0x0D) {
                ets_printf("This is my data... ");
            }
            // 0x59 (general info 3) used to be rendered through
            // iohcOther2W::extractAndNormalizeName. Nothing here has ever
            // drawn a 0x59 -- home_io_control records only a Somfy dimmer
            // answering one -- so it is printed raw rather than reviving the
            // module for a reply we have never seen.
            if (this->cmd() == 0x59) {
                ets_printf("general info 3 ");
            }
            if (this->cmd() == 0x3D) {
            }
            if (this->cmd() == 0x3C) {
                ets_printf("Challenge asked after Last Command %2.2X (%d)",
                           iohcLastCommandGet(nullptr),
                           static_cast<int>(iohcLastCommandLen()));
            }

        } else {
            if (this->cmd() == iohcDevice::DISCOVER_0x28) {
                ets_printf("2W Pairing Asked Waiting for 0x29/0x2B");
            }
            if (this->cmd() == iohcDevice::DISCOVER_ACTUATOR_0x2C) {
                ets_printf("2W Actuator Ack Asked Waiting for 0x2D");
            }
            if (this->cmd() == iohcDevice::LAUNCH_KEY_TRANSFERT_0x38) {
                ets_printf("2W Key Transfert Asked after Command %2.2X Waiting for 0x32",
                           this->cmd());
            }  //payload.packet.header.cmd);}
        }
    }
    ets_printf("\n");
    relStamp = packetStamp;
}

}  // namespace IOHC
