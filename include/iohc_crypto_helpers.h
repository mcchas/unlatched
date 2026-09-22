#ifndef IOHCCRYPTOHELPERS_H
#define IOHCCRYPTOHELPERS_H

#include <string>
#include <vector>
#include <tuple>
#include "mbedtls/aes.h"

#define CRC_POLYNOMIAL_CCITT 0x8408

uint8_t hexStringToBytes(const std::string hexString, uint8_t *byteString);
std::string bytesToHexString(const uint8_t *byteString, uint8_t len);

namespace iohcCrypto {
extern uint8_t system_key[16];
extern uint8_t transfert_key[16];

uint16_t computeCrc(uint8_t data, uint16_t crc);
uint16_t radioPacketComputeCrc(uint8_t *buffer, uint8_t bufferLength);
uint16_t radioPacketComputeCrc(std::vector<uint8_t> &buffer);
std::tuple<uint8_t, uint8_t> computeChecksum(uint8_t frame_byte, uint8_t chksum1, uint8_t chksum2);
std::vector<uint8_t> constructInitialValue(const std::vector<uint8_t> &frame_data,
                                           const uint8_t *challenge,
                                           const uint8_t *sequence_number);
std::vector<uint8_t> encrypt_2W_payload(const std::vector<uint8_t> &frame_data,
                                        const std::vector<uint8_t> &challenge, const uint8_t *key);
std::vector<uint8_t> decrypt_2W_payload(const std::vector<uint8_t> &encrypted_payload,
                                        const uint8_t *key);
}  // namespace iohcCrypto
#endif  //IOHCCRYPTOHELPERS_H
