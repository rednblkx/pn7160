#pragma once

#include <cstddef>
#include <cstdint>

// --- NCI Constants (NCI Core Spec v2.0 & PN7160 UM11495) ---
namespace nci {

// Message Types (MT)
inline constexpr uint8_t PKT_MT_DATA                = 0x00;
inline constexpr uint8_t PKT_MT_CTRL_COMMAND        = 0x01;
inline constexpr uint8_t PKT_MT_CTRL_RESPONSE       = 0x02; // NCI 2.0
inline constexpr uint8_t PKT_MT_CTRL_NOTIFICATION   = 0x03; // NCI 2.0

// Group IDs (GID)
inline constexpr uint8_t CORE_GID        = 0x00;
inline constexpr uint8_t RF_GID          = 0x01;
inline constexpr uint8_t NFCEE_MGMT_GID  = 0x02;
inline constexpr uint8_t PROPRIETARY_GID = 0x0F;

// Opcode IDs (OID) - Core GID
inline constexpr uint8_t CORE_RESET_OID             = 0x00;
inline constexpr uint8_t CORE_INIT_OID              = 0x01;
inline constexpr uint8_t CORE_SET_CONFIG_OID        = 0x02;
inline constexpr uint8_t CORE_GET_CONFIG_OID        = 0x03;
inline constexpr uint8_t CORE_CONN_CREATE_OID       = 0x04;
inline constexpr uint8_t CORE_CONN_CLOSE_OID        = 0x05;
inline constexpr uint8_t CORE_CONN_CREDITS_OID      = 0x06;
inline constexpr uint8_t CORE_GENERIC_ERROR_OID     = 0x07;
inline constexpr uint8_t CORE_INTERFACE_ERROR_OID   = 0x08;
inline constexpr uint8_t CORE_SET_POWER_SUB_STATE_OID = 0x09;

// Opcode IDs (OID) - RF Management GID
inline constexpr uint8_t RF_DISCOVER_MAP_OID            = 0x00;
inline constexpr uint8_t RF_SET_LISTEN_MODE_ROUTING_OID = 0x01;
inline constexpr uint8_t RF_GET_LISTEN_MODE_ROUTING_OID = 0x02;
inline constexpr uint8_t RF_DISCOVER_OID                = 0x03;
inline constexpr uint8_t RF_DISCOVER_SELECT_OID         = 0x04;
inline constexpr uint8_t RF_INTF_ACTIVATED_OID          = 0x05;
inline constexpr uint8_t RF_DEACTIVATE_OID              = 0x06;
inline constexpr uint8_t RF_FIELD_INFO_OID              = 0x07;
inline constexpr uint8_t RF_T3T_POLLING_OID             = 0x08;
inline constexpr uint8_t RF_NFCEE_ACTION_OID            = 0x09;
inline constexpr uint8_t RF_NFCEE_DISCOVERY_REQ_OID     = 0x0A;
inline constexpr uint8_t RF_PARAMETER_UPDATE_OID        = 0x0B;
inline constexpr uint8_t RF_INTF_EXT_START_OID          = 0x0C;
inline constexpr uint8_t RF_INTF_EXT_STOP_OID           = 0x0D;
inline constexpr uint8_t RF_EXT_AGG_ABORT_OID           = 0x0E;
inline constexpr uint8_t RF_NDEF_ABORT_OID              = 0x0F;
inline constexpr uint8_t RF_ISO_DEP_NAK_PRESENCE_OID    = 0x10;

inline constexpr uint8_t RF_DISCOVER_MAP_MODE_POLL   = 0x1;
inline constexpr uint8_t RF_DISCOVER_MAP_MODE_LISTEN = 0x2;

// Opcode IDs (OID) - NFCEE Management GID
inline constexpr uint8_t NFCEE_DISCOVER_OID            = 0x00;
inline constexpr uint8_t NFCEE_MODE_SET_OID            = 0x01;
inline constexpr uint8_t NFCEE_STATUS_OID              = 0x02;
inline constexpr uint8_t NFCEE_POWER_AND_LINK_CNTRL_OID = 0x03;

// NCI Status Codes (NCI Core Spec v2.0 Table 129)
inline constexpr uint8_t STATUS_OK                  = 0x00;
inline constexpr uint8_t STATUS_REJECTED            = 0x01;
inline constexpr uint8_t STATUS_RF_FRAME_CORRUPTED  = 0x02;
inline constexpr uint8_t STATUS_FAILED              = 0x03;
inline constexpr uint8_t STATUS_NOT_INITIALIZED     = 0x04;
inline constexpr uint8_t STATUS_SYNTAX_ERROR        = 0x05;
inline constexpr uint8_t STATUS_SEMANTIC_ERROR      = 0x06;
inline constexpr uint8_t STATUS_INVALID_PARAM       = 0x09;
inline constexpr uint8_t STATUS_MESSAGE_SIZE_EXCEEDED = 0x0A;
inline constexpr uint8_t DISCOVERY_ALREADY_STARTED          = 0xA0;
inline constexpr uint8_t DISCOVERY_TARGET_ACTIVATION_FAILED = 0xA1;
inline constexpr uint8_t DISCOVERY_TEAR_DOWN                = 0xA2;
inline constexpr uint8_t RF_TRANSMISSION_ERROR  = 0xB0;
inline constexpr uint8_t RF_PROTOCOL_ERROR      = 0xB1;
inline constexpr uint8_t RF_TIMEOUT_ERROR       = 0xB2;
inline constexpr uint8_t NFCEE_INTERFACE_ACTIVATION_FAILED = 0xC0;
inline constexpr uint8_t NFCEE_TRANSMISSION_ERROR          = 0xC1;
inline constexpr uint8_t NFCEE_PROTOCOL_ERROR              = 0xC2;
inline constexpr uint8_t NFCEE_TIMEOUT_ERROR               = 0xC3;
// Proprietary Status Codes (UM11495 Section 7.4.8 / Table 23)
inline constexpr uint8_t STATUS_LPCD_FAKE_DETECTION  = 0xA3;
inline constexpr uint8_t STATUS_BOOT_TRIM_CORRUPTED  = 0xE1;
inline constexpr uint8_t STATUS_EMVCO_PCD_COLLISION  = 0xE4;

// Deactivation Types (NCI Core Spec v2.0 Section 7.3.2)
inline constexpr uint8_t DEACTIVATION_TYPE_IDLE      = 0x00;
inline constexpr uint8_t DEACTIVATION_TYPE_SLEEP     = 0x01;
inline constexpr uint8_t DEACTIVATION_TYPE_SLEEP_AF  = 0x02;
inline constexpr uint8_t DEACTIVATION_TYPE_DISCOVERY = 0x03;

// RF Technologies (NCI Core Spec v2.0 Table 130)
inline constexpr uint8_t TECH_NFCA = 0x00;
inline constexpr uint8_t TECH_NFCB = 0x01;
inline constexpr uint8_t TECH_NFCF = 0x02;
inline constexpr uint8_t TECH_NFCV = 0x03;

// RF Technology and Mode (NCI Core Spec v2.0 Table 131)
inline constexpr uint8_t MODE_PASSIVE_POLL_A   = 0x00;
inline constexpr uint8_t MODE_PASSIVE_POLL_B   = 0x01;
inline constexpr uint8_t MODE_PASSIVE_POLL_F   = 0x02;
inline constexpr uint8_t MODE_ACTIVE_POLL      = 0x03;
inline constexpr uint8_t MODE_PASSIVE_POLL_V   = 0x06;
inline constexpr uint8_t MODE_PASSIVE_LISTEN_A = 0x80;
inline constexpr uint8_t MODE_PASSIVE_LISTEN_B = 0x81;
inline constexpr uint8_t MODE_PASSIVE_LISTEN_F = 0x82;
inline constexpr uint8_t MODE_ACTIVE_LISTEN    = 0x83;

// RF Protocols (NCI Core Spec v2.0 Table 133)
inline constexpr uint8_t PROT_UNDETERMINED = 0x00;
inline constexpr uint8_t PROT_T1T         = 0x01;
inline constexpr uint8_t PROT_T2T         = 0x02;
inline constexpr uint8_t PROT_T3T         = 0x03;
inline constexpr uint8_t PROT_ISODEP      = 0x04;
inline constexpr uint8_t PROT_NFCDEP      = 0x05;
inline constexpr uint8_t PROT_T5T         = 0x06;
inline constexpr uint8_t PROT_NDEF        = 0x07;
inline constexpr uint8_t PROT_MIFARE      = 0x80;

// RF Interfaces (NCI Core Spec v2.0 Table 134)
inline constexpr uint8_t INTF_NFCEE_DIRECT = 0x00;
inline constexpr uint8_t INTF_FRAME        = 0x01;
inline constexpr uint8_t INTF_ISODEP       = 0x02;
inline constexpr uint8_t INTF_NFCDEP       = 0x03;
inline constexpr uint8_t INTF_NDEF         = 0x06;
inline constexpr uint8_t INTF_TAGCMD       = 0x80;

// --- PN7160 Specific ---
inline constexpr uint16_t PN7160_DEFAULT_TIMEOUT_MS = 200;
inline constexpr uint16_t PN7160_INIT_TIMEOUT_MS    = 200;
inline constexpr uint16_t PN7160_IRQ_TIMEOUT_MS     = 250;
inline constexpr size_t   NCI_HEADER_SIZE            = 3;
inline constexpr size_t   NCI_MAX_PAYLOAD_SIZE       = 255;
inline constexpr size_t   NCI_MAX_PACKET_SIZE        = NCI_HEADER_SIZE + NCI_MAX_PAYLOAD_SIZE;

} // namespace nci
