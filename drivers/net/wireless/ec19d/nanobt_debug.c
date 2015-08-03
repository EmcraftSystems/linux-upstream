/* Copyright (c) 2012 by SNDC AB
 *
 * This software is copyrighted by and is the sole property of Samsung Nanoradio Design Centre (SNDC) AB.
 * All rights, title, ownership, or other interests in the software remain the property of SNDC AB.
 * This software may only be used in accordance with the corresponding license agreement.  Any unauthorized use, duplication, transmission, distribution, or disclosure of this software is expressly forbidden.
 *
 * This Copyright notice may not be removed or modified without prior written consent of SNDC AB.
 *
 * SNDC AB reserves the right to modify this software without notice.
 *
 * SNDC AB
 * Torshamnsgatan 39                       info@nanoradio.se
 * 164 40 Kista                            http://www.nanoradio.se
 * SWEDEN
 */

#include "driverenv.h"
#include "nanocore.h"
#include "nanobt_debug.h"

// HCI data types
#define HCI_COMMAND_PKT         0x01
#define HCI_ACLDATA_PKT         0x02
#define HCI_SCODATA_PKT         0x03
#define HCI_EVENT_PKT           0x04

// Some nice defines for disabling some debug logs
#define BT_SKIP_COMPLETED_PACKETS_EVENT
#define BT_SKIP_RX_L2CAP_DATA
#define BT_SKIP_TX_L2CAP_DATA

static const char *hci_cmd_str[0x09][0x6E] = {
{
   "OGF 0x00 IS EMPTY"
},
{
   "N/A_OGF01_OCF00",
   "INQUIRY",
   "INQUIRY_CANCEL",
   "PERIODIC_INQUIRY_MODE",
   "EXIT_PERIODIC_INQUIRY_MODE",
   "CREATE_CONNECTION",
   "DISCONNECT",
   "N/A_OGF01_OCF07",
   "CREATE_CONNECTION_CANCEL",
   "ACCEPT_CONNECTION_REQUEST",
   "REJECT_CONNECTION_REQUEST",
   "LINK_KEY_REQUEST_REPLY",
   "LINK_KEY_REQUEST_NEGATIVE_REPLY",
   "PIN_CODE_REQUEST_REPLY",
   "PIN_CODE_REQUEST_NEGATIVE_REPLY",
   "CHANGE_CONNECTION_PACKET_TYPE",
   "N/A_OGF01_OCF10",
   "AUTHENTICATION_REQUEST",
   "N/A_OGF01_OCF12",
   "SET_CONNECTION_ENCRYPTION",
   "N/A_OGF01_OCF14",
   "CHANGE_CONNECTION_LINK_KEY",
   "N/A_OGF01_OCF16",
   "MASTER_LINK_KEY",
   "N/A_OGF01_OCF18",
   "REMOTE_NAME_REQUEST",
   "REMOTE_NAME_REQUEST_CANCEL",
   "READ_REMOTE_SUPPORTED_FEATURES",
   "READ_REMOTE_EXTENDED_FEATURES",
   "READ_REMOTE_VERSION_INFORMATION",
   "N/A_OGF01_OCF1E",
   "READ_CLOCK_OFFSET",
   "READ_LMP_HANDLE",
   "N/A_OGF01_OCF21",
   "N/A_OGF01_OCF22",
   "N/A_OGF01_OCF23",
   "N/A_OGF01_OCF24",
   "N/A_OGF01_OCF25",
   "N/A_OGF01_OCF26",
   "N/A_OGF01_OCF27",
   "SETUP_SYNCHRONOUS_CONNECTION",
   "ACCEPT_SYNCHRONOUS_CONNECTION_REQUEST",
   "REJECT_SYNCHRONOUS_CONNECTION_REQUEST",
   "IO_CAPABILITY_REQUEST_REPLY",
   "USER_CONFIRMATION_REQUEST_REPLY",
   "USER_CONFIRMATION_REQUEST_NEGATIVE_REPLY",
   "USER_PASSKEY_REQUEST_REPLY",
   "USER_PASSKEY_REQUEST_NEGATIVE_REPLY",
   "REMOTE_OOB_DATA_REQUEST_REPLY",
   "N/A_OGF01_OCF31",
   "N/A_OGF01_OCF32",
   "REMOTE_OOB_DATA_REQUEST_NEGATIVE_REPLY",
   "IO_CAPABILITY_REQUEST_NEGATIVE_REPLY",
   "CREATE_PHYSICAL_LINK",
   "ACCEPT_PHYSICAL_LINK",
   "DISCONNECT_PHYSICAL_LINK",
   "CREATE_LOGICAL_LINK",
   "ACCEPT_LOGICAL_LINK",
   "DISCONNECT_LOGICAL_LINK",
   "LOGICAL_LINK_CANCEL",
   "FLOW_SPEC_MODIFY"
},
{
   "N/A_OGF02_OCF00",
   "HOLD_MODE",
   "N/A_OGF02_OCF02",
   "SNIFF_MODE",
   "EXIT_SNIFF_MODE",
   "PARK_STATE",
   "EXIT_PARK_STATE",
   "QOS_SETUP",
   "N/A_OGF02_OCF08",
   "ROLE_DISCOVERY",
   "N/A_OGF02_OCF0A",
   "SWITCH_ROLE",
   "READ_LINK_POLICY_SETTINGS",
   "WRITE_LINK_POLICY_SETTINGS",
   "READ_DEFAULT_LINK_POLICY_SETTINGS",
   "WRITE_DEFAULT_LINK_POLICY_SETTINGS",
   "FLOW_SPECIFICATION",
   "SNIFF_SUBRATING"
},
{
   "N/A_OGF03_OCF00",
   "SET_EVENT_MASK",
   "N/A_OGF03_OCF02",
   "RESET",
   "N/A_OGF03_OCF04",
   "SET_EVENT_FILTER",
   "N/A_OGF03_OCF06",
   "N/A_OGF03_OCF07",
   "FLUSH",
   "READ_PIN_TYPE",
   "WRITE_PIN_TYPE",
   "CREATE_NEW_UNIT_KEY",
   "N/A_OGF03_OCF0C",
   "READ_STORED_LINK_KEY",
   "N/A_OGF03_OCF0E",
   "N/A_OGF03_OCF0F",
   "N/A_OGF03_OCF10",
   "WRITE_STORED_LINK_KEY",
   "DELETE_STORED_LINK_KEY",
   "WRITE_LOCAL_NAME",
   "READ_LOCAL_NAME",
   "READ_CONNECTION_ACCEPT_TIMEOUT",
   "WRITE_CONNECTION_ACCEPT_TIMEOUT",
   "READ_PAGE_TIMEOUT",
   "WRITE_PAGE_TIMEOUT",
   "READ_SCAN_ENABLE",
   "WRITE_SCAN_ENABLE",
   "READ_PAGE_SCAN_ACTIVITY",
   "WRITE_PAGE_SCAN_ACTIVITY",
   "READ_INQUIRY_SCAN_ACTIVITY",
   "WRITE_INQUIRY_SCAN_ACTIVITY",
   "READ_AUTHENTICATION_ENABLE",
   "WRITE_AUTHENTICATION_ENABLE",
   "N/A_OGF03_OCF21",
   "N/A_OGF03_OCF22",
   "READ_CLASS_OF_DEVICE",
   "WRITE_CLASS_OF_DEVICE",
   "READ_VOICE_SETTINGS",
   "WRITE_VOICE_SETTINGS",
   "READ_AUTOMATIC_FLUSH_TIMEOUT",
   "WRITE_AUTOMATIC_FLUSH_TIMEOUT",
   "READ_NUM_BROADCAST_RETRANSMISSIONS",
   "WRITE_NUM_BROADCAST_RETRANSMISSIONS",
   "READ_HOLD_MODE_ACTIVITY",
   "WRITE_HOLD_MODE_ACTIVITY",
   "READ_TRANSMIT_POWER_LEVEL",
   "READ_SYNCHRONOUS_FLOW_CONTROL_ENABLE",
   "WRITE_SYNCHRONOUS_FLOW_CONTROL_ENABLE",
   "N/A_OGF03_OCF30",
   "SET_CONTROLLER_TO_HOST_FLOW_CONTROL",
   "N/A_OGF03_OCF32",
   "HOST_BUFFER_SIZE",
   "N/A_OGF03_OCF34",
   "HOST_NUMBER_OF_COMPLETED_PACKETS",
   "READ_LINK_SUPERVISION_TIMEOUT",
   "WRITE_LINK_SUPERVISION_TIMEOUT",
   "READ_NOF_SUPPORTED_IAC",
   "READ_CURRENT_IAC_LAP",
   "WRITE_CURRENT_IAC_LAP",
   "N/A_OGF03_OCF3B",
   "N/A_OGF03_OCF3C",
   "N/A_OGF03_OCF3D",
   "N/A_OGF03_OCF3E",
   "SET_AFH_HOST_CHANNEL_CLASSIFICATION",
   "N/A_OGF03_OCF40",
   "N/A_OGF03_OCF41",
   "READ_INQUIRY_SCAN_TYPE",
   "WRITE_INQUIRY_SCAN_TYPE",
   "READ_INQUIRY_MODE",
   "WRITE_INQUIRY_MODE",
   "READ_PAGE_SCAN_TYPE",
   "WRITE_PAGE_SCAN_TYPE",
   "READ_AFH_CHANNEL_ASSESSMENT_MODE",
   "WRITE_AFH_CHANNEL_ASSESSMENT_MODE",
   "N/A_OGF03_OCF4A",
   "N/A_OGF03_OCF4B",
   "N/A_OGF03_OCF4C",
   "N/A_OGF03_OCF4D",
   "N/A_OGF03_OCF4E",
   "N/A_OGF03_OCF4F",
   "N/A_OGF03_OCF50",
   "READ_EXTENDED_INQUIRY_RESPONSE",
   "WRITE_EXTENDED_INQUIRY_RESPONSE",
   "REFRESH_ENCRYPTION_KEY",
   "N/A_OGF03_OCF54",
   "READ_SIMPLE_PAIRING_MODE",
   "WRITE_SIMPLE_PAIRING_MODE",
   "READ_LOCAL_OOB_DATA",
   "READ_INQUIRY_RESPONSE_TRANSMIT_POWER_LEVEL",
   "WRITE_INQUIRY_TRANSMIT_POWER_LEVEL",
   "READ_DEFAULT_ERRONEOUS_DATA_REPORTING",
   "WRITE_DEFAULT_ERRONEOUS_DATA_REPORTING",
   "N/A_OGF03_OCF5C",
   "N/A_OGF03_OCF5D",
   "N/A_OGF03_OCF5E",
   "ENHANCED_FLUSH",
   "SEND_KEYPRESS_NOTIFICATION",
   "READ_LOGICAL_LINK_ACCEPT_TIMEOUT",
   "WRITE_LOGICAL_LINK_ACCEPT_TIMEOUT",
   "SET_EVENT_MASK_PAGE_2",
   "READ_LOCATION_DATA",
   "WRITE_LOCATION_DATA",
   "READ_FLOW_CONTROL_MODE",
   "WRITE_FLOW_CONTROL_MODE",
   "READ_ENHANCED_TRANSMIT_POWER_LEVEL",
   "READ_BEST_EFFORT_FLUSH_TIMEOUT",
   "WRITE_BEST_EFFORT_FLUSH_TIMEOUT",
   "SHORT_RANGE_MODE",
   "READ_LE_HOST_SUPPORTED",
   "WRITE_LE_HOST_SUPPORTED"
},
{
   "N/A_OGF04_OCF00",
   "READ_LOCAL_VERSION",
   "READ_LOCAL_SUPPORTED_COMMANDS",
   "READ_LOCAL_SUPPORTED_FEATURES",
   "READ_LOCAL_EXTENDED_FEATURES",
   "READ_BUFFER_SIZE",
   "N/A_OGF04_OCF06",
   "N/A_OGF04_OCF07",
   "N/A_OGF04_OCF08",
   "READ_BDADDR",
   "READ_DATA_BLOCK_SIZE"
},
{
   "N/A_OGF05_OCF00",
   "READ_FAILED_CONTACT_COUNTER",
   "WRITE_FAILED_CONTACT_COUNTER",
   "READ_LINK_QUALITY",
   "N/A_OGF05_OCF04",
   "READ_RSSI",
   "READ_AFH_CHANNEL_MAP",
   "READ_CLOCK",
   "READ_ENCRYPTION_KEY_SIZE",
   "READ_LOCAL_AMP_INFO",
   "READ_LOCAL_AMP_ASSOC",
   "WRITE_REMOTE_AMP_ASSOC"
},
{
   "N/A_OGF06_OCF00",
   "READ_LOCAL_LOOPBACK_MODE",
   "WRITE_LOCAL_LOOPBACK_MODE",
   "ENABLE_DEVICE_UNDER_TEST_MODE",
   "WRITE_SIMPLE_PAIRING_DEBUG_MODE",
   "ENABLE_AMP_RECEIVER_REPORTS",
   "AMP_TEST_END",
   "AMP_TEST"
},
{
   "OGF 0x07 IS EMPTY"
},
{
   "N/A_OGF_8_OCF00",
   "LE_SET_EVENT_MASK",
   "LE_READ_BUFFER_SIZE",
   "LE_READ_LOCAL_SUPPORTED_FEATURES",
   "N/A_OGF_8_OCF04",
   "LE_SET_RANDOM_ADDRESS",
   "LE_SET_ADVERTISING_PARAMETERS",
   "LE_READ_ADVERTISING_CHANNEL_TX_POWER",
   "LE_SET_ADVERTISING_DATA",
   "LE_SET_SCAN_RESPONSE_DATA",
   "LE_SET_ADVERTISING_ENABLE",
   "LE_SET_SCAN_PARAMETERS",
   "LE_SET_SCAN_ENABLE",
   "LE_CREATE_CONNECTION",
   "LE_CREATE_CONNECTION_CANCEL",
   "LE_READ_WHITE_LIST_SIZE",
   "LE_CLEAR_WHITE_LIST",
   "LE_ADD_DEVICE_TO_WHITE_LIST",
   "LE_REMOVE_DEVICE_FROM_WHITE_LIST",
   "LE_CONNECTION_UPDATE",
   "LE_SET_HOST_CHANNEL_CLASSIFICATION",
   "LE_READ_CHANNEL_MAP",
   "LE_READ_REMOTE_USED_FEATURES",
   "LE_ENCRYPT",
   "LE_RAND",
   "LE_START_ENCRYPTION",
   "LE_LONG_TERM_KEY_REQUEST_REPLY",
   "LE_LONG_TERM_KEY_REQUEST_NEGATIVE_REPLY",
   "LE_READ_LOCAL_SUPPORTED_STATES",
   "LE_RECEIVER_TEST",
   "LE_TRANSMITTER_TEST",
   "LE_END_TEST"
}
};

static const char *hci_event_str[] = {
   "BT_DEBUG",
   "INQUIRY_COMPLETE",
   "INQUIRY_RESULT",
   "CONNECTION_COMPLETE",
   "CONNECTION_REQUEST",
   "DISCONNECTION_COMPLETE",
   "AUTHENTICATION_COMPLETE",
   "REMOTE_NAME_REQUEST_COMPLETE",
   "ENCRYPTION_CHANGE",
   "CHANGE_CONN_LINK_KEY_COMPLETE",
   "NOT IMPLEMENTED 0x0A",
   "READ_REMOTE_SUPPORTED_FEATURES_COMPLETE",
   "READ_REMOTE_VERSION_INFORMATION_COMPLETE",
   "QOS_SETUP_COMPLETE",
   "COMMAND_COMPLETE",
   "COMMAND_STATUS",
   "HARDWARE_ERROR",
   "FLUSH_OCCURRED",
   "ROLE_CHANGE",
   "COMPLETED_PACKETS",
   "MODE_CHANGE",
   "RETURN_LINK_KEYS",
   "PIN_CODE_REQ",
   "LINK_KEY_REQ",
   "LINK_KEY_NOTIFICATION",
   "NOT IMPLEMENTED 0x19",
   "DATA_BUFFER_OVERFLOW",
   "MAX_SLOT_CHANGE",
   "READ_CLOCK_OFFSET_COMPLETE",
   "CONNECTION_PACKET_TYPE_CHANGED",
   "NOT IMPLEMENTED 0x1E",
   "NOT IMPLEMENTED 0x1F",
   "PAGE_SCAN_REPETITION_MODE_CHANGE",
   "FLOW_SPECIFICATION_COMPLETE",
   "INQUIRY_RESULT_WITH_RSSI",
   "READ_REMOTE_EXTENDED_FEATURES_COMPLETE",
   "NOT IMPLEMENTED 0x24",
   "NOT IMPLEMENTED 0x25",
   "NOT IMPLEMENTED 0x26",
   "NOT IMPLEMENTED 0x27",
   "NOT IMPLEMENTED 0x28",
   "NOT IMPLEMENTED 0x29",
   "NOT IMPLEMENTED 0x2A",
   "NOT IMPLEMENTED 0x2B",
   "SYNCHRONOUS_CONNECTION_COMPLETE",
   "SYNCHRONOUS_CONNECTION_CHANGED",
   "SNIFF_SUBRATING",
   "EXTENDED_INQUIRY_RESULT",
   "ENCRYPTION_KEY_REFRESH_COMPLETE",
   "IO_CAPABILITY_REQUEST",
   "IO_CAPABILITY_RESPONSE",
   "USER_CONFIRMATION_REQUEST",
   "USER_PASSKEY_REQUEST",
   "REMOTE_OOB_DATA_REQUEST",
   "SIMPLE_PAIRING_COMPLETE",
   "NOT IMPLEMENTED 0x37",
   "LINK_SUPERVISION_TIMEOUT_CHANGED",
   "NOT IMPLEMENTED 0x39",
   "NOT IMPLEMENTED 0x3A",
   "USER_PASSKEY_NOTIFICATION",
   "KEYPRESS_NOTIFICATION",
   "REMOTE_HOST_SUPPORTED_FEATURES_NOTIFICATION",
   "LE META EVENT", /* 0x3E */
};

static const char *hci_meta_event_str[] = {
   "LE_META_EVENT_0",
   "LE_CONNECTION_COMPLETE",
   "LE_ADVERTISING_REPORT",
   "LE_CONNECTION_UPDATE_COMPLETE",
   "LE_READ_REMOTE_USED_FEATURES_COMPLETE",
   "LE_LONG_TERM_KEY_REQUEST",
};

static const int hci_event_status_pos[] = {
   0, // "BT_DEBUG",
   1, // "INQUIRY_COMPLETE",
   0, // "INQUIRY_RESULT",
   1, // "CONNECTION_COMPLETE",
   0, // "CONNECTION_REQUEST",
   1, // "DISCONNECTION_COMPLETE",
   1, // "AUTHENTICATION_COMPLETE",
   1, // "REMOTE_NAME_REQUEST_COMPLETE",
   1, // "ENCRYPTION_CHANGE",
   1, // "CHANGE_CONN_LINK_KEY_COMPLETE",
   1, // "NOT IMPLEMENTED 0x0A",
   1, // "READ_REMOTE_SUPPORTED_FEATURES_COMPLETE",
   1, // "READ_REMOTE_VERSION_INFORMATION_COMPLETE",
   1, // "QOS_SETUP_COMPLETE",
   4, // "COMMAND_COMPLETE",
   1, // "COMMAND_STATUS",
   0, // "HARDWARE_ERROR",
   0, // "FLUSH_OCCURRED",
   1, // "ROLE_CHANGE",
   0, // "COMPLETED_PACKETS",
   1, // "MODE_CHANGE",
   0, // "RETURN_LINK_KEYS",
   0, // "PIN_CODE_REQ",
   0, // "LINK_KEY_REQ",
   0, // "LINK_KEY_NOTIFICATION",
   0, // "NOT IMPLEMENTED 0x19",
   0, // "NOT IMPLEMENTED 0x1A",
   0, // "MAX_SLOT_CHANGE",
   1, // "READ_CLOCK_OFFSET_COMPLETE",
   1, // "CONNECTION_PACKET_TYPE_CHANGED",
   0, // "NOT IMPLEMENTED 0x1E",
   0, // "NOT IMPLEMENTED 0x1F",
   0, // "PAGE_SCAN_REPETITION_MODE_CHANGE",
   1, // "FLOW_SPECIFICATION_COMPLETE",
   0, // "INQUIRY_RESULT_WITH_RSSI",
   1, // "READ_REMOTE_EXTENDED_FEATURES_COMPLETE",
   0, // "NOT IMPLEMENTED 0x24",
   0, // "NOT IMPLEMENTED 0x25",
   0, // "NOT IMPLEMENTED 0x26",
   0, // "NOT IMPLEMENTED 0x27",
   0, // "NOT IMPLEMENTED 0x28",
   0, // "NOT IMPLEMENTED 0x29",
   0, // "NOT IMPLEMENTED 0x2A",
   0, // "NOT IMPLEMENTED 0x2B",
   1, // "SYNCHRONOUS_CONNECTION_COMPLETE",
   1, // "SYNCHRONOUS_CONNECTION_CHANGED",
   1, // "SNIFF_SUBRATING",
   0, // "EXTENDED_INQUIRY_RESULT",
   1, // "ENCRYPTION_KEY_REFRESH_COMPLETE",
   0, // "IO_CAPABILITY_REQ",
   0, // "IO_CAPABILITY_RESP",
   0, // "USER_CONFIRMATION_REQ",
   0, // "USER_PASSKEY_REQ",
   0, // "REMOTE_OOB_DATA_REQ",
   1, // "SIMPLE_PAIRING_COMPLETE",
   0, // "NOT IMPLEMENTED 0x37",
   0, // "READ_CLOCK_OFFSET_COMPLETE",
   0, // "NOT IMPLEMENTED 0x39",
   0, // "NOT IMPLEMENTED 0x3A",
   0, // "USER_PASSKEY_NOTIFICATION",
   0, // "KEYPRESS_NOTIFICATION",
   0, // "REMOTE_HOST_SUPPORTED_FEATURES_NOTIFICATION",
   0, // "LE META EVENT",
};

static const char *lmp_str[] = {
   "UNDEF LMP 0",
   "NAME_REQ",
   "NAME_RES",
   "ACCEPTED",
   "NOT_ACCEPTED",
   "CLKOFFSET_REQ",
   "CLKOFFSET_RES",
   "DETACH",
   "IN_RAND",
   "COMB_KEY",
   "UNIT_KEY",
   "AU_RAND",
   "SRES",
   "TEMP_RAND",
   "TEMP_KEY",
   "ENCRYPTION_MODE_REQ",
   "ENCRYPRION_KEY_SIZE_REQ",
   "START_ENCRYPTION_REQ",
   "STOP_ENCRYPTION_REQ",
   "SWITCH_REQ",
   "HOLD",
   "HOLD_REQ",
   "UNDEF LMP 22",
   "SNIFF_REQ",
   "UNSNIFF_REQ",
   "PARK_REQ",
   "UNDEF LMP 26",
   "SET_BROADCAST_SCAN_WINDOW",
   "MODIFY_BEACON",
   "UNPARK_BD_ADDR_REQ",
   "UNPARK_PM_ADDR_REQ",
   "INCR_POWER_REQ",
   "DECR_POWER_REQ",
   "MAX_POWER",
   "MIN_POWER",
   "AUTO_RATE",
   "PREFERRED_RATE",
   "VERSION_REQ",
   "VERSION_RES",
   "FEATURES_REQ",
   "FEATURES_RES",
   "QUALITY_OF_SERVICE",
   "QUALITY_OF_SERVICE_REQ",
   "SCO_LINK_REQ",
   "REMOVE_SCO_LINK_REQ",
   "MAXSLOT",
   "MAXSLOT_REQ",
   "TIMING_ACCURACY_REQ",
   "TIMING_ACCURACY_RES",
   "SETUP_COMPLETE",
   "USE_SEMI_PERMANENT_KEY",
   "HOST_CONNECTION_REQ",
   "SLOT_OFFSET",
   "PAGE_MODE_REQ",
   "PAGE_SCAN_MODE_REQ",
   "SUPERVISION_TIMEOUT",
   "TEST_ACTIVATE",
   "TEST_CONTROL",
   "ENCRYPTION_KEY_SIZE_MASK_REQ",
   "ENCRYPTION_KEY_SIZE_MASK_RES",
   "SET_AFH",
   "ENCAPSULATED_HEADER",
   "ENCAPSULATED_PAYLOAD",
   "SIMPLE_PAIRING_CONFIRM",
   "SIMPLE_PAIRING_NUMBER",
   "DHKEY_CHECK",
   "UNDEF LMP 66",
   "UNDEF LMP 67",
   "UNDEF LMP 68",
   "UNDEF LMP 69",
   "UNDEF LMP 70",
   "UNDEF LMP 71",
   "UNDEF LMP 72",
   "UNDEF LMP 73",
   "UNDEF LMP 74",
   "UNDEF LMP 75",
   "UNDEF LMP 76",
   "UNDEF LMP 77",
   "UNDEF LMP 78",
   "UNDEF LMP 79",
   "UNDEF LMP 80",
   "UNDEF LMP 81",
   "UNDEF LMP 82",
   "UNDEF LMP 83",
   "UNDEF LMP 84",
   "UNDEF LMP 85",
   "UNDEF LMP 86",
   "UNDEF LMP 87",
   "UNDEF LMP 88",
   "UNDEF LMP 89",
   "UNDEF LMP 90",
   "UNDEF LMP 91",
   "UNDEF LMP 92",
   "UNDEF LMP 93",
   "UNDEF LMP 94",
   "UNDEF LMP 95",
   "UNDEF LMP 96",
   "UNDEF LMP 97",
   "UNDEF LMP 98",
   "UNDEF LMP 99",
   "UNDEF LMP 100",
   "UNDEF LMP 101",
   "UNDEF LMP 102",
   "UNDEF LMP 103",
   "UNDEF LMP 104",
   "UNDEF LMP 105",
   "UNDEF LMP 106",
   "UNDEF LMP 107",
   "UNDEF LMP 108",
   "UNDEF LMP 109",
   "UNDEF LMP 110",
   "UNDEF LMP 111",
   "UNDEF LMP 112",
   "UNDEF LMP 113",
   "UNDEF LMP 114",
   "UNDEF LMP 115",
   "UNDEF LMP 116",
   "UNDEF LMP 117",
   "UNDEF LMP 118",
   "UNDEF LMP 119",
   "UNDEF LMP 120",
   "UNDEF LMP 121",
   "UNDEF LMP 122",
   "UNDEF LMP 123",
   "ESCAPE_1",
   "ESCAPE_2",
   "ESCAPE_3",
   "ESCAPE_4",
};
static const char *lmp_ext_str[] = {
   "UNDEF LMP EXT 0",
   "ACCEPTED_EXT",
   "NOT_ACCEPTED_EXT",
   "FEATURES_REQ_EXT",
   "FEATURES_RESP_EXT",
   "UNDEF LMP EXT 5",
   "UNDEF LMP EXT 6",
   "UNDEF LMP EXT 7",
   "UNDEF LMP EXT 8",
   "UNDEF LMP EXT 9",
   "UNDEF LMP EXT 10",
   "PACKET_TYPE_TABLE_REQ",
   "ESCO_LINK_REQ",
   "REMOVE_ESCO_LINK_REQ",
   "UNDEF LMP EXT 14",
   "UNDEF LMP EXT 15",
   "CHANNEL_CLASSIFICATION_REQ",
   "CHANNEL_CLASSIFICATION",
   "UNDEF LMP EXT 18",
   "UNDEF LMP EXT 19",
   "UNDEF LMP EXT 20",
   "SNIFF_SUBRATING_REQ",
   "SNIFF_SUBRATING_RES",
   "PAUSE_ENCRYPTION_REQ",
   "RESUME_ENCRYPTION_REQ",
   "IO_CAPABILITY_REQ",
   "IO_CAPABILITY_RES",
   "NUMERIC_COMPARISION_FAILED",
   "PASSKEY_FAILED",
   "OOB_FAILED",
   "KEYPRESS_NOTIFICATION",
   "POWER_CONTROL_REQ",
   "POWER_CONTROL_RES",
};
static const char *lmp_dir_str[] = {
   "TX",
   "RX",
};

static void
nanobt_printbuf(const void *ptr, size_t len)
{
   size_t i;
   const unsigned char *p = ptr;
   char line[255];
   line[0] = '\0';

   for (i = 0; i < len; i++) {
      snprintf(line + strlen(line), sizeof(line) - strlen(line), "%c", p[i]);
   }
   de_print("BT: %s", line);
}

/* buf contains a full HCI message including the type as first byte */
void
nanobt_debug_log_tx(unsigned char *buf, size_t len)
{
   if (buf[0] == HCI_COMMAND_PKT)
   {
      uint8_t ogf = buf[2] >> 2;
      uint8_t ocf = buf[1] + ((buf[2] & 0x03) << 8);
      if (((ogf >= 1) && (ogf <= 6)) || (ogf == 8))
      {
        de_print("HCI_%s_COMMAND: ", hci_cmd_str[ogf][ocf]);
        nanoc_printbuf("    ", &buf[3], len-3); // length and parameters
      }
      else
      {
        de_print("ERROR: Strange OGF: %d", ogf);
        nanoc_printbuf("HCI-COMMAND", &buf[1], len-1);
      }
   }
   else if (buf[0] == HCI_ACLDATA_PKT)
   {
#ifndef BT_SKIP_TX_L2CAP_DATA
//      nanoc_printbuf("HCI-TX-L2CAP", &buf[1], len-1);
      de_print("HCI-TX-L2CAP: length %d", len);
#endif
   }
   else if (buf[0] == HCI_SCODATA_PKT)
   {
      nanoc_printbuf("HCI-TX-SCO", &buf[1], len-1);
   }
   else
   {
      de_debug(DE_TR_BT, "HCI-TX Unexp packet indicator: %d", buf[0]);
   }
}

/* buf contains a full HCI message including the type as first byte */
void
nanobt_debug_log_rx(unsigned char *buf, size_t len)
{
   uint8_t event_code;
   switch (buf[0]) {
      case HCI_ACLDATA_PKT:
      {
//#define BT_BATT_DATA_DBG
#ifdef BT_BATT_DATA_DBG
         /* buf[0]    - indicator
          * buf[1..2] - handle + flags
          * buf[3..4] - hci length
          * buf[5..]  - hci payload 
          *   buf[5..6] - l2cap length
          *   buf[7..8] - l2cap id
          */
#define NOF_ACL_HANDLES 3
         static uint8_t exp_data[NOF_ACL_HANDLES]; /* expected first payload byte in next payload packet */
         static int32_t l2cap_remaining[NOF_ACL_HANDLES] = {0, 0, 0}; /* i.e. waiting for start */
         uint16_t pay_len = buf[3] + (buf[4] << 8);
         uint8_t handle = buf[1] - 1; /* We use 1,2,3 in FW. */

         if (((buf[2] & 0x30) >> 4) == 2) // START
         {
           if (l2cap_remaining[handle] > 0) de_print("HCI-RX-L2CAP ERROR: Unexp START l2cap_remaining: %d", l2cap_remaining[handle]);

           l2cap_remaining[handle] = buf[5] + (buf[6] << 8);
           pay_len -= 4; // skip 4 bytes l2cap header
           de_print("HCI-RX-L2CAP START: l2cap_len: %d, pay_len: %d, 1st: 0x%02x", l2cap_remaining[handle], pay_len, buf[9]);
           exp_data[handle] = buf[9];
         }
         else // CONT
         {
           if (buf[5] != exp_data[handle]) de_print("HCI-RX-L2CAP ERROR: Unexp CONT 1st data 0%02x, exp: 0%02x", buf[5], exp_data[handle]);
         }
         exp_data[handle] += pay_len;
         l2cap_remaining[handle] -= pay_len;
         if (l2cap_remaining[handle] == 0)
         {
           de_print("HCI-RX-L2CAP COMPLETED");
         }
         else if (l2cap_remaining[handle] < 0)
         {
           de_print("HCI-RX-L2CAP ERROR: l2cap_remaining < 0 (%d)", l2cap_remaining[handle]);
         }
#else
#ifndef BT_SKIP_RX_L2CAP_DATA
//         nanoc_printbuf("HCI-RX-L2CAP", &buf[1], len-1);
         de_print("HCI-RX-L2CAP: length %d", len);
#endif
#endif
         break;
      }
      case HCI_EVENT_PKT:
         event_code = buf[1];
         if(event_code > 0) {
            /* An HCI event. */
#ifdef BT_SKIP_COMPLETED_PACKETS_EVENT
            if (event_code == 0x13) break;  /* COMPLETED_PACKETS_EVENT */
#endif
            if (hci_event_status_pos[event_code] > 0) { /* Event with status code */
               uint8_t error_code = buf[2 + hci_event_status_pos[event_code]];
               if (error_code == 0) {
                  de_print("HCI_%s_EVENT: OK", hci_event_str[event_code]);
               }
               else {
                  de_print("HCI_%s_EVENT: ERROR 0x%x", hci_event_str[event_code], error_code);
               }
            }
            else {
               if (event_code < 0x3E) {
                  de_print("HCI_%s_EVENT: ", hci_event_str[event_code]);
               }
               else if (event_code == 0x3E) {  /* LE_META_EVENT */
                  uint8_t subevent_code = buf[3];
                  if (subevent_code <= 5) {
                     de_print("HCI_%s_EVENT: ", hci_meta_event_str[subevent_code]);
                  }
                  else {
                     de_print("ERROR: Unexpected subevent code (%d)", subevent_code);
                  }
               }
               else {
                  de_print("ERROR: Unexpected event code (%d)", event_code);
               }
            }
            nanoc_printbuf("    ", &buf[1], len-1);
         }
         else {
            /* We use event_code == 0 for Nanoradio debug.
	     * Trace if enabled, bt do not pass to Linux BT stack. */
            uint8_t code = buf[3]; /* LM debug event */
            if (code <= 1) /* LMP */
            {
               uint8_t op_code = (buf[8] >> 1);
               uint8_t initiator = (buf[8] & 0x01);
               if (op_code < 124) /* Not ESCAPE_1..4 */
               {
                  de_print("LMP-%s %s(0x%02x) (i%d), length=%02zu",
                     lmp_dir_str[code], lmp_str[op_code], op_code, initiator, len-7);
                  nanoc_printbuf("   ", &buf[8], len-8);
               }
               else if (op_code == 127) /* ESCAPE_4 */
               {
                  uint8_t ext_op_code = buf[9];
                  de_print("LMP-%s %s(0x%02x/0x%02x) (i%d), length=%02zu ",
                     lmp_dir_str[code], lmp_ext_str[ext_op_code], op_code, ext_op_code, initiator, len-7);
                  nanoc_printbuf("   ", &buf[8], len-8);
               }
               else /* ESCAPE_2 or ESCAPE_3 */
               {
                  de_debug(DE_TR_BT, "Unexpected %s LMP opcode %d (%s)",
                     lmp_dir_str[code], op_code, lmp_str[op_code]);
               }
            }
            else if (code == 255)
            {
               /* DBG string - skip 7 bytes. */
               nanobt_printbuf(&buf[8], len-8);
            }
            else
            {
               de_debug(DE_TR_BT, "Unexpected BT DBG code (%d)", code);
            }
         }
         break;

      default:
         nanoc_printbuf("HCI-UNKNOWN", &buf[1], len);
         break;
   }
}
