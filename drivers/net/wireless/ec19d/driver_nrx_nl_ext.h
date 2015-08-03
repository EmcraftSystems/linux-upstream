

#ifndef NRX_NL_EXTENSIONS_H
#define NRX_NL_EXTENSIONS_H

typedef enum {
   NRX_CMD_RESERVED = 0,
   NRX_CMD_SET_NOA,
   NRX_CMD_SET_OPP,
   NRX_CMD_SET_PS,
   NRX_CMD_SET_AP_WPS_IE,
   NRX_CMD_RXFILTER,
   NRX_CMD_RESTART_PEERS,
   NRX_CMD_LAST
} nrx_cmd_id_t;

typedef struct {
   uint8_t        id;   /* nrx_cmd_id_t */
   uint8_t        count;
   uint32_t       start;
   uint32_t       duration;
   uint32_t       interval;
} __attribute__ ((packed)) nrx_buf_noa_t;

typedef struct {
   uint8_t        id;   /* nrx_cmd_id_t */
   int8_t         opp_enabled;
   int8_t         ctwindow;
} __attribute__ ((packed)) nrx_buf_opp_t;

typedef struct {
   uint8_t        id;   /* nrx_cmd_id_t */
   uint8_t        ie_type;
#define NRX_BUF_IE_BEACON    1
#define NRX_BUF_IE_PROBERSP  2
#define NRX_BUF_IE_ASSOCRSP  4
   uint16_t       ie_len;
#define NRX_NL_MAX_IE_BUF    256
   uint8_t        ie_buf[NRX_NL_MAX_IE_BUF];
} __attribute__ ((packed)) nrx_buf_ie_t;


typedef enum {
    NRX_RXFILTER_ADD = 0,
    NRX_RXFILTER_REMOVE,
    NRX_RXFILTER_START,
    NRX_RXFILTER_STOP,
} nrx_cmd_rxfilter_type_t;

typedef enum {
    NRX_RXFILTER_UNICAST    = 1,
    NRX_RXFILTER_BROADCAST  = 2,
    NRX_RXFILTER_MULTICAST4 = 4,
    NRX_RXFILTER_MULTICAST6 = 8,
} nrx_cmd_rxfilter_filters_t;

typedef struct {
    uint8_t  id;   /* nrx_cmd_id_t */
    uint8_t  type;
    uint8_t  filter;
} __attribute__ ((packed)) nrx_buf_rxfilter_t;

typedef struct {
   uint8_t        id;   /* nrx_cmd_id_t */
   uint8_t        iftype;
} __attribute__ ((packed)) nrx_buf_restart_t;

#endif
