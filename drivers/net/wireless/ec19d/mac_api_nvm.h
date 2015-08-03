/*******************************************************************************

            Copyright (c) 2004 by Nanoradio AB 

This software is copyrighted by and is the sole property of Nanoradio AB.
 All rights, title, ownership, or other interests in the
software remain the property of Nanoradio AB.  This software may
only be used in accordance with the corresponding license agreement.  Any
unauthorized use, duplication, transmission, distribution, or disclosure of
this software is expressly forbidden.

This Copyright notice may not be removed or modified without prior written
consent of Nanoradio AB.

Nanoradio AB reserves the right to modify this software without notice.

Nanoradio AB
Torshamnsgatan 39                  info@nanoradio.se
164 40 Kista                       http://www.nanoradio.com
SWEDEN
*******************************************************************************/
#ifndef MAC_API_NVM_H
#define MAC_API_NVM_H
#include "mac_api_defs.h"
#include "mac_api_mib.h"

/**
  @file mac_api_nvm.h

  @brief Scan

  @defgroup nvm Non-volatile memory interface.

  This file defines the interface for accessing the non-volatile memory
  accessible via the WiFi chip.

 */

/**
  @defgroup nvm_identity NRX_NVMEM_IDENTIFY_REQ
  @ingroup nvm
  @{

  Description.
 */

/** Message ID for requesting to identify Non-volatile memory device. */
#define NRX_NVMEM_IDENTIFY_REQ           (0 | MAC_API_PRIMITIVE_TYPE_REQ)

/** Message ID for the confirmation message of a IDENTIFY_REQ. */
#define NRX_NVMEM_IDENTIFY_CFM           (0 | MAC_API_PRIMITIVE_TYPE_CFM)

/** NV memory device description */
typedef struct
{
   /** HW Interface. The HW interface for the NV memory */
   uint8_t     hw_interface;

   /** Memory type. The exact type of the NV memory, vendor and model. */
   uint8_t     memtype;

   uint16_t    reserved;
} nrx_nvdev_t;

/** Message structure for a NRX_NVMEM_IDENTIFY_REQ */
typedef struct
{
   nrx_nvdev_t dev;
} nrx_nvmem_identify_req_t;

/** Message structure for a NRX_NVMEM_IDENTIFY_CFM */
typedef struct
{
   uint32_t    status;
   nrx_nvdev_t dev;
   uint32_t    page_size;
   uint32_t    reserved;
} nrx_nvmem_identify_cfm_t;

/** @} */

/**
  @defgroup nvm_erase NRX_NVMEM_ERASE_REQ
  @ingroup nvm
  @{

  Description.
 */

/** Message ID for requesting to erase a section of the NV memory. */
#define NRX_NVMEM_ERASE_REQ              (1 | MAC_API_PRIMITIVE_TYPE_REQ)

/** Message ID for the confirmation message of a NRX_NVMEM_ERASE_REQ. */
#define NRX_NVMEM_ERASE_CFM              (1 | MAC_API_PRIMITIVE_TYPE_CFM)

/** Message structure for a NRX_NVMEM_ERASE_REQ */
typedef struct
{
   /** NV device. The identity of the NV memory device */
   nrx_nvdev_t dev;

   /** Address. The address to start the erase from */ 
   uint32_t    addr;

   /** Size. The size of what should be erased */ 
   uint32_t    size;
} nrx_nvmem_erase_req_t;

/** Message structure for a NRX_NVMEM_ERASE_CFM */
typedef struct
{
   nrx_macapi_status_t  status;
} nrx_nvmem_erase_cfm_t;

/** @} */

/**
  @defgroup nvm_write NRX_NVMEM_WRITE_REQ
  @ingroup nvm
  @{

  Description.
 */

/** Message ID for requesting to write to a section of the NV memory. */
#define NRX_NVMEM_WRITE_REQ              (2 | MAC_API_PRIMITIVE_TYPE_REQ)

/** Message ID for the confirmation message of a NRX_NVMEM_WRITE_REQ. */
#define NRX_NVMEM_WRITE_CFM              (2 | MAC_API_PRIMITIVE_TYPE_CFM)

typedef struct
{
   /** NV device. The identity of the NV memory device */
   nrx_nvdev_t dev;

   /** Address. The address to start the writing at */ 
   uint32_t    addr;

   /** Size. The size of what should be written */ 
   uint32_t    size;
} nrx_nvmem_write_req_t;

typedef struct
{
   nrx_macapi_status_t  status;
} nrx_nvmem_write_cfm_t;

/** @} */

/**
  @defgroup nvm_read NRX_NVMEM_READ_REQ
  @ingroup nvm
  @{

  Description.
 */

/** Message ID for requesting to read from a section of the NV memory. */
#define NRX_NVMEM_READ_REQ               (3 | MAC_API_PRIMITIVE_TYPE_REQ)

/** Message ID for the confirmation message of a NRX_NVMEM_READ_REQ. */
#define NRX_NVMEM_READ_CFM               (3 | MAC_API_PRIMITIVE_TYPE_CFM)

typedef struct
{
   /** NV device. The identity of the NV memory device */
   nrx_nvdev_t dev;

   /** Address. The address to start reading from */ 
   uint32_t    addr;

   /** Size. The size of what should be read */ 
   uint32_t    size;
} nrx_nvmem_read_req_t;

typedef struct
{
   nrx_macapi_status_t  status;

   /** Size. The size of what has been read */ 
   uint32_t             size;
} nrx_nvmem_read_cfm_t;

/** @} */

/** Message ID for requesting to initialize MIBs from a NV memory. */
#define NVMEM_CFG_IN_NVM_TAG 0xFE

typedef struct 
{
   uint8_t cfg_tag;
   uint8_t version;
   uint8_t reserved[6];
}nrx_nvmem_cfg_header_t;

/** @} */

#endif
