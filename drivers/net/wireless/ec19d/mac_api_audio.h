/*
  Copyright (c) 2004 by Nanoradio AB

  This software is copyrighted by and is the sole property of Nanoradio AB.

  All rights, title, ownership, or other interests in the software remain the
  property of Nanoradio AB.  This software may only be used in accordance with
  the corresponding license agreement.  Any unauthorized use, duplication,
  transmission, distribution, or disclosure of this software is expressly
  forbidden.

  This Copyright notice may not be removed or modified without prior written
  consent of Nanoradio AB.

  Nanoradio AB reserves the right to modify this software without notice.
 */
#ifndef MAC_API_AUDIO_H
#define MAC_API_AUDIO_H
#include "mac_api_defs.h"
/**

  @file mac_api_audio.h

  @defgroup audio Audio Interface

  Interface for using the audio codec to playback and record.

  @note This interface is only supported for Nanoradio WiFi chips with the
  audio interface capability.  To query if the decice supports this interface
  the bit corresponding to The HIC_MESSAGE_TYPE_AUDIO should be set in the
  supported_types field in HIC_SET_ALIGNMENT_CFM message.

*/


/**
  @defgroup audio_setup NRX_AUDIO_SETUP_REQ
  @ingroup audio
  @{

  Configure the audio interface.

  This message must be sent before any other message in this interface.
 */

/** Message ID for request message */
#define NRX_AUDIO_SETUP_REQ              (0 | MAC_API_PRIMITIVE_TYPE_REQ)
/** Message ID for confirm message */
#define NRX_AUDIO_SETUP_CFM              (0 | MAC_API_PRIMITIVE_TYPE_CFM)

typedef struct {
   /** Audio sample rate in kHz. */
   uint16_t rate;
   /** Number of channels to use. Possible values are 1 for mono and 2 for stereo. */
   uint8_t  channels;
   /** Reserved for future use. Must be set to zero. */
   uint8_t  reserved;
} nrx_audio_setup_req;

typedef struct {
   /** Status of the operation. Zero on success. */
   uint32_t status;
} nrx_audio_setup_cfm;

/** @} */


/**
  @defgroup audio_shutdown NRX_AUDIO_SHUTDOWN_REQ
  @ingroup audio
  @{

  Shutdown the audio interface.

  This message will power-down the audio interface.
 */

#define NRX_AUDIO_SHUTDOWN_REQ           (1 | MAC_API_PRIMITIVE_TYPE_REQ)
#define NRX_AUDIO_SHUTDOWN_CFM           (1 | MAC_API_PRIMITIVE_TYPE_CFM)

typedef struct {
   uint32_t reserved;
} nrx_audio_shutdown_req;

typedef struct {
   uint32_t status;
} nrx_audio_shutdown_cfm;

/** @} */


/**
  @defgroup audio_rec_start NRX_AUDIO_REC_START_REQ
  @ingroup audio
  @{

  Start recording audio.
 */

#define NRX_AUDIO_REC_START_REQ          (2 | MAC_API_PRIMITIVE_TYPE_REQ)
#define NRX_AUDIO_REC_START_CFM          (2 | MAC_API_PRIMITIVE_TYPE_CFM)

typedef struct {
   uint32_t chunk_size;
} nrx_audio_rec_start_req;

typedef struct {
   uint32_t status;
} nrx_audio_rec_start_cfm;

/** @} */


/**
  @defgroup audio_rec_stop NRX_AUDIO_REC_STOP_REQ
  @ingroup audio
  @{

  Stop recording audio.
 */

#define NRX_AUDIO_REC_STOP_REQ           (3 | MAC_API_PRIMITIVE_TYPE_REQ)
#define NRX_AUDIO_REC_STOP_CFM           (3 | MAC_API_PRIMITIVE_TYPE_CFM)

typedef struct {
   uint32_t reserved;
} nrx_audio_rec_stop_req;

typedef struct {
   uint32_t status;
} nrx_audio_rec_stop_cfm;

/** @} */



/**
  @defgroup audio_ctrl_req NRX_AUDIO_CTRL_REQ
  @ingroup audio
  @{

  Breif description of NRX_AUDIO_CTRL_REQ

  Detailed description of NRX_AUDIO_CTRL_REQ.
 */

#define NRX_AUDIO_CTRL_REQ               (4 | MAC_API_PRIMITIVE_TYPE_REQ)
#define NRX_AUDIO_CTRL_CFM               (4 | MAC_API_PRIMITIVE_TYPE_CFM)


typedef struct {
   uint8_t  vol_l;
   uint8_t  vol_r;
   uint16_t reserved;
} nrx_audio_ctrl_req;

typedef struct {
   uint32_t status;
} nrx_audio_ctrl_cfm;

/** @} */



/**
  @defgroup audio_play_req NRX_AUDIO_PLAY_REQ
  @ingroup audio
  @{

  Breif description of NRX_AUDIO_PLAY_REQ
  An audio play request consists of raw adio data preceded by the number of samples-1.

  Detailed description of NRX_AUDIO_PLAY_REQ.
 */

#define NRX_AUDIO_PLAY_REQ               (5 | MAC_API_PRIMITIVE_TYPE_REQ)
#define NRX_AUDIO_PLAY_CFM               (5 | MAC_API_PRIMITIVE_TYPE_CFM)


typedef struct {
   uint16_t num_samples;
} nrx_audio_play_req;

typedef struct {
   uint32_t status;
} nrx_audio_play_cfm;

/** @} */


/**
  @defgroup audio_rec_ind NRX_AUDIO_REC_IND
  @ingroup audio
  @{

  Message from device to host carrying one frame of recorded samples.
 */

#define NRX_AUDIO_REC_IND                (0 | MAC_API_PRIMITIVE_TYPE_IND)

typedef struct {
   uint16_t magic;
   uint16_t num_samples;
} nrx_audio_rec_ind;

/** @} */

#endif
