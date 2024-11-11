#pragma once
// MESSAGE AIRLINK_EYE_HP PACKING

#define MAVLINK_MSG_ID_AIRLINK_EYE_HP 52004


typedef struct __mavlink_airlink_eye_hp_t {
 uint8_t resp_type; /*<  Hole push response type*/
} mavlink_airlink_eye_hp_t;

#define MAVLINK_MSG_ID_AIRLINK_EYE_HP_LEN 1
#define MAVLINK_MSG_ID_AIRLINK_EYE_HP_MIN_LEN 1
#define MAVLINK_MSG_ID_52004_LEN 1
#define MAVLINK_MSG_ID_52004_MIN_LEN 1

#define MAVLINK_MSG_ID_AIRLINK_EYE_HP_CRC 39
#define MAVLINK_MSG_ID_52004_CRC 39



#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_AIRLINK_EYE_HP { \
    52004, \
    "AIRLINK_EYE_HP", \
    1, \
    {  { "resp_type", NULL, MAVLINK_TYPE_UINT8_T, 0, 0, offsetof(mavlink_airlink_eye_hp_t, resp_type) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_AIRLINK_EYE_HP { \
    "AIRLINK_EYE_HP", \
    1, \
    {  { "resp_type", NULL, MAVLINK_TYPE_UINT8_T, 0, 0, offsetof(mavlink_airlink_eye_hp_t, resp_type) }, \
         } \
}
#endif

/**
 * @brief Pack a airlink_eye_hp message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param resp_type  Hole push response type
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_airlink_eye_hp_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                               uint8_t resp_type)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_AIRLINK_EYE_HP_LEN];
    _mav_put_uint8_t(buf, 0, resp_type);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_AIRLINK_EYE_HP_LEN);
#else
    mavlink_airlink_eye_hp_t packet;
    packet.resp_type = resp_type;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_AIRLINK_EYE_HP_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_AIRLINK_EYE_HP;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_AIRLINK_EYE_HP_MIN_LEN, MAVLINK_MSG_ID_AIRLINK_EYE_HP_LEN, MAVLINK_MSG_ID_AIRLINK_EYE_HP_CRC);
}

/**
 * @brief Pack a airlink_eye_hp message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param status MAVLink status structure
 * @param msg The MAVLink message to compress the data into
 *
 * @param resp_type  Hole push response type
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_airlink_eye_hp_pack_status(uint8_t system_id, uint8_t component_id, mavlink_status_t *_status, mavlink_message_t* msg,
                               uint8_t resp_type)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_AIRLINK_EYE_HP_LEN];
    _mav_put_uint8_t(buf, 0, resp_type);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_AIRLINK_EYE_HP_LEN);
#else
    mavlink_airlink_eye_hp_t packet;
    packet.resp_type = resp_type;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_AIRLINK_EYE_HP_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_AIRLINK_EYE_HP;
#if MAVLINK_CRC_EXTRA
    return mavlink_finalize_message_buffer(msg, system_id, component_id, _status, MAVLINK_MSG_ID_AIRLINK_EYE_HP_MIN_LEN, MAVLINK_MSG_ID_AIRLINK_EYE_HP_LEN, MAVLINK_MSG_ID_AIRLINK_EYE_HP_CRC);
#else
    return mavlink_finalize_message_buffer(msg, system_id, component_id, _status, MAVLINK_MSG_ID_AIRLINK_EYE_HP_MIN_LEN, MAVLINK_MSG_ID_AIRLINK_EYE_HP_LEN);
#endif
}

/**
 * @brief Pack a airlink_eye_hp message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param resp_type  Hole push response type
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_airlink_eye_hp_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   uint8_t resp_type)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_AIRLINK_EYE_HP_LEN];
    _mav_put_uint8_t(buf, 0, resp_type);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_AIRLINK_EYE_HP_LEN);
#else
    mavlink_airlink_eye_hp_t packet;
    packet.resp_type = resp_type;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_AIRLINK_EYE_HP_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_AIRLINK_EYE_HP;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_AIRLINK_EYE_HP_MIN_LEN, MAVLINK_MSG_ID_AIRLINK_EYE_HP_LEN, MAVLINK_MSG_ID_AIRLINK_EYE_HP_CRC);
}

/**
 * @brief Encode a airlink_eye_hp struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param airlink_eye_hp C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_airlink_eye_hp_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_airlink_eye_hp_t* airlink_eye_hp)
{
    return mavlink_msg_airlink_eye_hp_pack(system_id, component_id, msg, airlink_eye_hp->resp_type);
}

/**
 * @brief Encode a airlink_eye_hp struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param airlink_eye_hp C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_airlink_eye_hp_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_airlink_eye_hp_t* airlink_eye_hp)
{
    return mavlink_msg_airlink_eye_hp_pack_chan(system_id, component_id, chan, msg, airlink_eye_hp->resp_type);
}

/**
 * @brief Encode a airlink_eye_hp struct with provided status structure
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param status MAVLink status structure
 * @param msg The MAVLink message to compress the data into
 * @param airlink_eye_hp C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_airlink_eye_hp_encode_status(uint8_t system_id, uint8_t component_id, mavlink_status_t* _status, mavlink_message_t* msg, const mavlink_airlink_eye_hp_t* airlink_eye_hp)
{
    return mavlink_msg_airlink_eye_hp_pack_status(system_id, component_id, _status, msg,  airlink_eye_hp->resp_type);
}

/**
 * @brief Send a airlink_eye_hp message
 * @param chan MAVLink channel to send the message
 *
 * @param resp_type  Hole push response type
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_airlink_eye_hp_send(mavlink_channel_t chan, uint8_t resp_type)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_AIRLINK_EYE_HP_LEN];
    _mav_put_uint8_t(buf, 0, resp_type);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_AIRLINK_EYE_HP, buf, MAVLINK_MSG_ID_AIRLINK_EYE_HP_MIN_LEN, MAVLINK_MSG_ID_AIRLINK_EYE_HP_LEN, MAVLINK_MSG_ID_AIRLINK_EYE_HP_CRC);
#else
    mavlink_airlink_eye_hp_t packet;
    packet.resp_type = resp_type;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_AIRLINK_EYE_HP, (const char *)&packet, MAVLINK_MSG_ID_AIRLINK_EYE_HP_MIN_LEN, MAVLINK_MSG_ID_AIRLINK_EYE_HP_LEN, MAVLINK_MSG_ID_AIRLINK_EYE_HP_CRC);
#endif
}

/**
 * @brief Send a airlink_eye_hp message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_airlink_eye_hp_send_struct(mavlink_channel_t chan, const mavlink_airlink_eye_hp_t* airlink_eye_hp)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_airlink_eye_hp_send(chan, airlink_eye_hp->resp_type);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_AIRLINK_EYE_HP, (const char *)airlink_eye_hp, MAVLINK_MSG_ID_AIRLINK_EYE_HP_MIN_LEN, MAVLINK_MSG_ID_AIRLINK_EYE_HP_LEN, MAVLINK_MSG_ID_AIRLINK_EYE_HP_CRC);
#endif
}

#if MAVLINK_MSG_ID_AIRLINK_EYE_HP_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This variant of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_airlink_eye_hp_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  uint8_t resp_type)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mav_put_uint8_t(buf, 0, resp_type);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_AIRLINK_EYE_HP, buf, MAVLINK_MSG_ID_AIRLINK_EYE_HP_MIN_LEN, MAVLINK_MSG_ID_AIRLINK_EYE_HP_LEN, MAVLINK_MSG_ID_AIRLINK_EYE_HP_CRC);
#else
    mavlink_airlink_eye_hp_t *packet = (mavlink_airlink_eye_hp_t *)msgbuf;
    packet->resp_type = resp_type;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_AIRLINK_EYE_HP, (const char *)packet, MAVLINK_MSG_ID_AIRLINK_EYE_HP_MIN_LEN, MAVLINK_MSG_ID_AIRLINK_EYE_HP_LEN, MAVLINK_MSG_ID_AIRLINK_EYE_HP_CRC);
#endif
}
#endif

#endif

// MESSAGE AIRLINK_EYE_HP UNPACKING


/**
 * @brief Get field resp_type from airlink_eye_hp message
 *
 * @return  Hole push response type
 */
static inline uint8_t mavlink_msg_airlink_eye_hp_get_resp_type(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  0);
}

/**
 * @brief Decode a airlink_eye_hp message into a struct
 *
 * @param msg The message to decode
 * @param airlink_eye_hp C-struct to decode the message contents into
 */
static inline void mavlink_msg_airlink_eye_hp_decode(const mavlink_message_t* msg, mavlink_airlink_eye_hp_t* airlink_eye_hp)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    airlink_eye_hp->resp_type = mavlink_msg_airlink_eye_hp_get_resp_type(msg);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_AIRLINK_EYE_HP_LEN? msg->len : MAVLINK_MSG_ID_AIRLINK_EYE_HP_LEN;
        memset(airlink_eye_hp, 0, MAVLINK_MSG_ID_AIRLINK_EYE_HP_LEN);
    memcpy(airlink_eye_hp, _MAV_PAYLOAD(msg), len);
#endif
}
