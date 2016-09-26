// MESSAGE SET_NETID PACKING

#define MAVLINK_MSG_ID_SET_NETID 222

typedef struct __mavlink_set_netid_t
{
 uint16_t seq; ///< Sequence
} mavlink_set_netid_t;

#define MAVLINK_MSG_ID_SET_NETID_LEN 2
#define MAVLINK_MSG_ID_42_LEN 2

#define MAVLINK_MSG_ID_SET_NETID_CRC 28
#define MAVLINK_MSG_ID_222_CRC 28



#define MAVLINK_MESSAGE_INFO_SET_NETID { \
	"SET_NETID", \
	1, \
	{  { "seq", NULL, MAVLINK_TYPE_UINT16_T, 0, 0, offsetof(mavlink_set_netid_t, seq) }, \
         } \
}


/**
 * @brief Pack a set_netid message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param seq Sequence
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_set_netid_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
						       uint16_t seq)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_SET_NETID_LEN];
	_mav_put_uint16_t(buf, 0, seq);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_SET_NETID_LEN);
#else
	mavlink_set_netid_t packet;
	packet.seq = seq;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_SET_NETID_LEN);
#endif

	msg->msgid = MAVLINK_MSG_ID_SET_NETID;
#if MAVLINK_CRC_EXTRA
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_SET_NETID_LEN, MAVLINK_MSG_ID_SET_NETID_CRC);
#else
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_SET_NETID_LEN);
#endif
}

/**
 * @brief Pack a set_netid message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param seq Sequence
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_set_netid_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
							   mavlink_message_t* msg,
						           uint16_t seq)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_SET_NETID_LEN];
	_mav_put_uint16_t(buf, 0, seq);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_SET_NETID_LEN);
#else
	mavlink_set_netid_t packet;
	packet.seq = seq;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_SET_NETID_LEN);
#endif

	msg->msgid = MAVLINK_MSG_ID_SET_NETID;
#if MAVLINK_CRC_EXTRA
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_SET_NETID_LEN, MAVLINK_MSG_ID_SET_NETID_CRC);
#else
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_SET_NETID_LEN);
#endif
}

/**
 * @brief Encode a set_netid struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param set_netid C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_set_netid_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_set_netid_t* set_netid)
{
	return mavlink_msg_set_netid_pack(system_id, component_id, msg, set_netid->seq);
}

/**
 * @brief Encode a set_netid struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param set_netid C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_set_netid_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_set_netid_t* set_netid)
{
	return mavlink_msg_set_netid_pack_chan(system_id, component_id, chan, msg, set_netid->seq);
}

/**
 * @brief Send a set_netid message
 * @param chan MAVLink channel to send the message
 *
 * @param seq Sequence
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_set_netid_send(mavlink_channel_t chan, uint16_t seq)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_SET_NETID_LEN];
	_mav_put_uint16_t(buf, 0, seq);

#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_SET_NETID, buf, MAVLINK_MSG_ID_SET_NETID_LEN, MAVLINK_MSG_ID_SET_NETID_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_SET_NETID, buf, MAVLINK_MSG_ID_SET_NETID_LEN);
#endif
#else
	mavlink_set_netid_t packet;
	packet.seq = seq;

#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_SET_NETID, (const char *)&packet, MAVLINK_MSG_ID_SET_NETID_LEN, MAVLINK_MSG_ID_SET_NETID_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_SET_NETID, (const char *)&packet, MAVLINK_MSG_ID_SET_NETID_LEN);
#endif
#endif
}

#if MAVLINK_MSG_ID_SET_NETID_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_set_netid_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  uint16_t seq)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char *buf = (char *)msgbuf;
	_mav_put_uint16_t(buf, 0, seq);

#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_SET_NETID, buf, MAVLINK_MSG_ID_SET_NETID_LEN, MAVLINK_MSG_ID_SET_NETID_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_SET_NETID, buf, MAVLINK_MSG_ID_SET_NETID_LEN);
#endif
#else
	mavlink_set_netid_t *packet = (mavlink_set_netid_t *)msgbuf;
	packet->seq = seq;

#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_SET_NETID, (const char *)packet, MAVLINK_MSG_ID_SET_NETID_LEN, MAVLINK_MSG_ID_SET_NETID_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_SET_NETID, (const char *)packet, MAVLINK_MSG_ID_SET_NETID_LEN);
#endif
#endif
}
#endif

#endif

// MESSAGE SET_NETID UNPACKING


/**
 * @brief Get field seq from set_netid message
 *
 * @return Sequence
 */
static inline uint16_t mavlink_msg_set_netid_get_seq(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint16_t(msg,  0);
}

/**
 * @brief Decode a set_netid message into a struct
 *
 * @param msg The message to decode
 * @param set_netid C-struct to decode the message contents into
 */
static inline void mavlink_msg_set_netid_decode(const mavlink_message_t* msg, mavlink_set_netid_t* set_netid)
{
#if MAVLINK_NEED_BYTE_SWAP
	set_netid->seq = mavlink_msg_set_netid_get_seq(msg);
#else
	memcpy(set_netid, _MAV_PAYLOAD(msg), MAVLINK_MSG_ID_SET_NETID_LEN);
#endif
}
