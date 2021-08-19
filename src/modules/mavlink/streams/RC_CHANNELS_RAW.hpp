/****************************************************************************
 *
 *   Copyright (c) 2020 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

#ifndef RC_CHANNELS_RAW_HPP
#define RC_CHANNELS_RAW_HPP

#include <uORB/topics/input_rc_raw.h>

class MavlinkStreamRCChannelsRaw : public MavlinkStream
{
public:
	static MavlinkStream *new_instance(Mavlink *mavlink) { return new MavlinkStreamRCChannelsRaw(mavlink); }

	static constexpr const char *get_name_static() { return "RC_CHANNELS_RAW"; }
	static constexpr uint16_t get_id_static() { return MAVLINK_MSG_ID_RC_CHANNELS_RAW; }

	const char *get_name() const override { return get_name_static(); }
	uint16_t get_id() override { return get_id_static(); }

	unsigned get_size() override
	{
		return _input_rc_raw_sub.advertised() ? (MAVLINK_MSG_ID_RC_CHANNELS_RAW_LEN + MAVLINK_NUM_NON_PAYLOAD_BYTES) : 0;
	}

private:
	explicit MavlinkStreamRCChannelsRaw(Mavlink *mavlink) : MavlinkStream(mavlink) {}

	uORB::Subscription _input_rc_raw_sub{ORB_ID(input_rc_raw)};

	bool send() override
	{
		input_rc_raw_s rc;

		if (_input_rc_raw_sub.update(&rc)) {
			// send RC channel data and RSSI
			mavlink_rc_channels_raw_t msg{};

			msg.time_boot_ms = rc.timestamp / 1000;
			//msg.chancount = rc.channel_count;
			msg.chan1_raw  = (rc.channel_count > 0)  ? rc.values[0]  : UINT16_MAX;
			msg.chan2_raw  = (rc.channel_count > 1)  ? rc.values[1]  : UINT16_MAX;
			msg.chan3_raw  = (rc.channel_count > 2)  ? rc.values[2]  : UINT16_MAX;
			msg.chan4_raw  = (rc.channel_count > 3)  ? rc.values[3]  : UINT16_MAX;
			msg.chan5_raw  = (rc.channel_count > 4)  ? rc.values[4]  : UINT16_MAX;
			msg.chan6_raw  = (rc.channel_count > 5)  ? rc.values[5]  : UINT16_MAX;
			msg.chan7_raw  = (rc.channel_count > 6)  ? rc.values[6]  : UINT16_MAX;
			msg.chan8_raw  = (rc.channel_count > 7)  ? rc.values[7]  : UINT16_MAX;
			//msg.rssi = (rc.channel_count > 0) ? rc.rssi : 0;

			mavlink_msg_rc_channels_raw_send_struct(_mavlink->get_channel(), &msg);
			return true;
		}

		return false;
	}
};

#endif // RC_CHANNELS_RAW_HPP
