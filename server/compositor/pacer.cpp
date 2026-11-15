/*
 * WiVRn VR streaming
 * Copyright (C) 2024  Guillaume Meunier <guillaume.meunier@centraliens.net>
 * Copyright (C) 2024  Patrick Nicolas <patricknicolas@laposte.net>
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <https://www.gnu.org/licenses/>.
 */

#include "pacer.h"

#include "driver/clock_offset.h"

#include "util/u_var.h"
#include <algorithm>
#include <cmath>

#include "os/os_time.h"
#include "util/u_debug.h"
#include "util/u_logging.h"

DEBUG_GET_ONCE_FLOAT_OPTION(client_margin_ms, "WIVRN_CLIENT_MARGIN_MS", 1.f)

namespace wivrn
{

static const int64_t margin_ns = 3'000'000;
static const int64_t slop_ns = 500'000;

pacer::pacer(uint64_t frame_duration) :
        frame_duration_ns(frame_duration),
        client_margin_ms{
                .val = debug_get_float_option_client_margin_ms(),
                .step = 0.1,
                .min = 0,
                .max = 20,
        },
        frame_times(5000),
        frame_times_compute(frame_times),
        worker([this](std::stop_token t) {
	        std::vector<XrDuration> samples;
	        while (not t.stop_requested())
	        {
		        samples.clear();
		        {
			        std::unique_lock lock(compute_mutex);
			        compute_cv.wait(lock);
			        for (const auto & time: frame_times_compute)
			        {
				        if (time.decoded > time.present)
					        samples.push_back(time.decoded - time.present);
			        }
		        }
		        if (samples.empty())
			        continue;
		        // 本地改动：上游取 99.5 分位（保守，实测在本机 USB 链路上留出了 14.7 ms 缓冲，直接抬高端到端延迟）。
		        // 改取 99 分位，再配合客户端自适应缓冲吸收偶发晚帧。
		        // shortcut: 缓冲变小后每 100 帧可能有一帧晚到（表现为单次轻顿）；链路变差时要改回 995。
		        auto it = samples.begin() + (samples.size() * 99) / 100;
		        std::ranges::nth_element(samples, it);

		        std::unique_lock lock(mutex);
		        safe_present_to_decoded_ns = *it + client_margin_ms.val * U_TIME_1MS_IN_NS;
	        }
        })
{
	u_var_add_root(this, "Pacer", false);
	u_var_add_draggable_f32(this, &client_margin_ms, "headset margin (ms)");
}

pacer::~pacer()
{
	u_var_remove_root(this);
	worker.request_stop();
	compute_cv.notify_all();
}

uint64_t pacer::get_frame_duration() const
{
	std::lock_guard lock(mutex);
	return frame_duration_ns;
}

void pacer::set_frame_duration(uint64_t frame_duration_ns)
{
	std::lock_guard lock(mutex);
	this->frame_duration_ns = frame_duration_ns;
}

void pacer::predict(
        int64_t & frame_id,
        int64_t & out_wake_up_time_ns,
        int64_t & out_desired_present_time_ns,
        int64_t & out_present_slop_ns,
        int64_t & out_predicted_display_time_ns)
{
	std::lock_guard lock(mutex);
	frame_id = this->frame_id++;
	auto now = os_monotonic_get_ns();

	int64_t predicted_client_render = last_ns + frame_duration_ns;

	if (now + mean_wake_up_to_present_ns + safe_present_to_decoded_ns > predicted_client_render)
		predicted_client_render += frame_duration_ns * ((now + mean_wake_up_to_present_ns + safe_present_to_decoded_ns - predicted_client_render) / frame_duration_ns);

	out_predicted_display_time_ns = predicted_client_render + mean_render_to_display_ns;
	out_desired_present_time_ns = predicted_client_render - safe_present_to_decoded_ns;
	out_wake_up_time_ns = out_desired_present_time_ns - mean_wake_up_to_present_ns + margin_ns; // we should be awoken early by the application
	last_wake_up_ns = out_wake_up_time_ns;

	last_ns = predicted_client_render;

	in_flight_frames[frame_id % in_flight_frames.size()] = {
	        .frame_id = frame_id,
	        .present_ns = out_desired_present_time_ns,
	        .predicted_display_time = out_predicted_display_time_ns,
	};

	out_present_slop_ns = slop_ns;
}

void pacer::on_feedback(const wivrn::from_headset::feedback & feedback, const clock_offset & offset)
{
	if (feedback.times_displayed > 1 or not feedback.blitted)
		return;

	std::lock_guard lock(mutex);
	auto & when = in_flight_frames[feedback.frame_index % in_flight_frames.size()];
	if (when.frame_id != feedback.frame_index)
		return;

	auto & times = frame_times[feedback.frame_index % frame_times.size()];
	if (times.frame_id != feedback.frame_index)
	{
		times.frame_id = feedback.frame_index;
		times.present = when.present_ns;
		times.decoded = 0;
	}
	times.decoded = std::max(times.decoded, offset.from_headset(feedback.received_from_decoder));

	if (feedback.stream_index == 0)
	{
		if (feedback.frame_index % 100 == 0)
		{
			std::unique_lock lock(compute_mutex);
			std::swap(frame_times, frame_times_compute);
			compute_cv.notify_all();
		}
		// adjust phase
		auto d = (frame_duration_ns / 2 + offset.from_headset(feedback.blitted) - last_ns) % frame_duration_ns;
		if (d < 0)
			d += frame_duration_ns;
		d -= frame_duration_ns / 2;
		auto l = last_ns;
		last_ns += d / 10;
	}

	if (feedback.displayed and feedback.displayed > feedback.blitted and feedback.displayed < feedback.blitted + 100'000'000)
		mean_render_to_display_ns = std::lerp(mean_render_to_display_ns, feedback.displayed - feedback.blitted, 0.1);

	// 调试计时：WIVRN_TIMING_LOG=1 时逐帧打印流水线各段耗时（全部相对帧号，单位 ms）。
	// encode_begin 是服务端收到该帧、开始编码的时刻，displayed 是头显真正上屏的时刻，
	// 两者的差就是“这一帧从提交到上屏”的端到端延迟。
	static const bool timing_log = getenv("WIVRN_TIMING_LOG") != nullptr;
	if (timing_log and feedback.stream_index == 0 and feedback.blitted)
	{
		auto ms = [](int64_t ns) { return ns / 1e6; };
		auto decoded_server = offset.from_headset(feedback.received_from_decoder);
		U_LOG_I("TIMING frame=%" PRIu64
		        " enc=%.2f send=%.2f net=%.2f(decode-in %.2f) dec=%.2f blit=%.2f disp=%.2f | app->photon=%.2f safe=%.2f wake=%.2f r2d=%.2f shown=%u",
		        feedback.frame_index,
		        ms(feedback.encode_end - feedback.encode_begin),
		        ms(feedback.send_end - feedback.send_begin),
		        ms(offset.from_headset(feedback.received_first_packet) - offset.from_headset(feedback.send_end)),
		        ms(offset.from_headset(feedback.received_last_packet) - offset.from_headset(feedback.received_first_packet)),
		        ms(feedback.received_from_decoder - feedback.sent_to_decoder),
		        ms(feedback.blitted - feedback.received_from_decoder),
		        ms(feedback.displayed - feedback.blitted),
		        ms(feedback.displayed - feedback.encode_begin),
		        // 时隙三要素：safe 是“为了让 99.5% 的帧提前解码完成”而留的余量（随实测晚解码量自适应增长），
		        // wake 是应用被唤醒到提交的均值，r2d 是客户端报的“合成到上屏”。present 排在 now+safe+wake 之后，
		        // 所以 safe 一大，端到端就整体后移——这三项才是延迟的根，不是传输。
		        ms(safe_present_to_decoded_ns),
		        ms(mean_wake_up_to_present_ns),
		        ms(mean_render_to_display_ns),
		        unsigned(feedback.times_displayed));
	}
}
void pacer::mark_timing_point(
        comp_target_timing_point point,
        int64_t frame_id,
        int64_t when_ns)
{
	switch (point)
	{
		//! Woke up after sleeping in wait frame.
		case COMP_TARGET_TIMING_POINT_WAKE_UP:
			return;

		//! Began CPU side work for GPU.
		case COMP_TARGET_TIMING_POINT_BEGIN:
			return;

		//! Just before submitting work to the GPU.
		case COMP_TARGET_TIMING_POINT_SUBMIT_BEGIN:
			return;

		//! Just after submitting work to the GPU.
		case COMP_TARGET_TIMING_POINT_SUBMIT_END:
			if (when_ns > last_wake_up_ns and when_ns < last_wake_up_ns + 100'000'000)
			{
				std::lock_guard lock(mutex);
				mean_wake_up_to_present_ns = std::lerp(mean_wake_up_to_present_ns, when_ns - last_wake_up_ns, 0.1);
			}
	}
}

pacer::frame_info pacer::present_to_info(int64_t present)
{
	std::lock_guard lock(mutex);
	for (const auto & info: in_flight_frames)
	{
		if (info.present_ns == present)
			return info;
	}
	assert(false);
	return {};
}

void pacer::reset()
{
	std::lock_guard lock(mutex);
	std::ranges::fill(frame_times, frame_time{});
}
} // namespace wivrn
