/****************************************************************************
 *
 *   Copyright (c) 2013-2019 PX4 Development Team. All rights reserved.
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

#include <drivers/drv_hrt.h>
#include <lib/ecl/attitude_fw/ecl_pitch_controller.h>
#include <lib/ecl/attitude_fw/ecl_roll_controller.h>
#include <lib/ecl/attitude_fw/ecl_wheel_controller.h>
#include <lib/ecl/attitude_fw/ecl_yaw_controller.h>
#include <lib/ecl/geo/geo.h>
#include <lib/mathlib/mathlib.h>
#include <lib/parameters/param.h>
#include <lib/perf/perf_counter.h>
#include <matrix/math.hpp>
#include <px4_platform_common/px4_config.h>
#include <px4_platform_common/defines.h>
#include <px4_platform_common/module.h>
#include <px4_platform_common/module_params.h>
#include <px4_platform_common/posix.h>
#include <px4_platform_common/tasks.h>
#include <px4_platform_common/px4_work_queue/WorkItem.hpp>
#include <uORB/Publication.hpp>
#include <uORB/PublicationMulti.hpp>
#include <uORB/Subscription.hpp>
#include <uORB/SubscriptionCallback.hpp>
#include <uORB/topics/actuator_controls.h>
#include <uORB/topics/airspeed_validated.h>
#include <uORB/topics/battery_status.h>
#include <uORB/topics/manual_control_setpoint.h>
#include <uORB/topics/parameter_update.h>
#include <uORB/topics/rate_ctrl_status.h>
#include <uORB/topics/vehicle_angular_velocity.h>
#include <uORB/topics/vehicle_attitude.h>
#include <uORB/topics/vehicle_attitude_setpoint.h>
#include <uORB/topics/vehicle_control_mode.h>
#include <uORB/topics/vehicle_global_position.h>
#include <uORB/topics/vehicle_land_detected.h>
#include <uORB/topics/vehicle_rates_setpoint.h>
#include <uORB/topics/vehicle_status.h>

using matrix::Eulerf;
using matrix::Quatf;

using uORB::SubscriptionData;

class SymmetricGliderAttitudeControl final : public ModuleBase<SymmetricGliderAttitudeControl>, public ModuleParams,
	public px4::WorkItem
{
public:
	SymmetricGliderAttitudeControl();
	~SymmetricGliderAttitudeControl() override;

	/** @see ModuleBase */
	static int task_spawn(int argc, char *argv[]);

	/** @see ModuleBase */
	static int custom_command(int argc, char *argv[]);

	/** @see ModuleBase */
	static int print_usage(const char *reason = nullptr);

	void Run() override;

	bool init();

private:

	uORB::SubscriptionCallbackWorkItem _att_sub{this, ORB_ID(vehicle_attitude)};	/**< vehicle attitude */

	uORB::Subscription _att_sp_sub{ORB_ID(vehicle_attitude_setpoint)};		/**< vehicle attitude setpoint */
	uORB::Subscription _battery_status_sub{ORB_ID(battery_status)};			/**< battery status subscription */
	uORB::Subscription _global_pos_sub{ORB_ID(vehicle_global_position)};		/**< global position subscription */
	uORB::Subscription _manual_sub{ORB_ID(manual_control_setpoint)};		/**< notification of manual control updates */
	uORB::Subscription _parameter_update_sub{ORB_ID(parameter_update)};		/**< notification of parameter updates */
	uORB::Subscription _rates_sp_sub{ORB_ID(vehicle_rates_setpoint)};		/**< vehicle rates setpoint */
	uORB::Subscription _vcontrol_mode_sub{ORB_ID(vehicle_control_mode)};		/**< vehicle status subscription */
	uORB::Subscription _vehicle_status_sub{ORB_ID(vehicle_status)};			/**< vehicle status subscription */
	uORB::Subscription _vehicle_rates_sub{ORB_ID(vehicle_angular_velocity)};

	uORB::Publication<actuator_controls_s>		_actuators_0_pub{ORB_ID(actuator_controls_0)};
	uORB::Publication<vehicle_attitude_setpoint_s>	_attitude_sp_pub{ORB_ID(vehicle_attitude_setpoint)};
	uORB::Publication<vehicle_rates_setpoint_s>	_rate_sp_pub{ORB_ID(vehicle_rates_setpoint)};
	uORB::PublicationMulti<rate_ctrl_status_s>	_rate_ctrl_status_pub{ORB_ID(rate_ctrl_status)};

	actuator_controls_s			_actuators {};		/**< actuator control inputs */
	manual_control_setpoint_s		_manual {};		/**< r/c channel data */
	vehicle_attitude_s			_att {};		/**< vehicle attitude setpoint */
	vehicle_attitude_setpoint_s		_att_sp {};		/**< vehicle attitude setpoint */
	vehicle_control_mode_s			_vcontrol_mode {};	/**< vehicle control mode */
	vehicle_global_position_s		_global_pos {};		/**< global position */
	vehicle_rates_setpoint_s		_rates_sp {};		/* attitude rates setpoint */
	vehicle_status_s			_vehicle_status {};	/**< vehicle status */

	perf_counter_t	_loop_perf;			/**< loop performance counter */

	float _flaps_applied{0.0f};
	float _flaperons_applied{0.0f};

	float _airspeed_scaling{1.0f};

	bool _landed{true};

	float _battery_scale{1.0f};

	bool _flag_control_attitude_enabled_last{false};

	bool _is_tailsitter{false};

	DEFINE_PARAMETERS(
		(ParamFloat<px4::params::GB_MAN_P_SC>) _param_gb_man_p_sc,
		(ParamFloat<px4::params::GB_MAN_R_SC>) _param_gb_man_r_sc,
		(ParamFloat<px4::params::GB_MAN_Y_SC>) _param_gb_man_y_sc,

		(ParamFloat<px4::params::TRIM_PITCH>) _param_trim_pitch,
		(ParamFloat<px4::params::TRIM_ROLL>) _param_trim_roll,
		(ParamFloat<px4::params::TRIM_YAW>) _param_trim_yaw
	)

	/**
	 * Update our local parameter cache.
	 */
	int		parameters_update();

	void		vehicle_control_mode_poll();
	void		vehicle_manual_poll();
	void		vehicle_attitude_setpoint_poll();
	void		vehicle_rates_setpoint_poll();
};
