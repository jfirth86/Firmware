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

#include "SymmetricGliderAttitudeControl.hpp"

#include <vtol_att_control/vtol_type.h>

using namespace time_literals;
using math::constrain;
using math::gradual;
using math::radians;

SymmetricGliderAttitudeControl::SymmetricGliderAttitudeControl() :
	ModuleParams(nullptr),
	WorkItem(MODULE_NAME, px4::wq_configurations::att_pos_ctrl),
	_loop_perf(perf_alloc(PC_ELAPSED, MODULE_NAME": cycle"))
{
	/* fetch initial parameter values */
	parameters_update();
}

SymmetricGliderAttitudeControl::~SymmetricGliderAttitudeControl()
{
	perf_free(_loop_perf);
}

bool
SymmetricGliderAttitudeControl::init()
{
	if (!_att_sub.registerCallback()) {
		PX4_ERR("vehicle attitude callback registration failed!");
		return false;
	}

	return true;
}

int
SymmetricGliderAttitudeControl::parameters_update()
{
	return PX4_OK;
}

void
SymmetricGliderAttitudeControl::vehicle_control_mode_poll()
{
	_vcontrol_mode_sub.update(&_vcontrol_mode);
}

void
SymmetricGliderAttitudeControl::vehicle_manual_poll()
{
	if (_vcontrol_mode.flag_control_manual_enabled) {

		// Always copy the new manual setpoint, even if it wasn't updated, to fill the _actuators with valid values
		if (_manual_sub.copy(&_manual)) {

			/* manual/direct control */
			_actuators.control[actuator_controls_s::INDEX_ROLL] = _manual.y * _param_gb_man_r_sc.get() + _param_trim_roll.get();
			_actuators.control[actuator_controls_s::INDEX_PITCH] = -_manual.x * _param_gb_man_p_sc.get() + _param_trim_pitch.get();
			_actuators.control[actuator_controls_s::INDEX_YAW] = _manual.r * _param_gb_man_y_sc.get() + _param_trim_yaw.get();
		}
	}
}

void
SymmetricGliderAttitudeControl::vehicle_attitude_setpoint_poll()
{
	if (_att_sp_sub.update(&_att_sp)) {
		_rates_sp.thrust_body[0] = _att_sp.thrust_body[0];
		_rates_sp.thrust_body[1] = _att_sp.thrust_body[1];
		_rates_sp.thrust_body[2] = _att_sp.thrust_body[2];
	}
}

void
SymmetricGliderAttitudeControl::vehicle_rates_setpoint_poll()
{
	if (_rates_sp_sub.update(&_rates_sp)) {
	}
}

void SymmetricGliderAttitudeControl::Run()
{
	if (should_exit()) {
		_att_sub.unregisterCallback();
		exit_and_cleanup();
		return;
	}

	perf_begin(_loop_perf);

	if (_att_sub.update(&_att)) {

		// only update parameters if they changed
		bool params_updated = _parameter_update_sub.updated();

		// check for parameter updates
		if (params_updated) {
			// clear update
			parameter_update_s pupdate;
			_parameter_update_sub.copy(&pupdate);

			// update parameters from storage
			updateParams();
			parameters_update();
		}

		/* only run controller if attitude changed */
		static uint64_t last_run = 0;
		static uint64_t run_count = 0;

		float deltaT = constrain((hrt_elapsed_time(&last_run) / 1e6f), 0.01f, 0.1f);
		last_run = hrt_absolute_time();

		float t = (float)run_count * deltaT;
		float cmd[3] = {0.0f};
		run_count++;

        static matrix::Quaternionf q_init;

		/* for the first second, command the current attitude */
		if (t < 1.0f)
		{
			q_init = matrix::Quaternionf(_att.q);
		}

		/* get current rotation matrix and euler angles from control state quaternions */
		matrix::Dcmf R = matrix::Quatf(_att.q);

		vehicle_angular_velocity_s angular_velocity{};
		_vehicle_rates_sub.copy(&angular_velocity);
		//float rollspeed = angular_velocity.xyz[0];
		//float pitchspeed = angular_velocity.xyz[1];
		//float yawspeed = angular_velocity.xyz[2];

		const matrix::Eulerf euler_angles(R);

		vehicle_attitude_setpoint_poll();

		// vehicle status update must be before the vehicle_control_mode_poll(), otherwise rate sp are not published during whole transition
		_vehicle_status_sub.update(&_vehicle_status);

		vehicle_control_mode_poll();
		vehicle_manual_poll();
		_global_pos_sub.update(&_global_pos);


		/* calculate the error quaternion */
		matrix::Quaternionf qCmdToBody = q_init * matrix::Quaternionf(_att.q).inversed();

		cmd[0] += 0.0f * qCmdToBody(1) * _param_gb_gain_err_r.get();
		cmd[1] += 0.0f * qCmdToBody(2) * _param_gb_gain_err_p.get();
		cmd[2] += 0.0f * qCmdToBody(3) * _param_gb_gain_err_y.get();

		cmd[0] += _att.q[1] * _param_gb_gain_err_r.get();
		cmd[1] += _att.q[2] * _param_gb_gain_err_p.get();
		cmd[2] += _att.q[3] * _param_gb_gain_err_y.get();

		cmd[0] -= angular_velocity.xyz[0] * _param_gb_gain_rate_r.get();
		cmd[1] -= angular_velocity.xyz[1] * _param_gb_gain_rate_p.get();
		cmd[2] -= angular_velocity.xyz[2] * _param_gb_gain_rate_y.get();


		_actuators.control[actuator_controls_s::INDEX_ROLL] = cmd[0];

		_actuators.control[actuator_controls_s::INDEX_PITCH] = cmd[1];

		_actuators.control[actuator_controls_s::INDEX_YAW] = cmd[2];

		_actuators.control[actuator_controls_s::INDEX_THROTTLE] = 0.0f;

		/* lazily publish the setpoint only once available */
		_actuators.timestamp = hrt_absolute_time();
		_actuators.timestamp_sample = _att.timestamp;

		_actuators_0_pub.publish(_actuators);
	}


	perf_end(_loop_perf);
}


int SymmetricGliderAttitudeControl::task_spawn(int argc, char *argv[])
{
	SymmetricGliderAttitudeControl *instance = new SymmetricGliderAttitudeControl();

	if (instance) {
		_object.store(instance);
		_task_id = task_id_is_work_queue;

		if (instance->init()) {
			return PX4_OK;
		}

	} else {
		PX4_ERR("alloc failed");
	}

	delete instance;
	_object.store(nullptr);
	_task_id = -1;

	return PX4_ERROR;
}

int SymmetricGliderAttitudeControl::custom_command(int argc, char *argv[])
{
	return print_usage("unknown command");
}

int SymmetricGliderAttitudeControl::print_usage(const char *reason)
{
	if (reason) {
		PX4_WARN("%s\n", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description
gb_att_control is the symmetric glider attitude controller.

)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("gb_att_control", "controller");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

	return 0;
}

extern "C" __EXPORT int gb_att_control_main(int argc, char *argv[])
{
	return SymmetricGliderAttitudeControl::main(argc, argv);
}
