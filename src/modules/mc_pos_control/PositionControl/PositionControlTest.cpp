/****************************************************************************
 *
 *   Copyright (C) 2019 PX4 Development Team. All rights reserved.
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

#include <gtest/gtest.h>
#include <PositionControl.hpp>

using namespace matrix;

class PositionControlBasicTest : public ::testing::Test
{
public:
	PositionControlBasicTest()
	{
		_position_control.setPositionGains(Vector3f(1, 1, 1));
		_position_control.setVelocityGains(Vector3f(1, 1, 1), Vector3f(1, 1, 1), Vector3f(1, 1, 1));
		_position_control.setVelocityLimits(1, 1, 1);
		_position_control.setTiltLimit(1);
		_position_control.setHoverThrust(.5f);

		_contraints.tilt = 1.f;

		_input_setpoint.x = NAN;
		_input_setpoint.y = NAN;
		_input_setpoint.z = NAN;
		_input_setpoint.yaw = NAN;
		_input_setpoint.yawspeed = NAN;
		_input_setpoint.vx = NAN;
		_input_setpoint.vy = NAN;
		_input_setpoint.vz = NAN;
		Vector3f(NAN, NAN, NAN).copyTo(_input_setpoint.acceleration);
		Vector3f(NAN, NAN, NAN).copyTo(_input_setpoint.thrust);
	}

	void runController()
	{
		_position_control.setConstraints(_contraints);
		_position_control.setInputSetpoint(_input_setpoint);
		_position_control.update(0.1f);
		_position_control.getLocalPositionSetpoint(_output_setpoint);
		_position_control.getAttitudeSetpoint(_attitude);
	}

	PositionControl _position_control;
	vehicle_constraints_s _contraints{};
	vehicle_local_position_setpoint_s _input_setpoint{};
	vehicle_local_position_setpoint_s _output_setpoint{};
	vehicle_attitude_setpoint_s _attitude{};
};

TEST(PositionControlTest, EmptySetpoint)
{
	PositionControl position_control;

	vehicle_local_position_setpoint_s output_setpoint{};
	position_control.getLocalPositionSetpoint(output_setpoint);
	EXPECT_EQ(output_setpoint.x, 0);
	EXPECT_EQ(output_setpoint.y, 0);
	EXPECT_EQ(output_setpoint.z, 0);
	EXPECT_EQ(output_setpoint.yaw, 0);
	EXPECT_EQ(output_setpoint.yawspeed, 0);
	EXPECT_EQ(output_setpoint.vx, 0);
	EXPECT_EQ(output_setpoint.vy, 0);
	EXPECT_EQ(output_setpoint.vz, 0);
	EXPECT_EQ(Vector3f(output_setpoint.acceleration), Vector3f(0, 0, 0));
	EXPECT_EQ(Vector3f(output_setpoint.jerk), Vector3f(0, 0, 0));
	EXPECT_EQ(Vector3f(output_setpoint.thrust), Vector3f(0, 0, 0));

	vehicle_attitude_setpoint_s attitude{};
	position_control.getAttitudeSetpoint(attitude);
	EXPECT_EQ(attitude.roll_body, 0);
	EXPECT_EQ(attitude.pitch_body, 0);
	EXPECT_EQ(attitude.yaw_body, 0);
	EXPECT_EQ(attitude.yaw_sp_move_rate, 0);
	EXPECT_EQ(Quatf(attitude.q_d), Quatf(1, 0, 0, 0));
	//EXPECT_EQ(attitude.q_d_valid, false); // TODO should not be true when there was no control
	EXPECT_EQ(Vector3f(attitude.thrust_body), Vector3f(0, 0, 0));
	EXPECT_EQ(attitude.roll_reset_integral, false);
	EXPECT_EQ(attitude.pitch_reset_integral, false);
	EXPECT_EQ(attitude.yaw_reset_integral, false);
	EXPECT_EQ(attitude.fw_control_yaw, false);
	EXPECT_EQ(attitude.apply_flaps, 0);//vehicle_attitude_setpoint_s::FLAPS_OFF); // TODO why no reference?
}

class PositionControlBasicDirectionTest : public PositionControlBasicTest
{
public:
	void checkDirection()
	{
		Vector3f acceleration(_output_setpoint.acceleration);
		EXPECT_GT(acceleration(0), 0);
		EXPECT_GT(acceleration(1), 0);
		EXPECT_LT(acceleration(2), 0);

		Vector3f body_z = Quatf(_attitude.q_d).dcm_z();
		EXPECT_LT(body_z(0), 0);
		EXPECT_LT(body_z(1), 0);
		EXPECT_GT(body_z(2), 0);
	}
};

TEST_F(PositionControlBasicDirectionTest, PositionControlDirectionP)
{
	_input_setpoint.x = 1;
	_input_setpoint.y = 1;
	_input_setpoint.z = -1;
	runController();
	checkDirection();
}

TEST_F(PositionControlBasicDirectionTest, VelocityControlDirection)
{
	_input_setpoint.vx = 1;
	_input_setpoint.vy = 1;
	_input_setpoint.vz = -1;
	runController();
	checkDirection();
}

TEST_F(PositionControlBasicDirectionTest, AccelerationControlDirection)
{
	Vector3f(1, 1, -1).copyTo(_input_setpoint.acceleration);
	runController();
	checkDirection();
}
