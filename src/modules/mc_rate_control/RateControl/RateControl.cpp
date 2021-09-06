/****************************************************************************
 *
 *   Copyright (c) 2019 PX4 Development Team. All rights reserved.
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

/**
 * @file RateControl.cpp
 */

#include <RateControl.hpp>
#include <px4_platform_common/defines.h>

using namespace matrix;

void RateControl::setGains(const Vector3f &P, const Vector3f &I, const Vector3f &D)
{
	_gain_p = P;
	_gain_i = I;
	_gain_d = D;
}

void RateControl::setSaturationStatus(const MultirotorMixer::saturation_status &status)
{
	_mixer_saturation_positive[0] = status.flags.roll_pos;
	_mixer_saturation_positive[1] = status.flags.pitch_pos;
	_mixer_saturation_positive[2] = status.flags.yaw_pos;
	_mixer_saturation_negative[0] = status.flags.roll_neg;
	_mixer_saturation_negative[1] = status.flags.pitch_neg;
	_mixer_saturation_negative[2] = status.flags.yaw_neg;
}

Vector3f RateControl::update(const Vector3f &rate, const Vector3f &rate_sp, const Vector3f &angular_accel,
			     const float dt, const bool landed)
{
	// ADDED - JH
	// const Vector3f torque = rate_sp;

	// return torque;

	// angular rates error
	Vector3f rate_error = rate_sp - rate;

	// PID control with feed forward
	const Vector3f torque = _gain_p.emult(rate_error) + _rate_int - _gain_d.emult(angular_accel) + _gain_ff.emult(rate_sp);

	// update integral only if we are not landed
	if (!landed) {
		updateIntegral(rate_error, dt);
	}

	return torque;
}
// JH - ADDED
// modified DOB version rate_controller
// Vector3f RateControl::update_JH(const Vector3f &angle_euler, const Vector3f &rate, const Vector3f &rate_sp, const Vector3f &angular_accel,
// 			     const float dt, const bool landed)
// {
// 	// angular rates error
// 	Vector3f rate_error = rate_sp - rate;

// 	PX4_INFO("Firmware changed! - version 1");

// 	// PID control with feed forward
// 	// outer loop control input
// 	Vector3f tau_rpy = _gain_p.emult(rate_error) + _rate_int - _gain_d.emult(angular_accel) + _gain_ff.emult(rate_sp);

// 	// JH
// 	// DOB code
// 	// MOI
// 	// parameter
// 	float I_m1 = 0.01; float I_m2 = 0.01; float I_m3 = 0.01;
// 	float a0_ = 1.0; float a1_ = 2.0; float eps_ = 0.1;

// 	Vector3f I_m_vec(I_m1,I_m2,I_m3);
// 	Matrix3f I_m;
// 	I_m(0,0) = I_m1; I_m(1,1) = I_m2; I_m(2,2) = I_m3;
// 	Matrix3f I_m_inv;
// 	I_m_inv(0,0) = (float)1.0/I_m1; I_m_inv(1,1) = (float)1.0/I_m2; I_m_inv(2,2) = (float)1.0/I_m3;
// 	float Ibar_m = I_m_vec.min();

// 	Matrix3f Lambda;
// 	Lambda(0,0) = Ibar_m; Lambda(1,1) = Ibar_m; Lambda(2,2) = Ibar_m;

// 	Matrix3f Lambda_inv;
// 	Lambda_inv(0,0) = (float)1.0/Ibar_m; Lambda_inv(1,1) = (float)1.0/Ibar_m; Lambda_inv(2,2) = (float)1.0/Ibar_m;

// 	// unit: [rad]
// 	float phi_ = angle_euler(0);
// 	float theta_ = angle_euler(1);
// 	float psi_ = angle_euler(2);

// 	Matrix3f Q_;
// 	Q_(0,0) = 1.0; Q_(0,1) = 0.0; Q_(0,2) = -sin(theta_);
// 	Q_(1,0) = 0.0; Q_(1,1) = cos(phi_); Q_(1,2) = sin(phi_)*cos(theta_);
// 	Q_(2,0) = 0.0; Q_(2,1) = -sin(phi_); Q_(2,2) = cos(phi_)*cos(theta_);

// 	Matrix3f Q_inv_;
// 	Q_inv_(0,0) = 1.0; Q_inv_(0,1) = (sin(phi_)*sin(theta_))/cos(theta_); 	Q_inv_(0,2) = (cos(phi_)*sin(theta_))/cos(theta_);
// 	Q_inv_(1,0) = 0.0; Q_inv_(1,1) = cos(phi_); 				Q_inv_(1,2) = -sin(phi_);
// 	Q_inv_(2,0) = 0.0; Q_inv_(2,1) = sin(phi_)/cos(theta_); 		Q_inv_(2,2) = cos(phi_)/cos(theta_);

// 	// body angular rate
// 	Vector3f om_ = rate;
// 	Vector3f Phidot_ = Q_inv_*om_;

// 	float phidot_ = Phidot_(0);
// 	float thetadot_ = Phidot_(1);
// 	// float psidot_ = Phidot_(2);

// 	Matrix3f Qdot_;
// 	Qdot_(0,0) = 0.0; Qdot_(0,1) = 0.0; 			Qdot_(0,2) = -thetadot_*cos(theta_);
// 	Qdot_(1,0) = 0.0; Qdot_(1,1) = -phidot_*sin(phi_); 	Qdot_(1,2) = phidot_*cos(phi_)*cos(theta_) - thetadot_*sin(phi_)*sin(theta_);
// 	Qdot_(2,0) = 0.0; Qdot_(2,1) = -phidot_*cos(phi_); 	Qdot_(2,2) = -phidot_*sin(phi_)*cos(theta_) - thetadot_*cos(phi_)*sin(theta_);

// 	Matrix3f GBar_PHI = Q_inv_*Lambda_inv;
// 	Vector3f FBar_PHI = Q_inv_*Qdot_*Phidot_*(-1.0) - Q_inv_*I_m_inv*om_.hat()*I_m*om_;

// 	matrix::Matrix<float,2,2> A_;
// 	Vector2f B_;
// 	A_(0,1) = 1.0;
// 	A_(1,0) = -a0_/(eps_*eps_);
// 	A_(1,1) = -a1_/eps_;
// 	B_(1) = a0_/(eps_*eps_);

// 	Vector2f dqphi_FF = A_*qphi_FF + B_*phi_;
// 	Vector2f dqtheta_FF = A_*qtheta_FF + B_*theta_;
// 	Vector2f dqpsi_FF = A_*qpsi_FF + B_*psi_;

// 	float uphi_FF 	= math::constrain<float>(pphi_FF(0) - Lambda(0,0)*(dqphi_FF(1) - FBar_PHI(0)),(float)-100.0,(float)100.0);
// 	float utheta_FF 	= math::constrain<float>(ptheta_FF(0) - Lambda(1,1)*(dqtheta_FF(1) - FBar_PHI(1)),(float)-100.0,(float)100.0);
// 	float upsi_FF 	= math::constrain<float>(ppsi_FF(0) - Lambda(2,2)*(dqpsi_FF(1) - FBar_PHI(2)),(float)-100.0,(float)100.0);

// 	Vector3f uPhi_FF(uphi_FF,utheta_FF,upsi_FF);
// 	Vector3f tau2_PHI = Lambda*GBar_PHI*tau_rpy + uPhi_FF;
// 	// inv(GBar_PHI) = Lambda_*Q_
// 	// inv(Lambda*GBar_PHI) = inv(GBar_PHI)*Lambda_inv = Lambda_*Q_*Lambda_inv
// 	Vector3f tau_PHI = tau_rpy + Lambda*Q_*Lambda_inv*uPhi_FF;

// 	Vector2f dpphi_FF = A_*pphi_FF + B_*tau2_PHI(0);
// 	Vector2f dptheta_FF = A_*ptheta_FF + B_*tau2_PHI(1);
// 	Vector2f dppsi_FF = A_*ppsi_FF + B_*tau2_PHI(2);

// 	// PQ filter update
// 	pphi_FF += dpphi_FF*dt;
// 	ptheta_FF += dptheta_FF*dt;
// 	ppsi_FF += dppsi_FF*dt;
// 	qphi_FF += dqphi_FF*dt;
// 	qtheta_FF += dqtheta_FF*dt;
// 	qpsi_FF += dqpsi_FF*dt;

// 	printf("torque(0): %5.3f \n",(double)tau_PHI(0));
// 	printf("torque(1): %5.3f \n",(double)tau_PHI(1));
// 	printf("torque(2): %5.3f \n",(double)tau_PHI(2));


// 	// update integral only if we are not landed
// 	if (!landed) {
// 		updateIntegral(rate_error, dt);
// 	}

// 	return tau_PHI;
// }

// JH - ADDED
// modified DOB version 2 rate_controller
Vector3f RateControl::update_JH2(const Vector3f &angle_euler, const Vector3f &rate, const Vector3f &rate_sp, const Vector3f &angular_accel,
			     const float dt, const bool landed, const int mode_, const float thrust_)
{
	// angular rates error
	Vector3f rate_error = rate_sp - rate;

	PX4_INFO("Firmware changed! - version 2");

	// JH
	// parameters
	float g_ = 9.81; // gravitational acceleration
	float m_B_ = 2.7; // mass
	float l_1 = 0.35; // length of the link
	float I_m1 = 0.01; float I_m2 = 0.01; float I_m3 = 0.01; // MOI
	// DOB parameters
	float a0_ = 1.0; float a1_ = 2.0; float eps_ = 0.1; float gma = 0.01;

	Vector3f I_m_vec(I_m1,I_m2,I_m3);
	Matrix3f I_m;
	I_m(0,0) = I_m1; I_m(1,1) = I_m2; I_m(2,2) = I_m3;
	Matrix3f I_m_inv;
	I_m_inv(0,0) = (float)1.0/I_m1; I_m_inv(1,1) = (float)1.0/I_m2; I_m_inv(2,2) = (float)1.0/I_m3;
	float Ibar_m = I_m_vec.min();

	// unit: [rad]
	float phi_ = angle_euler(0);
	float theta_ = angle_euler(1);
	float psi_ = angle_euler(2);

	Matrix3f Q_;
	Q_(0,0) = 1.0; Q_(0,1) = 0.0; Q_(0,2) = -sin(theta_);
	Q_(1,0) = 0.0; Q_(1,1) = cos(phi_); Q_(1,2) = sin(phi_)*cos(theta_);
	Q_(2,0) = 0.0; Q_(2,1) = -sin(phi_); Q_(2,2) = cos(phi_)*cos(theta_);

	Matrix3f Q_inv_;
	Q_inv_(0,0) = 1.0; Q_inv_(0,1) = (sin(phi_)*sin(theta_))/cos(theta_); 	Q_inv_(0,2) = (cos(phi_)*sin(theta_))/cos(theta_);
	Q_inv_(1,0) = 0.0; Q_inv_(1,1) = cos(phi_); 				Q_inv_(1,2) = -sin(phi_);
	Q_inv_(2,0) = 0.0; Q_inv_(2,1) = sin(phi_)/cos(theta_); 		Q_inv_(2,2) = cos(phi_)/cos(theta_);

	// body angular rate
	Vector3f om_ = rate;
	Vector3f Phidot_ = Q_inv_*om_;

	float phidot_ = Phidot_(0);
	float thetadot_ = Phidot_(1);
	// float psidot_ = Phidot_(2);

	Matrix3f Qdot_;
	Qdot_(0,0) = 0.0; Qdot_(0,1) = 0.0; 			Qdot_(0,2) = -thetadot_*cos(theta_);
	Qdot_(1,0) = 0.0; Qdot_(1,1) = -phidot_*sin(phi_); 	Qdot_(1,2) = phidot_*cos(phi_)*cos(theta_) - thetadot_*sin(phi_)*sin(theta_);
	Qdot_(2,0) = 0.0; Qdot_(2,1) = -phidot_*cos(phi_); 	Qdot_(2,2) = -phidot_*sin(phi_)*cos(theta_) - thetadot_*cos(phi_)*sin(theta_);

	// rotation matrix
	Matrix3f Rz_;
	Rz_(0,0) = cos(psi_); Rz_(0,1) = -sin(psi_); Rz_(0,2) = 0.0;
	Rz_(1,0) = sin(psi_); Rz_(1,1) = cos(psi_); Rz_(1,2) = 0.0;
	Rz_(2,0) = 0.0; Rz_(2,1) = 0.0; Rz_(2,2) = 1.0;

	Matrix3f Ry_;
	Ry_(0,0) = cos(theta_); Ry_(0,1) = 0.0; Ry_(0,2) = sin(theta_);
	Ry_(1,0) = 0.0; Ry_(1,1) = 1.0; Ry_(1,2) = 0.0;
	Ry_(2,0) = -sin(theta_); Ry_(2,1) = 0.0; Ry_(2,2) = cos(theta_);

	Matrix3f Rx_;
	Rx_(0,0) = 1.0; Ry_(0,1) = 0.0; Ry_(0,2) = 0.0;
	Rx_(1,0) = 0.0; Ry_(1,1) = cos(phi_); Ry_(1,2) = -sin(phi_);
	Rx_(2,0) = 0.0; Ry_(2,1) = sin(phi_); Ry_(2,2) = cos(phi_);

	Matrix3f R_;
	R_ = Rz_ * Ry_* Rx_;

	// z-axis extractor
	Vector3f e_3;
	e_3(0) = 0.0; e_3(1) = 0.0; e_3(2) = 1.0;

	// body to end-effector position
	Vector3f p_BE_;
	p_BE_(0) = (float)1.7321 * l_1 / (float)2.0;
	p_BE_(1) = 0.0;
	p_BE_(2) = -(float)0.09 - l_1 / (float)2.0;
	Vector3f pI_BE_ = R_ * p_BE_;
	Matrix3f J_t;
	J_t = I_m - m_B_ * pI_BE_.hat() * pI_BE_.hat();

	Matrix3f GBar_PHI_FF = Q_inv_ / Ibar_m;
	Vector3f FBar_PHI_FF = Q_inv_*Qdot_*Phidot_*(-1.0) - Q_inv_*I_m_inv*om_.hat()*I_m*om_;

	Matrix3f GBar_PHI_PP = Q_inv_ / Ibar_m;
	Vector3f temp = om_.hat() * pI_BE_;
	Vector3f FBar_PHI_PP = - Q_inv_ * geninv(J_t) * ((-m_B_*pI_BE_.hat()*(temp.hat() * Q_ + pI_BE_.hat() * Qdot_)+I_m*Qdot_*om_.hat()*I_m*Q_)*Phidot_ - m_B_ * g_ * pI_BE_.hat() * e_3 - thrust_ * Q_.transpose() * pI_BE_.hat() * R_ * e_3);

	matrix::Matrix<float,2,2> A_;
	Vector2f B_;
	A_(0,1) = 1.0;
	A_(1,0) = -a0_/(eps_*eps_);
	A_(1,1) = -a1_/eps_;
	B_(1) = a0_/(eps_*eps_);

	Vector2f dqphi_FF = A_*qphi_FF + B_*phi_;
	Vector2f dqtheta_FF = A_*qtheta_FF + B_*theta_;
	Vector2f dqpsi_FF = A_*qpsi_FF + B_*psi_;
	Vector2f dqphi_PP = A_*qphi_PP + B_*phi_;
	Vector2f dqtheta_PP = A_*qtheta_PP + B_*theta_;
	Vector2f dqpsi_PP 	= A_*qpsi_PP + B_*psi_;

	float uphi_FF 	= math::constrain<float>(pphi_FF(0) - Ibar_m*(dqphi_FF(1) - FBar_PHI_FF(0)),(float)-100.0,(float)100.0);
	float utheta_FF 	= math::constrain<float>(ptheta_FF(0) - Ibar_m*(dqtheta_FF(1) - FBar_PHI_FF(1)),(float)-100.0,(float)100.0);
	float upsi_FF 	= math::constrain<float>(ppsi_FF(0) - Ibar_m*(dqpsi_FF(1) - FBar_PHI_FF(2)),(float)-100.0,(float)100.0);
	float uphi_PP 	= math::constrain<float>(pphi_PP(0) - Ibar_m*(dqphi_PP(1) - FBar_PHI_PP(0)),(float)-100.0,(float)100.0);
	float utheta_PP 	= math::constrain<float>(ptheta_PP(0) - Ibar_m*(dqtheta_PP(1) - FBar_PHI_PP(1)),(float)-100.0,(float)100.0);
	float upsi_PP 	= math::constrain<float>(ppsi_PP(0) - Ibar_m*(dqpsi_PP(1) - FBar_PHI_PP(2)),(float)-100.0,(float)100.0);

	Vector3f uPhi_FF(uphi_FF,utheta_FF,upsi_FF);
	Vector3f uPhi_PP(uphi_PP,utheta_PP,upsi_PP);

	// PID control with feed forward
	// outer loop control input
	Vector3f tau_rpy_FF;
	Vector3f tau_rpy_PP;
	if (mode_ == 1)
	{
		tau_rpy_FF = _gain_p.emult(rate_error) + _rate_int - _gain_d.emult(angular_accel) + _gain_ff.emult(rate_sp);
		tau_rpy_PP = Ibar_m * Q_ * (- FBar_PHI_PP - gma * uPhi_PP);
	}
	else if (mode_ == 2)
	{
		tau_rpy_FF = Ibar_m * Q_ * (- FBar_PHI_FF - gma * uPhi_FF);
		tau_rpy_PP = _gain_p.emult(rate_error) + _rate_int - _gain_d.emult(angular_accel) + _gain_ff.emult(rate_sp);
	}
	else
		PX4_WARN("MODE SELECTION ERROR!");

	Vector3f tau2_PHI_FF = Ibar_m*GBar_PHI_FF*tau_rpy_FF + uPhi_FF;
	Vector3f tau2_PHI_PP = Ibar_m*GBar_PHI_PP*tau_rpy_PP + uPhi_PP;
	// inv(GBar_PHI) = Lambda_*Q_
	// inv(Lambda*GBar_PHI) = inv(GBar_PHI)*Lambda_inv = Lambda_*Q_*Lambda_inv
	Vector3f tau_PHI;
	if (mode_ == 1)
		tau_PHI = tau_rpy_FF + Q_*uPhi_FF;
	else if (mode_ == 2)
		tau_PHI = tau_rpy_PP + Q_*uPhi_PP;
	else
		PX4_WARN("MODE SELECTION ERROR!");

	Vector2f dpphi_FF = A_*pphi_FF + B_*tau2_PHI_FF(0);
	Vector2f dptheta_FF = A_*ptheta_FF + B_*tau2_PHI_FF(1);
	Vector2f dppsi_FF = A_*ppsi_FF + B_*tau2_PHI_FF(2);
	Vector2f dpphi_PP = A_*pphi_PP + B_*tau2_PHI_PP(0);
	Vector2f dptheta_PP = A_*ptheta_PP + B_*tau2_PHI_PP(1);
	Vector2f dppsi_PP = A_*ppsi_PP + B_*tau2_PHI_PP(2);

	// PQ filter update
	pphi_FF += dpphi_FF*dt;
	ptheta_FF += dptheta_FF*dt;
	ppsi_FF += dppsi_FF*dt;
	qphi_FF += dqphi_FF*dt;
	qtheta_FF += dqtheta_FF*dt;
	qpsi_FF += dqpsi_FF*dt;
	pphi_PP += dpphi_PP*dt;
	ptheta_PP += dptheta_PP*dt;
	ppsi_PP += dppsi_PP*dt;
	qphi_PP += dqphi_PP*dt;
	qtheta_PP += dqtheta_PP*dt;
	qpsi_PP += dqpsi_PP*dt;

	printf("torque(0): %5.3f \n",(double)tau_PHI(0));
	printf("torque(1): %5.3f \n",(double)tau_PHI(1));
	printf("torque(2): %5.3f \n",(double)tau_PHI(2));


	// update integral only if we are not landed
	if (!landed) {
		updateIntegral(rate_error, dt);
	}

	return tau_PHI;
}

void RateControl::updateIntegral(Vector3f &rate_error, const float dt)
{
	for (int i = 0; i < 3; i++) {
		// prevent further positive control saturation
		if (_mixer_saturation_positive[i]) {
			rate_error(i) = math::min(rate_error(i), 0.f);
		}

		// prevent further negative control saturation
		if (_mixer_saturation_negative[i]) {
			rate_error(i) = math::max(rate_error(i), 0.f);
		}

		// I term factor: reduce the I gain with increasing rate error.
		// This counteracts a non-linear effect where the integral builds up quickly upon a large setpoint
		// change (noticeable in a bounce-back effect after a flip).
		// The formula leads to a gradual decrease w/o steps, while only affecting the cases where it should:
		// with the parameter set to 400 degrees, up to 100 deg rate error, i_factor is almost 1 (having no effect),
		// and up to 200 deg error leads to <25% reduction of I.
		float i_factor = rate_error(i) / math::radians(400.f);
		i_factor = math::max(0.0f, 1.f - i_factor * i_factor);

		// Perform the integration using a first order method
		float rate_i = _rate_int(i) + i_factor * _gain_i(i) * rate_error(i) * dt;

		// do not propagate the result if out of range or invalid
		if (PX4_ISFINITE(rate_i)) {
			_rate_int(i) = math::constrain(rate_i, -_lim_int(i), _lim_int(i));
		}
	}
}

void RateControl::getRateControlStatus(rate_ctrl_status_s &rate_ctrl_status)
{
	rate_ctrl_status.rollspeed_integ = _rate_int(0);
	rate_ctrl_status.pitchspeed_integ = _rate_int(1);
	rate_ctrl_status.yawspeed_integ = _rate_int(2);
}
