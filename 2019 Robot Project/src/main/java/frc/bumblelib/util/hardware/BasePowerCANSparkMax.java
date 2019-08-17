/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.bumblelib.util.hardware;

import com.revrobotics.CANSparkMax;

import frc.bumblelib.util.BasePowerCalculator;

/**
 * Add your docs here.
 */
public class BasePowerCANSparkMax extends CANSparkMax {

	private double frictionOvercomePower;
	private double gravityCompensationPower;
	private double scalingFactor = 1.0;

	public BasePowerCANSparkMax(int deviceID, MotorType type, double frictionOvercomePower) {
		super(deviceID, type);
		this.frictionOvercomePower = frictionOvercomePower;
		this.gravityCompensationPower = 0.0;
	}

	public BasePowerCANSparkMax(int deviceID, MotorType type) {
		super(deviceID, type);
		this.frictionOvercomePower = 0.0;
		this.gravityCompensationPower = 0.0;
	}

	public void setGravityCompensationPower(double gravityCompensationPower) {
		this.gravityCompensationPower = gravityCompensationPower;
	}

	public void setFrictionOvercomePower(double frictionOvercomePower) {
		this.frictionOvercomePower = frictionOvercomePower;
	}

	/**
	 * @return the frictionOvercomePower
	 */
	public double getFrictionOvercomePower() {
		return frictionOvercomePower;
	}

	@Override
	public void pidWrite(double output) {
		set(BasePowerCalculator.calculateModifiedOutput(output, gravityCompensationPower, frictionOvercomePower));
	}

	@Override
	public void set(double speed) {
		super.set(speed * scalingFactor);
	}

	/**
	 * @param scalingFactor the scalingFactor to set
	 */
	public void setScalingFactor(double scalingFactor) {
		this.scalingFactor = scalingFactor;
	}
}
