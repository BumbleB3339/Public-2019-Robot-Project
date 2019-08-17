package frc.bumblelib.util.hardware;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.PIDOutput;
import frc.bumblelib.util.BasePowerCalculator;

public class BasePowerWPI_TalonSRX extends WPI_TalonSRX implements PIDOutput {
	private double frictionOvercomePower;
	private double gravityCompensationPower;
	private double scalingFactor = 1.0;

	public BasePowerWPI_TalonSRX(int deviceNumber, double frictionOvercomePower) {
		super(deviceNumber);
		this.frictionOvercomePower = frictionOvercomePower;
		this.gravityCompensationPower = 0.0;
	}

	public BasePowerWPI_TalonSRX(int deviceNumber) {
		super(deviceNumber);
		this.frictionOvercomePower = 0.0;
		this.gravityCompensationPower = 0.0;
	}

	public void setGravityCompensationPower(double gravityCompensationPower) {
		this.gravityCompensationPower = gravityCompensationPower;
	}

	@Override
	public void pidWrite(double output) {
		output = BasePowerCalculator.calculateModifiedOutput(output, gravityCompensationPower, frictionOvercomePower);
		set(output);
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
