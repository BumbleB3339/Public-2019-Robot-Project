package frc.bumblelib.util.pid_calibration_tool;

import edu.wpi.first.wpilibj.SpeedController;

/**
 * Dummy class of SpeedController type, can be used an an argument in methods that require the use of SpeedController and then ourput can be retrieved.
 */
public class BumbleMotor implements SpeedController {

	private double output = 0.0;
	private boolean isInverted = false;;

	@Override
	public void pidWrite(double output) {
		this.output = output;
	}

	@Override
	public void set(double speed) {
		output = speed;
	}

	@Override
	public double get() {
		return isInverted ? -output : output;
	}

	@Override
	public void setInverted(boolean isInverted) {
		this.isInverted = isInverted;
	}

	@Override
	public boolean getInverted() {
		return isInverted;
	}

	@Override
	public void disable() {}

	@Override
	public void stopMotor() {
		output = 0.0;
	}
}
