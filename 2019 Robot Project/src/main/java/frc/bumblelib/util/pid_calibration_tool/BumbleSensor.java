package frc.bumblelib.util.pid_calibration_tool;

import edu.wpi.first.wpilibj.PIDSource;
import edu.wpi.first.wpilibj.PIDSourceType;

public class BumbleSensor implements PIDSource {

	private PIDSourceType type;
	private double output;

	public BumbleSensor(PIDSourceType type) {
		this.type = type;
	}

	@Override
	public void setPIDSourceType(PIDSourceType pidSource) {
		type = pidSource;
	}

	@Override
	public PIDSourceType getPIDSourceType() {
		return type;
	}

	@Override
	public double pidGet() {
		return output;
	}

	public void setSensorOutput(double output) {
		this.output = output;
	}
}
