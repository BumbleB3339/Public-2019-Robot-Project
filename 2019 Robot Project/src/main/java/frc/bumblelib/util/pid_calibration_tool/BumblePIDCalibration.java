package frc.bumblelib.util.pid_calibration_tool;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.PIDSource;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * A class for calibrating PID gains, works for both TalonSRX and non-TalonSRX
 * applications.
 * 
 * @author BumbleB
 *
 */
public class BumblePIDCalibration {

	private BumbleMotor output;
	private PIDSource source;

	private WPI_TalonSRX talonSRX;
	private boolean isTalon;

	private PIDController controller;

	private double p = 0.0, i = 0.0, d = 0.0, f = 0.0, target = 0.0;
	private boolean start = false, allowTargetChange = true;

	/**
	 * A constructor for PID calibration with a TalonSRX
	 * 
	 * @param source
	 *            The sensor to be used for calculating PID
	 * @param tolerance
	 *            The absolute value of tolerance in which the controller is on
	 *            target
	 * @param outputRange
	 *            The absolute value of the output allowed to be sent to the motor
	 */
	public BumblePIDCalibration(PIDSource source, double tolerance, double outputRange) {
		this.source = source;
		output = new BumbleMotor();
		isTalon = false;

		SmartDashboard.putNumber("P", 0.0);
		SmartDashboard.putNumber("I", 0.0);
		SmartDashboard.putNumber("D", 0.0);
		SmartDashboard.putNumber("PIDOutput", 0.0);
		SmartDashboard.putNumber("PIDSource", 0.0);
		SmartDashboard.putNumber("PIDTarget", 0.0);
		SmartDashboard.putBoolean("PIDStart", false);
		SmartDashboard.putBoolean("PIDOnTarget", false);

		controller = new PIDController(p, i, d, source, output);
		controller.setAbsoluteTolerance(tolerance);
		controller.setOutputRange(-outputRange, outputRange);
	}

	/**
	 * A constructor for PID calibration with a TalonSRX
	 * 
	 * @param talonSRX
	 *            The TalonSRX on which to calibrate PID
	 * @param tolerance
	 *            The absolute value of tolerance in which the controller is on
	 *            target
	 * @param outputRange
	 *            The absolute value of the output allowed to be sent to the motor
	 */
	public BumblePIDCalibration(WPI_TalonSRX talonSRX, int tolerance, double outputRange) {
		this.talonSRX = talonSRX;
		isTalon = true;

		SmartDashboard.putNumber("P", 0.0);
		SmartDashboard.putNumber("I", 0.0);
		SmartDashboard.putNumber("D", 0.0);
		SmartDashboard.putNumber("F", 0.0);
		SmartDashboard.putNumber("PIDOutput", 0.0);
		SmartDashboard.putNumber("PIDSource", 0.0);
		SmartDashboard.putNumber("PIDTarget", 0.0);
		SmartDashboard.putBoolean("PIDStart", false);
		SmartDashboard.putBoolean("PIDOnTarget", false);

		talonSRX.configPeakOutputForward(outputRange, 10);
		talonSRX.configPeakOutputReverse(-outputRange, 10);
		talonSRX.configAllowableClosedloopError(0, tolerance, 10);
	}

	/**
	 * Method for executing PID calibration. The method updates SmartDashboard
	 * values and executes closed loop position control.
	 */
	public void execute() {
		p = SmartDashboard.getNumber("P", 0.0);
		i = SmartDashboard.getNumber("I", 0.0);
		d = SmartDashboard.getNumber("D", 0.0);
		f = SmartDashboard.getNumber("F", 0.0);
		start = SmartDashboard.getBoolean("PIDStart", false);

		if (isTalon) {
			if (allowTargetChange) {
				target = SmartDashboard.getNumber("PIDTarget", 0.0);
			}
			if (!start) {
				talonSRX.set(ControlMode.PercentOutput, 0.0);
				SmartDashboard.putBoolean("PIDStart", false);
				allowTargetChange = true;
			} else {
				talonSRX.config_kP(0, p, 10);
				talonSRX.config_kI(0, i, 10);
				talonSRX.config_kD(0, d, 10);
				talonSRX.config_kF(0, f, 10);
				talonSRX.set(ControlMode.Position, target);
				allowTargetChange = false;
			}

			SmartDashboard.putNumber("PIDOutput", talonSRX.getMotorOutputPercent());
			SmartDashboard.putNumber("PIDSource", talonSRX.getSelectedSensorPosition(0));
			SmartDashboard.putBoolean("PIDOnTarget",
					Math.abs(talonSRX.getSelectedSensorPosition(0) - target) < talonSRX.getClosedLoopError(0));

		} else {
			if (allowTargetChange) {
				target = SmartDashboard.getNumber("PIDTarget", 0.0);
				controller.setSetpoint(target);

			}

			if (!start) {
				controller.disable();
				SmartDashboard.putBoolean("PIDStart", false);
				allowTargetChange = true;
			} else {
				controller.setPID(p, i, d);
				controller.enable();
				allowTargetChange = false;
			}

			SmartDashboard.putNumber("PIDOutput", output.get());
			SmartDashboard.putNumber("PIDSource", source.pidGet());
			SmartDashboard.putBoolean("PIDOnTarget", controller.onTarget());
		}

	}

	/**
	 * Returns PID calculated motor output
	 * 
	 * @return Calculated motor output
	 */
	public double getOutput() {
		return output.get();
	}
}
