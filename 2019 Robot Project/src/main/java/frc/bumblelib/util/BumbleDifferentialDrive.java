package frc.bumblelib.util;

import edu.wpi.first.wpilibj.SpeedController;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SendableBuilder;
import frc.bumblelib.util.pid_calibration_tool.BumbleMotor;
import frc.robot.Robot;
import frc.robot.subsystems.Drivetrain.Gear;

import static frc.robot.profiles.InitProfiles.ROBOT_PROFILE;

public class BumbleDifferentialDrive extends DifferentialDrive {

	private double m_leftPower = 0;
	private double m_rightPower = 0;

	private static BumbleMotor m_dummyLeftMotor = new BumbleMotor();
	private static BumbleMotor m_dummyRightMotor = new BumbleMotor();

	private SpeedController m_leftMotor;
	private SpeedController m_rightMotor;

	private boolean m_isAccelerationActive = false;

	// Acceleration Factors
	private final double POWER_PER_CYCLE = 0.02;
	private final double LOWER_LIMIT = 0.3;

	// Default Deadband
	public final double DEADBAND = 0.2;

	// TankDrive Factors
	private final boolean TANK_IS_INPUT_SQUARED = false;

	// TRXDrive Factors
	private final double TRX_POWER_EXPONENT = 1.5; // Original power argument will be raised to the power of this
													// number.
	private final double TRX_STEERING_EXPONENT = 2; // Original steering argument will be raised to the power of this
													// number.
	private final double TRX_QUICK_TURN_EXPONENT = 1.5; // Original steering argument will be raised to the power of
														// this
	private final double TRX_TRIGGERS_COEFFICIENT = 0.8; // Original triggers values will be multiplied by this number.

	private final double SLOW_TURN_POWER = ROBOT_PROFILE.bumbleDifferentialDriveParams.slowTurnPower;

	private final double TRX_QUICK_TURN_COEFFICIENT = ROBOT_PROFILE.bumbleDifferentialDriveParams.TRXQuickTurnCoefficient;
	// Original quick turn will be multiplied by this number.

	private final double TRX_POWER_COEFFICIENT = 0.7; // Original power will be multiplied by this number.

	// GTADrive Factors
	private final double GTA_POWER_EXPONENT = 1.5; // Original power argument will be raised to the power of this
													// number.
	private final double GTA_STEERING_EXPONENT = 3; // Original steering argument will be raised to the power of this
													// number.

	/**
	 * Construct a BumbleDifferentialDrive.
	 *
	 * <p>
	 * To pass multiple motors per side, use a {@link SpeedControllerGroup}. If a
	 * motor needs to be inverted, do so before passing it in.
	 */
	public BumbleDifferentialDrive(SpeedController leftMotor, SpeedController rightMotor) {
		super(m_dummyLeftMotor, m_dummyRightMotor);
		setRightSideInverted(false); // Because we invert the motors when we initiate them

		this.m_leftMotor = leftMotor;
		this.m_rightMotor = rightMotor;

		super.setDeadband(DEADBAND);
	}

	/**
	 * Enables or disables the operation of the moderate acceleration algorithem.
	 */
	public void setAcceleraionActive(boolean isActive) {
		m_isAccelerationActive = isActive;
	}

	/**
	 * Tank drive method for differential drive platform.
	 *
	 * @param leftSpeed  The robot's left side speed along the X axis [-1.0..1.0].
	 *                   Forward is positive.
	 * @param rightSpeed The robot's right side speed along the X axis [-1.0..1.0].
	 *                   Forward is positive.
	 */
	@Override
	public void tankDrive(double leftSpeed, double rightSpeed) {
		super.tankDrive(leftSpeed, rightSpeed, TANK_IS_INPUT_SQUARED);

		setLeftRightMotorOutputs(m_dummyLeftMotor.get(), m_dummyRightMotor.get());
	}

	/**
	 * TRX drive method for differential drive platform.
	 *
	 * <p>
	 * The left and right triggers arguments control the curvature of the robot's
	 * path rather than its rate of heading change. This makes the robot more
	 * controllable at high speeds. Also handles the robot's quick turn
	 * functionality - "quick turn" overrides constant-curvature turning for
	 * turn-in-place maneuvers.
	 *
	 * @param yAxis        Value of the y axis of the controller.
	 * @param leftTrigger  Value of the left trigger of the controller.
	 * @param rightTrigger Value of the right trigger of the controller.
	 * @param isQuickTurn  If set, overrides constant-curvature turning for
	 *                     turn-in-place maneuvers.
	 */
	public void TRXDrive(double yAxis, double leftTrigger, double rightTrigger, boolean isQuickTurn) {
		if ((rightTrigger > 0.8 || leftTrigger > 0.8) && Math.abs(yAxis) < DEADBAND && !isQuickTurn) {

			if (leftTrigger > 0.8) {
				setLeftRightMotorOutputs(-SLOW_TURN_POWER, SLOW_TURN_POWER);
			} else if (rightTrigger > 0.8) {
				setLeftRightMotorOutputs(SLOW_TURN_POWER, -SLOW_TURN_POWER);
			}

		} else {
			if (!isQuickTurn) {
				rightTrigger *= TRX_TRIGGERS_COEFFICIENT;
				leftTrigger *= TRX_TRIGGERS_COEFFICIENT;
			}
			double xSpeed = Math.copySign(Math.pow(Math.abs(yAxis), TRX_POWER_EXPONENT), yAxis);
			double zRotation = (isQuickTurn ? 1 : Math.signum(yAxis))
					* Math.copySign(
							Math.pow(Math.abs(rightTrigger - leftTrigger),
									isQuickTurn ? TRX_QUICK_TURN_EXPONENT : TRX_STEERING_EXPONENT),
							rightTrigger - leftTrigger);
			super.curvatureDrive(xSpeed, zRotation, isQuickTurn);
			if (isQuickTurn) {
				setLeftRightMotorOutputs(m_dummyLeftMotor.get() * TRX_QUICK_TURN_COEFFICIENT,
						m_dummyRightMotor.get() * TRX_QUICK_TURN_COEFFICIENT);
			} else {
				double powerCoeff = Robot.m_drivetrain.getGear() == Gear.SPEED_GEAR ? TRX_POWER_COEFFICIENT : 1;
				setLeftRightMotorOutputs(m_dummyLeftMotor.get() * powerCoeff, m_dummyRightMotor.get() * powerCoeff);
			}
		}
	}

	/**
	 * GTA drive method for differential drive platform.
	 *
	 * <p>
	 * The x axis argument controls the curvature of the robot's path rather than
	 * its rate of heading change. This makes the robot more controllable at high
	 * speeds. Also handles the robot's quick turn functionality - "quick turn"
	 * overrides constant-curvature turning for turn-in-place maneuvers.
	 *
	 * @param xAxis        Value of the x axis of the controller.
	 * @param leftTrigger  Value of the left trigger of the controller.
	 * @param rightTrigger Value of the right trigger of the controller.
	 * @param isQuickTurn  If set, overrides constant-curvature turning for
	 *                     turn-in-place maneuvers.
	 */
	public void GTADrive(double xAxis, double leftTrigger, double rightTrigger, boolean isQuickTurn) {
		double xSpeed = Math.copySign(Math.pow(Math.abs(rightTrigger - leftTrigger), GTA_POWER_EXPONENT),
				rightTrigger - leftTrigger);
		double zRotation = (!isQuickTurn ? Math.signum(rightTrigger - leftTrigger) : 1)
				* Math.copySign(Math.pow(Math.abs(xAxis), GTA_STEERING_EXPONENT), xAxis);

		super.curvatureDrive(xSpeed, zRotation, isQuickTurn);

		setLeftRightMotorOutputs(m_dummyLeftMotor.get(), m_dummyRightMotor.get());
	}

	/**
	 * Set left and right motor outputs.
	 * 
	 * @param leftOutput  Output of left motor.
	 * @param rightOutput Output of right motor.
	 */
	public void setLeftRightMotorOutputs(double leftOutput, double rightOutput) {
		setLeftRightMotorOutputs(leftOutput, rightOutput, m_isAccelerationActive);
	}

	/**
	 * Set left and right motor outputs.
	 * 
	 * @param leftOutput     Output of left motor.
	 * @param rightOutput    Output of right motor.
	 * @param isAcceleration Override to the isAcceleration setting of the class.
	 */
	public void setLeftRightMotorOutputs(double leftOutput, double rightOutput, boolean isAcceleration) {
		m_leftPower = isAcceleration ? limitPowerAcc(m_leftPower, leftOutput) : leftOutput;
		m_rightPower = isAcceleration ? limitPowerAcc(m_rightPower, rightOutput) : rightOutput;

		setLeftMotorOutput(m_leftPower);
		setRightMotorOutput(m_rightPower);
	}

	/**
	 * Set left motor output.
	 * 
	 * @param output Output of left motor.
	 */
	public void setLeftMotorOutput(double output) {
		m_leftMotor.set(output);
		feed();
	}

	/**
	 * Set right motor output.
	 * 
	 * @param output Output of right motor.
	 */
	public void setRightMotorOutput(double output) {
		m_rightMotor.set(output);
		feed();
	}

	@Override
	public void stopMotor() {
		m_leftMotor.stopMotor();
		m_rightMotor.stopMotor();
		feed();
	}

	@Override
	public void initSendable(SendableBuilder builder) {
		builder.setSmartDashboardType("DifferentialDrive");
		builder.setActuator(true);
		builder.setSafeState(this::stopMotor);
		builder.addDoubleProperty("Left Motor Speed", m_leftMotor::get, m_leftMotor::set);
		builder.addDoubleProperty("Right Motor Speed", m_rightMotor::get, m_rightMotor::set);
	}

	// Accelerated Drive Methods
	private double limitPowerAcc(double currentPower, double newPower) {
		if (newPower >= LOWER_LIMIT) {
			if (currentPower >= LOWER_LIMIT) {
				return limitEdges(newPower, LOWER_LIMIT, currentPower + POWER_PER_CYCLE);
			} else {
				return LOWER_LIMIT;
			}
		} else if (newPower <= -LOWER_LIMIT) {
			if (currentPower <= -LOWER_LIMIT) {
				return limitEdges(newPower, currentPower - POWER_PER_CYCLE, -LOWER_LIMIT);
			} else {
				return -LOWER_LIMIT;
			}
		} else {
			return newPower;
		}
	}

	private double limitEdges(double value, double lowerLimit, double upperLimit) {
		if (value > upperLimit)
			return upperLimit;
		if (value < lowerLimit)
			return lowerLimit;
		return value;
	}

	// TODO: Add documentation
	// positive zRotation is clockwise
	public void arcadeDrive(double zRotation, double xSpeed) {
		this.arcadeDrive(xSpeed, zRotation, false);
		setLeftMotorOutput(m_dummyLeftMotor.get());
		setRightMotorOutput(m_dummyRightMotor.get());
	}
}
