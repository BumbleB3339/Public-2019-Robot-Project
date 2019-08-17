package frc.bumblelib.bumblelib_autonomous.auto_commands;

import edu.wpi.first.wpilibj.command.TimedCommand;
import frc.robot.Robot;
import frc.robot.subsystems.Drivetrain.DrivetrainControlState;
import frc.robot.subsystems.Drivetrain.Gear;

/**
 *
 */
public class DriveByTime extends TimedCommand {

	private double power;
	private Gear gear;

	public DriveByTime(double timeout, double power, Gear gear) {
		super(timeout);
		requires(Robot.m_drivetrain);
		this.power = power;
		this.gear = gear;
	}

	// Called just before this Command runs the first time
	@Override
	protected void initialize() {
		Robot.m_drivetrain.initDrivetrainControlState(DrivetrainControlState.OPEN_LOOP);
		Robot.m_drivetrain.setGear(gear);
	}

	// Called repeatedly when this Command is scheduled to run
	@Override
	protected void execute() {
		Robot.m_drivetrain.setLeftRightMotorOutputs(power, power);
	}

	// Called once after timeout
	@Override
	protected void end() {
		Robot.m_drivetrain.stopMotors();
	}

	// Called when another command which requires one or more of the same
	// subsystems is scheduled to run
	@Override
	protected void interrupted() {
		end();
	}
}
