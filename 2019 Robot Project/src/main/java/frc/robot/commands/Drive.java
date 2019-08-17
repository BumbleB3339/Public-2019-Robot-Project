package frc.robot.commands;

import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.OI;
import frc.robot.Robot;
import frc.robot.subsystems.Drivetrain.DrivetrainControlState;

/**
 *
 */
public class Drive extends Command {

	private final static DriveType DEFAULT_DRIVE_TYPE = DriveType.TRX;
	SendableChooser<DriveType> driveModeChooser = new SendableChooser<DriveType>();

	public Drive() {
		requires(Robot.m_drivetrain);

		// Drive Type Selector
		driveModeChooser = new SendableChooser<DriveType>();
		driveModeChooser.setDefaultOption(DEFAULT_DRIVE_TYPE.toString(), DEFAULT_DRIVE_TYPE);
		driveModeChooser.addOption(DriveType.TANK.toString(), DriveType.TANK);
		driveModeChooser.addOption(DriveType.TRX.toString(), DriveType.TRX);
		driveModeChooser.addOption(DriveType.GTA.toString(), DriveType.GTA);
		SmartDashboard.putData("Drive Type", driveModeChooser);
	}

	@Override
	protected void initialize() {
		Robot.m_drivetrain.initDrivetrainControlState(DrivetrainControlState.OPEN_LOOP);
	}

	@Override
	protected void execute() {
		switch (driveModeChooser.getSelected()) {
		case GTA:
			Robot.m_drivetrain.bumbleDrive.GTADrive(OI.driverController.getX(Hand.kLeft),
					OI.driverController.getTriggerAxis(Hand.kLeft), OI.driverController.getTriggerAxis(Hand.kRight),
					OI.driverController.getAButton());
			break;
		case TRX:
			if (OI.driverController.getTriggerAxis(Hand.kLeft) > 0.1
					|| OI.driverController.getTriggerAxis(Hand.kRight) > 0.1) {
			}
			Robot.m_drivetrain.bumbleDrive.TRXDrive(OI.driverController.getY(Hand.kLeft),
					OI.driverController.getTriggerAxis(Hand.kLeft), OI.driverController.getTriggerAxis(Hand.kRight),
					OI.driverController.getAButton());
			break;
		case TANK:
			Robot.m_drivetrain.bumbleDrive.tankDrive(OI.driverController.getY(Hand.kLeft),
					OI.driverController.getY(Hand.kRight));
			break;
		}
	}

	@Override
	protected boolean isFinished() {
		return false;
	}

	@Override
	protected void end() {
		Robot.m_drivetrain.stopMotors();
	}

	@Override
	protected void interrupted() {
		end();
	}

	private enum DriveType {
		TANK, TRX, GTA;

		@Override
		public String toString() {
			switch (this) {
			case TANK:
				return "Tank Drive";
			case TRX:
				return "TRX Drive";
			case GTA:
				return "GTA Drive";
			default:
				return "Null";
			}
		}
	}
}
