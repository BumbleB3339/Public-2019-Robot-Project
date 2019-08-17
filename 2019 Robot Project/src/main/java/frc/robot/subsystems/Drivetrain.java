package frc.robot.subsystems;

import static frc.robot.profiles.InitProfiles.ROBOT_PROFILE;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.bumblelib.util.BumbleBDrivetrain;
import frc.bumblelib.util.BumbleDifferentialDrive;
import frc.bumblelib.util.SmartdashboardDebugging;
import frc.bumblelib.util.hardware.Gyro;
import frc.bumblelib.util.hardware.NavX;
import frc.robot.Robot;
import frc.robot.RobotMap;
import frc.robot.commands.Drive;
import frc.robot.profiles.InitProfiles;

public class Drivetrain extends Subsystem implements SmartdashboardDebugging, BumbleBDrivetrain {

	// Motor Controllers
	private CANSparkMax frontRight = new CANSparkMax(RobotMap.DrivetrainPorts.FRONT_RIGHT, MotorType.kBrushless);
	private CANSparkMax middleRight = new CANSparkMax(RobotMap.DrivetrainPorts.MIDDLE_RIGHT, MotorType.kBrushless);
	private CANSparkMax rearRight = new CANSparkMax(RobotMap.DrivetrainPorts.REAR_RIGHT, MotorType.kBrushless);
	private CANSparkMax frontLeft = new CANSparkMax(RobotMap.DrivetrainPorts.FRONT_LEFT, MotorType.kBrushless);
	private CANSparkMax middleLeft = new CANSparkMax(RobotMap.DrivetrainPorts.MIDDLE_LEFT, MotorType.kBrushless);
	private CANSparkMax rearLeft = new CANSparkMax(RobotMap.DrivetrainPorts.REAR_LEFT, MotorType.kBrushless);

	// Gear shifter
	private Solenoid gearShifter;
	private final Gear DEFAULT_GEAR = Gear.SPEED_GEAR;
	private final boolean IS_PCM_CONNECTED = ROBOT_PROFILE.drivetrainParams.isPCMConnected;

	private Gear currentGear = DEFAULT_GEAR;

	public Compressor compressor;
	private final boolean ENABLE_COMPRESSOR = ROBOT_PROFILE.drivetrainParams.enableCompressor;

	public BumbleDifferentialDrive bumbleDrive = new BumbleDifferentialDrive(frontLeft, frontRight);

	// Drivetrain encoders are connected to motor controllers of other subsystems
	// because SPARK MAX doesn't support connecting external encoders when in
	// brushless mode. This is correct for February 2019.
	public WPI_TalonSRX leftEncoderController = Robot.m_intakeArm.intakeArmMotor;
	public WPI_TalonSRX rightEncoderController = (WPI_TalonSRX) Robot.m_rearClimb.rightMotor;

	private final double NAVX_STOP_RATE = 0.05;
	private NavX navX = new NavX();

	/* Drivetrain Constants */
	private static final int SMART_CURRENT_LIMIT = ROBOT_PROFILE.drivetrainParams.smartCurrentLimit;

	private static final double NOMINAL_VOLTAGE = ROBOT_PROFILE.drivetrainParams.nominalVoltage;
	private static final boolean IS_VOLTAGE_COMPENSATION_ENABLED = ROBOT_PROFILE.drivetrainParams.isVoltageCompensationEnabled;

	private static final double RAMP_RATE = ROBOT_PROFILE.drivetrainParams.rampRate; // seconds from neutral to full
																						// power

	private static final IdleMode DEFAULT_IDLE_MODE = IdleMode.kBrake;

	private static final boolean IS_RIGHT_SIDE_INVERTED = ROBOT_PROFILE.drivetrainParams.isRightSideInverted;
	private static final boolean IS_LEFT_SIDE_INVERTED = ROBOT_PROFILE.drivetrainParams.isLeftSideInverted;

	private static final boolean LEFT_ENCODER_SENSOR_PHASE = ROBOT_PROFILE.drivetrainParams.leftEncoderSensorPhase;
	private static final boolean RIGHT_ENCODER_SENSOR_PHASE = ROBOT_PROFILE.drivetrainParams.rightEncoderSensorPhase;

	public Drivetrain() {
		/* Drivetrain Configuration */
		restoreFactoryDefaults();
		setIdlelMode(DEFAULT_IDLE_MODE);
		configCurrentLimit(SMART_CURRENT_LIMIT);
		setInverted(IS_LEFT_SIDE_INVERTED, IS_RIGHT_SIDE_INVERTED);
		configEncoders(LEFT_ENCODER_SENSOR_PHASE, RIGHT_ENCODER_SENSOR_PHASE);
		setFollowerMode();
		enableVoltageCompensation(IS_VOLTAGE_COMPENSATION_ENABLED, NOMINAL_VOLTAGE);
		enableRampRate(RAMP_RATE);
		navX.setNavxStopRate(NAVX_STOP_RATE);

		// Gear shifter and compressor related
		if (IS_PCM_CONNECTED) {
			gearShifter = new Solenoid(RobotMap.DrivetrainPorts.GEAR_SHIFTER);
			compressor = new Compressor();
			compressor.setClosedLoopControl(ENABLE_COMPRESSOR);
		}
	}

	public void setGear(Gear gear) {
		currentGear = gear;
		gearShifter.set(currentGear.getSolenoidOn());
	}

	public Gear getGear() {
		return currentGear;
	}

	public void restoreFactoryDefaults() {
		frontRight.restoreFactoryDefaults();
		middleRight.restoreFactoryDefaults();
		rearRight.restoreFactoryDefaults();

		frontLeft.restoreFactoryDefaults();
		middleLeft.restoreFactoryDefaults();
		rearLeft.restoreFactoryDefaults();
	}

	public void setIdlelMode(IdleMode idleMode) {
		frontLeft.setIdleMode(idleMode);
		frontRight.setIdleMode(idleMode);
		middleLeft.setIdleMode(idleMode);
		middleRight.setIdleMode(idleMode);
		rearLeft.setIdleMode(idleMode);
		rearRight.setIdleMode(idleMode);
	}

	public void configCurrentLimit(int smartCurrentLimit) {
		frontLeft.setSmartCurrentLimit(smartCurrentLimit);
		frontRight.setSmartCurrentLimit(smartCurrentLimit);
		middleLeft.setSmartCurrentLimit(smartCurrentLimit);
		middleRight.setSmartCurrentLimit(smartCurrentLimit);
		rearLeft.setSmartCurrentLimit(smartCurrentLimit);
		rearRight.setSmartCurrentLimit(smartCurrentLimit);
	}

	public void setInverted(boolean isLeftSideInverted, boolean isRightSideInverted) {
		frontRight.setInverted(isRightSideInverted);
		middleRight.setInverted(isRightSideInverted);
		rearRight.setInverted(isRightSideInverted);

		frontLeft.setInverted(isLeftSideInverted);
		middleLeft.setInverted(isLeftSideInverted);
		rearLeft.setInverted(isLeftSideInverted);
	}

	public void configEncoders(boolean leftEncoderSensorPhase, boolean rightEncoderSensorPhase) {
		leftEncoderController.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, 10);
		rightEncoderController.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, 10);

		leftEncoderController.setSensorPhase(leftEncoderSensorPhase);
		rightEncoderController.setSensorPhase(rightEncoderSensorPhase);

		leftEncoderController.setStatusFramePeriod(StatusFrameEnhanced.Status_3_Quadrature, 20, 10);
		rightEncoderController.setStatusFramePeriod(StatusFrameEnhanced.Status_3_Quadrature, 20, 10);
	}

	public void setFollowerMode() {
		middleLeft.follow(frontLeft);
		rearLeft.follow(frontLeft);

		middleRight.follow(frontRight);
		rearRight.follow(frontRight);
	}

	public void enableVoltageCompensation(boolean isEnabled, double nominalVoltage) {
		if (isEnabled) {
			frontRight.enableVoltageCompensation(nominalVoltage);
			middleRight.enableVoltageCompensation(nominalVoltage);
			rearRight.enableVoltageCompensation(nominalVoltage);

			frontLeft.enableVoltageCompensation(nominalVoltage);
			middleLeft.enableVoltageCompensation(nominalVoltage);
			rearRight.enableVoltageCompensation(nominalVoltage);
		} else {
			frontRight.disableVoltageCompensation();
			middleRight.disableVoltageCompensation();
			rearRight.disableVoltageCompensation();

			frontLeft.disableVoltageCompensation();
			middleLeft.disableVoltageCompensation();
			rearRight.disableVoltageCompensation();
		}

	}

	public void enableRampRate(double rampRate) {
		frontRight.setOpenLoopRampRate(rampRate);
		middleRight.setOpenLoopRampRate(rampRate);
		rearRight.setOpenLoopRampRate(rampRate);

		frontLeft.setOpenLoopRampRate(rampRate);
		middleLeft.setOpenLoopRampRate(rampRate);
		rearRight.setOpenLoopRampRate(rampRate);
	}

	public void disableRampRate() {
		frontRight.setOpenLoopRampRate(0);
		middleRight.setOpenLoopRampRate(0);
		rearRight.setOpenLoopRampRate(0);

		frontLeft.setOpenLoopRampRate(0);
		middleLeft.setOpenLoopRampRate(0);
		rearRight.setOpenLoopRampRate(0);
	}

	@Override
	public void initDefaultCommand() {
		setDefaultCommand(new Drive());
	}

	public double getAverageVoltage() {
		return (frontLeft.getBusVoltage() + middleLeft.getBusVoltage() + rearLeft.getBusVoltage()
				+ frontRight.getBusVoltage() + middleRight.getBusVoltage() + rearRight.getBusVoltage()) / 6;
	}

	public double getRightCurrent() {
		return (frontRight.getOutputCurrent() + middleRight.getOutputCurrent() + rearRight.getOutputCurrent()) / 3.0;
	}

	public double getLeftCurrent() {
		return (frontLeft.getOutputCurrent() + middleLeft.getOutputCurrent() + rearLeft.getOutputCurrent()) / 3.0;
	}

	public double lastAvgPower = 0.0;

	public void stopMotors() {
		bumbleDrive.stopMotor();
	}

	@Override
	public void sendDebuggingData() {
		SmartDashboard.putNumber("Debugging/Drivetrain/Left Encoder Ticks",
				leftEncoderController.getSelectedSensorPosition());
		SmartDashboard.putNumber("Debugging/Drivetrain/Right Encoder Ticks",
				rightEncoderController.getSelectedSensorPosition());
		SmartDashboard.putNumber("Debugging/Drivetrain/Left Encoder Distance", getLeftDistance());
		SmartDashboard.putNumber("Debugging/Drivetrain/Right Encoder Distance", getRightDistance());
		SmartDashboard.putNumber("Debugging/Drivetrain/Right Encoder Speed", getRightVelocity());
		SmartDashboard.putNumber("Debugging/Drivetrain/Left Encoder Speed", getLeftVelocity());
		SmartDashboard.putNumber("Debugging/Drivetrain/Right Current", getRightCurrent());
		SmartDashboard.putNumber("Debugging/Drivetrain/Left Current", getLeftCurrent());
		SmartDashboard.putBoolean("Debugging/Drivetrain/Is Drivetrain Stopped", isDrivetrainStopped());
		SmartDashboard.putNumber("Debugging/NavX/Pitch", navX.getPitch());
		SmartDashboard.putBoolean("Debugging/NavX/Is Moving", navX.isMoving());
		SmartDashboard.putNumber("Debugging/NavX/Raw Yaw", navX.getYaw());
		SmartDashboard.putBoolean("Debugging/NavX/Is Connected", navX.isConnected());
		SmartDashboard.putString("Current gear", getGear().toString());
	}

	public void setLeftRightMotorOutputs(double leftOutput, double rightOutput) {
		bumbleDrive.setLeftRightMotorOutputs(leftOutput, rightOutput, false);
	}

	public void initDrivetrainControlState(DrivetrainControlState drivetrainControlState) {
		switch (drivetrainControlState) {
		case OPEN_LOOP:
			frontRight.set(0);
			frontLeft.set(0);
			bumbleDrive.setSafetyEnabled(true);
			bumbleDrive.setAcceleraionActive(false);
			enableRampRate(RAMP_RATE);
			break;
		case PATH_FOLLOWING:
			bumbleDrive.setSafetyEnabled(false); // TODO: IS it really necessary?
			bumbleDrive.setAcceleraionActive(false);
			disableRampRate();
			break;
		}
	}

	@Override
	public Gyro getGyro() {
		return navX;
	}

	@Override
	public int getLeftEncoderTicks() {
		return leftEncoderController.getSelectedSensorPosition(0);
	}

	@Override
	public int getRightEncoderTicks() {
		return rightEncoderController.getSelectedSensorPosition(0);
	}

	@Override
	public double getRightVelocity() {
		return Math.PI
				* (10 * rightEncoderController.getSelectedSensorVelocity()
						* InitProfiles.ROBOT_PROFILE.autonomousParams.wheel_diameter)
				/ (InitProfiles.ROBOT_PROFILE.autonomousParams.ticks_per_rev);
	}

	@Override
	public double getLeftVelocity() {
		return Math.PI
				* (10 * leftEncoderController.getSelectedSensorVelocity()
						* InitProfiles.ROBOT_PROFILE.autonomousParams.wheel_diameter)
				/ (InitProfiles.ROBOT_PROFILE.autonomousParams.ticks_per_rev);
	}

	@Override
	public double getAverageVelocity() {
		return (getLeftVelocity() + getRightVelocity()) / 2.0;
	}

	public double getMotorsPowers() {
		return (frontLeft.get() + frontRight.get()) / 2.0;
	}

	public double getLastMotorsPowers() {
		return lastAvgPower;
	}

	// todo:
	public double getLeftDistance() {
		return Math.PI * (getLeftEncoderTicks() * InitProfiles.ROBOT_PROFILE.autonomousParams.wheel_diameter)
				/ InitProfiles.ROBOT_PROFILE.autonomousParams.ticks_per_rev;
	}

	// todo:
	public double getRightDistance() {
		return Math.PI * (getRightEncoderTicks() * InitProfiles.ROBOT_PROFILE.autonomousParams.wheel_diameter)
				/ InitProfiles.ROBOT_PROFILE.autonomousParams.ticks_per_rev;
	}

	public double getAverageDistance() {
		return (getRightDistance() + getLeftDistance()) / 2.0;
	}

	public void resetEncoders() {
		leftEncoderController.setSelectedSensorPosition(0);
		rightEncoderController.setSelectedSensorPosition(0);
	}

	public boolean isDrivetrainStopped() {
		return Math.abs(getAverageVelocity()) < 0.03;
	}

	// The robot drivetrain's various states.
	public enum DrivetrainControlState {
		OPEN_LOOP, // open loop voltage control
		PATH_FOLLOWING; // motion profiling control
	}

	public double getAverageCurrent() {
		return (getRightCurrent() + getLeftCurrent()) / 2;
	}

	public enum Gear {
		POWER_GEAR, SPEED_GEAR;

		public Gear getOtherGear() {
			return this.equals(POWER_GEAR) ? SPEED_GEAR : POWER_GEAR;
		}

		public boolean getSolenoidOn() {
			switch (this) {
			case POWER_GEAR:
				return true;
			case SPEED_GEAR:
				return false;
			default:
				return false;
			}
		}
	}
}
