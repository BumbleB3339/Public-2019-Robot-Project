package frc.robot.profiles;

import frc.bumblelib.bumblelib_autonomous.pathing.RobotReferencePoint;
import frc.bumblelib.util.PIDPreset;
import frc.robot.RobotConfiguration;
import frc.robot.RobotGameState.Direction;
import frc.robot.subsystems.Drivetrain.Gear;

/**
 * An abstract class containing all relevant robot parameters
 * 
 * @author BumbleB 3339
 *
 */
public abstract class RobotProfile {

	public AutonomousParams autonomousParams = new AutonomousParams();
	public PathfinderParams pathfinderParams = new PathfinderParams();
	public RobotReferencePoints robotReferencePoints = new RobotReferencePoints();
	public DrivetrainParams drivetrainParams = new DrivetrainParams();
	public IntakeArmParams intakeArmParams = new IntakeArmParams();
	public LiftArmParams liftArmParams = new LiftArmParams();
	public ElementArmParams elementArmParams = new ElementArmParams();
	public CargoHandlerParams cargoHandlerParams = new CargoHandlerParams();
	public LiftParams liftParams = new LiftParams();
	public ClimbParams climbParams = new ClimbParams();
	public HatchHandlerParams hatchHandlerParams = new HatchHandlerParams();
	public IntakeRollerParams floorRollerParams = new IntakeRollerParams();
	public PixyParams pixyParams = new PixyParams();
	public RobotStateParams robotStateParams = new RobotStateParams();
	public CameraParams cameraParams = new CameraParams();
	public BumbleDifferentialDriveParams bumbleDifferentialDriveParams = new BumbleDifferentialDriveParams();
	public AlignToTargetParams alignToTargetParams = new AlignToTargetParams();

	public static class AutonomousParams {
		public double wheel_diameter, wheel_base_effective_width, robot_length, robot_width, max_rotation_radius; // Robot length and width
																								// are measured WITH
																								// bumpers on.
		public int ticks_per_rev;
	}

	public static class PathfinderParams {
		public PowerGear powerGear = new PowerGear();
		public SpeedGear speedGear = new SpeedGear();

		public double maxJerk, deltaTime;

		public abstract class GearParams {
			public double maxAccel, maxVel, kV, kA, gyroP, vIntercept;
			public PIDPreset slowPathPreset, fastPathPreset;
			public double rotMaxAccel, rotMaxVel, rotGyroP, rotVIntercept, rotKv, rotKa;
		}

		public class PowerGear extends GearParams {
		}

		public class SpeedGear extends GearParams {
		}

		public GearParams getGearParams(Gear gear) {
			return gear == Gear.POWER_GEAR ? powerGear : speedGear;
		}
	}

	public static class RobotReferencePoints {
		public RobotReferencePoint jaciCenterOfRotation; // Pathfinder can't handle with non robot centered x coordinate
															// so this center of rotation is always with x = robotWidth
															// / 2.0
		public RobotReferencePoint realCenterOfRotation; // real robot center of rotation, can be in any cooridinate
															// (for use when turnig in place)
		public RobotReferencePoint frontCenter, rearCenter;
		public RobotReferencePoint frontPixy, rearPixy;
		public RobotReferencePoint elementArmCenter;
	}

	public static class BumbleDifferentialDriveParams {
		public double slowTurnPower;
		public double TRXQuickTurnCoefficient; 
	}

	public static class PixyParams {
		public double ServoAngleInZeroDegrees;
		public int pixyUID;
		public double sensorAngle;
		public double edgeServoAngle;
		public double maxServoPWM;
		public double minServoPWM;
		public double forwardAngle;
		public double backwardAngle;
		public double elementArmXOffsetForward;
		public double elementArmXOffsetBackward;
	}

	public static class AlignToTargetParams {
		public double minSlowdownPower;
		public double maxSlowdownPower;
		public double proximityPower;
		public double alignOnTargetWallPower;
		public double p_offsetAngle_X;
		public double p_xDisplacement_X;
		public double p_offsetAngle_Chase;
		public double maxRotationPower_Chase;
		public double rotationBasePower;
		public double maxRotationPower_General;
	}

	public static class DrivetrainParams {
		public boolean isPCMConnected, enableCompressor, isRightSideInverted, isLeftSideInverted,
				leftEncoderSensorPhase, rightEncoderSensorPhase, isVoltageCompensationEnabled;
		public int smartCurrentLimit;
		public double rampRate, nominalVoltage;
	}

	public static class IntakeArmParams {
		public boolean isMotorInverted;
		public double voltageInAngle0;
		public double voltageInAngle90;
		public PIDPreset pidPreset;
		public double manualPowerForward;
		public double manualPowerBackward;
		public double fPower;
		public double frictionOvercomePower;
		public double angleToBalanced;
		public double supportRobotPower;
		public double liftRobotPower;
		public double releaseRobotPower;
		public double foldedAngle;
		public double collectCargoAngle;
		public double collectHatchAngle;
		public double deliverHatchPanelAngle;
		public double prepareToClimbAngle;
		public double holdRobotAngle;
	}

	public static class LiftArmParams {
		public double maxOutputDownStrong;
		public boolean isMotorInverted;
		public double voltageInAngle0;
		public double voltageInAngle90;
		public PIDPreset pidPreset_none;
		public PIDPreset pidPreset_cargo;
		public PIDPreset pidPreset_hatch;
		public double outputRange;
		public double manualPowerForward;
		public double manualPowerBackward;
		public double gravityF_0_none;
		public double gravityF_90_none;
		public double gravityF_0_cargo;
		public double gravityF_90_cargo;
		public double gravityF_0_hatchPanel;
		public double gravityF_90_hatchPanel;
		public double frictionOvercomePower;
		public double maxOutputUpStrong;
		public double rampRate;
		public double angleFreedom; // Arm freedom when the arm leans to direction opposite to the one it was
									// calibrated in
	}

	public static class LiftParams {
		public boolean isMotorInverted;
		public boolean sensorPhase;
		public double manualPowerUp;
		public double manualPowerDown;
		public double gravityCompensationPower;
		public double resetModePowerDown;
		public PIDPreset upMovementPIDPreset;
		public PIDPreset downMovementPIDPreset;
		public boolean bottomSwitchNormallyClosed;
		public boolean isBottomSwitchDefective;
		public double positionToleranceHeight;
	}

	public static class ElementArmParams {
		public boolean isMotorInverted;
		public double voltageInAngleMinus90;
		public double voltageInAngle90;
		public PIDPreset pidPreset_none;
		public PIDPreset pidPreset_cargo;
		public PIDPreset pidPreset_hatch;
		public double outputRange;
		public double manualPowerForward;
		public double manualPowerBackward;
		public double gravityF_none;
		public double gravityF_cargo;
		public double gravityF_hatchPanel;
		public double frictionOvercomePower;
		public double gravityF_Ejecting;
		public double angleFreedom; // Arm freedom when the arm leans to direction opposite to the one it was
									// calibrated in
	}

	public static class CargoHandlerParams {
		public boolean isMotorInverted;
	}

	public static class ClimbParams {
		public boolean isFrontLeftMotorInverted;
		public boolean isBackLeftMotorInverted;
		public boolean isFrontRightMotorInverted;
		public boolean isBackRightMotorInverted;

		public double frontLeftPotentiometerUpperVoltage; // 0 height is the upper voltage
		public double frontLeftPotentiometerLowerVoltage;
		public double frontLeftPotentiometerUpperValue;
		public double frontLeftPotentiometerLowerValue;

		public double frontRightPotentiometerUpperVoltage; // 0 height is the upper voltage
		public double frontRightPotentiometerLowerVoltage;
		public double frontRightPotentiometerUpperValue;
		public double frontRightPotentiometerLowerValue;

		public double rearLeftPotentiometerUpperVoltage; // 0 height is the upper voltage
		public double rearLeftPotentiometerLowerVoltage;
		public double rearLeftPotentiometerUpperValue;
		public double rearLeftPotentiometerLowerValue;

		public double rearRightPotentiometerUpperVoltage; // 0 height is the upper voltage
		public double rearRightPotentiometerLowerVoltage;
		public double rearRightPotentiometerUpperValue;
		public double rearRightPotentiometerLowerValue;

		public double foldedHeight;
	}

	public static class HatchHandlerParams {
		public double timeToFoldHolderAfterPushExtend;
	}

	public static class IntakeRollerParams {
		public boolean isMotorInverted;
		public double collectHatchPanelPower;
		public double collectCargoPower;
		public double deliverHatchPanelPower;
		public double releaseHatchPanelPower;
		public double dragPower;
	}

	public static class RobotStateParams {
		// These ones are not states, but common bases for other states
		public RobotConfiguration fromAbove;

		public RobotConfiguration folded;
		public RobotConfiguration foldedWithCargo;
		public RobotConfiguration climbLevel3;
		public RobotConfiguration climbLevel2;
		public RobotConfiguration frontIntakeArmGapHigh;
		public RobotConfiguration frontIntakeArmGapLow;
		public RobotConfiguration l1Hatch;
		public RobotConfiguration feederHatchStart;
		public RobotConfiguration feederHatchEnd;
		public RobotConfiguration feederCargoFromAbove;
		public RobotConfiguration feederCargoFromBelow;
		public RobotConfiguration floorCargo;
		public RobotConfiguration floorHatchStart;
		public RobotConfiguration floorHatchDelivery;
		public RobotConfiguration floorHatchEnd;
		public RobotConfiguration l2Hatch;
		public RobotConfiguration l2HatchFolded;
		public RobotConfiguration l3Hatch;
		public RobotConfiguration l1Cargo;
		public RobotConfiguration l2CargoFromAbove;
		public RobotConfiguration l2CargoFromBelow;
		public RobotConfiguration l3Cargo;
		public RobotConfiguration shipCargoFromAbove;
		public RobotConfiguration shipCargoFromBelow;
		public RobotConfiguration abortCargoFloorCollect;
	}

	public static class CameraParams {
		public Direction unflippedDirection;
		public double servoOffsetAngle;
		public boolean negateServoAngles;
	}
}
