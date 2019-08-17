package frc.robot.profiles.robot_profiles;

import frc.bumblelib.bumblelib_autonomous.pathing.RobotReferencePoint;
import frc.bumblelib.util.PIDPreset;
import frc.bumblelib.util.Units;
import frc.robot.RobotConfiguration;
import frc.robot.RobotGameState.Direction;
import frc.robot.profiles.RobotProfile;

public class RobotB extends RobotProfile {

	public RobotB() {
		// Autonomous Params
		autonomousParams.robot_length = 0.97;
		autonomousParams.robot_width = 0.87;
		autonomousParams.max_rotation_radius = Math.sqrt(
				Math.pow(0.5 * autonomousParams.robot_length, 2) + Math.pow(0.5 * autonomousParams.robot_width, 2));

		autonomousParams.wheel_diameter = Units.inches_to_meters(6.0);

		double gear_output_to_wheel_reduction = 64.0 / 20.0;
		double gear_output_to_encoder_reduction = 3.0;
		int ticks_per_encoder_rev = 4096;
		autonomousParams.ticks_per_rev = (int) ((ticks_per_encoder_rev * gear_output_to_wheel_reduction
				* gear_output_to_encoder_reduction) * 1.02);

		autonomousParams.wheel_base_effective_width = 0.68;

		// Pathfinder Params
		pathfinderParams.maxJerk = 60.0;
		pathfinderParams.deltaTime = 0.02;

		// Speed Gear Values
		pathfinderParams.speedGear.maxVel = 4; // Calibrated
		pathfinderParams.speedGear.maxAccel = 2.5; // Calibrated
		pathfinderParams.speedGear.kV = 0.2209; // Calibrated
		pathfinderParams.speedGear.kA = 0.0439; // Calibrated
		pathfinderParams.speedGear.vIntercept = 0.0377; // Calibrated
		pathfinderParams.speedGear.gyroP = 0.01; // Calibrated

		pathfinderParams.speedGear.fastPathPreset = new PIDPreset(0.8, 0.0, 0.0); // Calibrated
		pathfinderParams.speedGear.slowPathPreset = new PIDPreset(0.8, 0.0, 0.0); // Calibrated

		pathfinderParams.speedGear.rotMaxVel = 3.84;
		pathfinderParams.speedGear.rotGyroP = 0.0;
		pathfinderParams.speedGear.rotMaxAccel = 3.0;
		pathfinderParams.speedGear.rotKv = 3.84;
		pathfinderParams.speedGear.rotKa = 0.0;
		pathfinderParams.speedGear.rotVIntercept = 0.1;

		// Power Gear Values
		pathfinderParams.powerGear.maxVel = 2.26; // Calibrated
		pathfinderParams.powerGear.maxAccel = 9.47; // Calibrated
		pathfinderParams.powerGear.kV = 0.4506; // Calibrated
		pathfinderParams.powerGear.kA = 0.0293; // Calibrated
		pathfinderParams.powerGear.vIntercept = 0.01; // Calibrated
		pathfinderParams.speedGear.gyroP = 0.01; // Calibrated

		pathfinderParams.powerGear.fastPathPreset = new PIDPreset(0.7, 0.0, 0.012); // Calibrated
		pathfinderParams.powerGear.slowPathPreset = new PIDPreset(0.7, 0.0, 0.012); // Calibrated

		pathfinderParams.powerGear.rotMaxVel = 1.823; // Calibrated
		pathfinderParams.powerGear.rotGyroP = 0.0; // Calibrated
		pathfinderParams.powerGear.rotMaxAccel = 11.536; // Calibrated
		pathfinderParams.powerGear.rotKv = 0.4517; // Calibrated
		pathfinderParams.powerGear.rotKa = 0.0346; // Calibrated
		pathfinderParams.powerGear.rotVIntercept = 0.0265; // Calibrated

		// Robot Reference Points
		robotReferencePoints.jaciCenterOfRotation = new RobotReferencePoint(autonomousParams.robot_width / 2.0,
				autonomousParams.robot_length / 2.0);
		robotReferencePoints.realCenterOfRotation = new RobotReferencePoint(autonomousParams.robot_width / 2.0,
				autonomousParams.robot_length / 2.0);

		robotReferencePoints.frontCenter = new RobotReferencePoint(autonomousParams.robot_width / 2.0,
				autonomousParams.robot_length);
		robotReferencePoints.rearCenter = new RobotReferencePoint(autonomousParams.robot_width / 2.0, 0.0);

		robotReferencePoints.frontPixy = new RobotReferencePoint(autonomousParams.robot_width / 2.0 + 0.23,
				autonomousParams.robot_length / 2.0);
		robotReferencePoints.rearPixy = new RobotReferencePoint(autonomousParams.robot_width / 2.0 - 0.23,
				autonomousParams.robot_length / 2.0);

		robotReferencePoints.elementArmCenter = new RobotReferencePoint(autonomousParams.robot_width / 2.0,
				autonomousParams.robot_length / 2.0);

		// Pixy params
		pixyParams.ServoAngleInZeroDegrees = -8;
		pixyParams.pixyUID = 0x02A69555;
		pixyParams.sensorAngle = 15.0;
		pixyParams.edgeServoAngle = 135.0;
		pixyParams.maxServoPWM = 2.5;
		pixyParams.minServoPWM = .5;
		pixyParams.forwardAngle = 75;
		pixyParams.backwardAngle = -75;
		pixyParams.elementArmXOffsetForward = 0.06;
		pixyParams.elementArmXOffsetBackward = 0.00;

		// Align to target
		alignToTargetParams.minSlowdownPower = 0.35;
		alignToTargetParams.maxSlowdownPower = 0.7;
		alignToTargetParams.proximityPower = 0.3;
		alignToTargetParams.alignOnTargetWallPower = 0.4;
		alignToTargetParams.p_offsetAngle_X = 0.025;
		alignToTargetParams.p_xDisplacement_X = 0.9;
		alignToTargetParams.p_offsetAngle_Chase = 0.02;
		alignToTargetParams.maxRotationPower_Chase = 0.4;
		alignToTargetParams.rotationBasePower = 0.2;
		alignToTargetParams.maxRotationPower_General = 0.6;

		// Drivetrain Params
		drivetrainParams.isPCMConnected = true;
		drivetrainParams.enableCompressor = true;
		drivetrainParams.isRightSideInverted = false;
		drivetrainParams.isLeftSideInverted = true;
		drivetrainParams.leftEncoderSensorPhase = false;
		drivetrainParams.rightEncoderSensorPhase = true;
		drivetrainParams.isVoltageCompensationEnabled = true;
		drivetrainParams.smartCurrentLimit = 40; //// 0.0 to disable
		drivetrainParams.nominalVoltage = 11;
		drivetrainParams.rampRate = 0.2; // 0.0 to disable

		bumbleDifferentialDriveParams.TRXQuickTurnCoefficient = 0.6;
		bumbleDifferentialDriveParams.slowTurnPower = 0.1;

		// IntakeArm Params
		intakeArmParams.isMotorInverted = true;
		intakeArmParams.angleToBalanced = 27;
		intakeArmParams.voltageInAngle0 = 0.333;
		intakeArmParams.voltageInAngle90 = 0.468;
		intakeArmParams.pidPreset = new PIDPreset(0.025, 0.0, 0.01);
		intakeArmParams.fPower = 0.05;
		intakeArmParams.frictionOvercomePower = 0.05;
		intakeArmParams.liftRobotPower = 0.8;
		intakeArmParams.supportRobotPower = 0.3339;
		intakeArmParams.releaseRobotPower = -0.2;
		intakeArmParams.foldedAngle = -33;
		intakeArmParams.deliverHatchPanelAngle = 45;
		intakeArmParams.collectCargoAngle = 92;
		intakeArmParams.collectHatchAngle = 152;
		intakeArmParams.prepareToClimbAngle = 27.0;
		intakeArmParams.holdRobotAngle = 150.0;

		// LiftArmParams
		liftArmParams.isMotorInverted = true;
		liftArmParams.voltageInAngle0 = 0.51;
		liftArmParams.voltageInAngle90 = 0.386;

		liftArmParams.pidPreset_none = new PIDPreset(0.022, 0.0, 0.01);
		liftArmParams.pidPreset_cargo = new PIDPreset(0.022, 0.0, 0.01);
		liftArmParams.pidPreset_hatch = new PIDPreset(0.022, 0.0, 0.02);
		liftArmParams.gravityF_0_none = 0.2;
		liftArmParams.gravityF_90_none = 0.25;
		liftArmParams.gravityF_0_cargo = 0.2;
		liftArmParams.gravityF_90_cargo = 0.25;
		liftArmParams.gravityF_0_hatchPanel = 0.33;
		liftArmParams.gravityF_90_hatchPanel = 0.24;
		liftArmParams.frictionOvercomePower = 0.06;
		liftArmParams.maxOutputUpStrong = 1.0;
		liftArmParams.rampRate = 0.1;
		liftArmParams.angleFreedom = 5.0;
		liftArmParams.maxOutputDownStrong = 0.6;

		// ElementArm Params
		elementArmParams.isMotorInverted = true;
		elementArmParams.voltageInAngleMinus90 = 0.532;
		elementArmParams.voltageInAngle90 = 0.162;
		elementArmParams.pidPreset_none = new PIDPreset(0.017, 0.0, 0.01);
		elementArmParams.pidPreset_cargo = new PIDPreset(0.017, 0.0, 0.01);
		elementArmParams.pidPreset_hatch = new PIDPreset(0.017, 0.0, 0.01);
		elementArmParams.outputRange = 0.7;
		elementArmParams.gravityF_none = 0.08;
		elementArmParams.gravityF_hatchPanel = 0.15;
		elementArmParams.gravityF_cargo = 0.08;
		elementArmParams.frictionOvercomePower = 0.09;
		elementArmParams.gravityF_Ejecting = 0.0;
		elementArmParams.angleFreedom = 12.0;

		// CargoHandler Params
		cargoHandlerParams.isMotorInverted = false;

		// Lift Params
		liftParams.isMotorInverted = false;
		liftParams.gravityCompensationPower = 0.09;
		liftParams.manualPowerDown = -0.2;
		liftParams.manualPowerUp = 0.5;
		liftParams.downMovementPIDPreset = new PIDPreset(0.1, 0.0, 0.09);
		liftParams.upMovementPIDPreset = new PIDPreset(0.1, 0.0, 0.05);
		liftParams.sensorPhase = true;
		liftParams.isBottomSwitchDefective = true;
		liftParams.bottomSwitchNormallyClosed = false;
		liftParams.positionToleranceHeight = 5.0;

		// Climb Params
		climbParams.isBackLeftMotorInverted = false;
		climbParams.isBackRightMotorInverted = true;
		climbParams.isFrontLeftMotorInverted = false;
		climbParams.isFrontRightMotorInverted = true;

		climbParams.frontLeftPotentiometerUpperVoltage = 0.737; // 0 height is the upper voltage
		climbParams.frontLeftPotentiometerLowerVoltage = 0.068; // 0.0124 per degree
		climbParams.frontLeftPotentiometerUpperValue = 0.0;
		climbParams.frontLeftPotentiometerLowerValue = 53.0;

		climbParams.frontRightPotentiometerUpperVoltage = 0.265; // 0 height is the upper voltage
		climbParams.frontRightPotentiometerLowerVoltage = 0.940;// 0.0126 per degree
		climbParams.frontRightPotentiometerUpperValue = 0.0;
		climbParams.frontRightPotentiometerLowerValue = 52.5;

		climbParams.rearLeftPotentiometerUpperVoltage = 0.254; // 0 height is the upper voltage
		climbParams.rearLeftPotentiometerLowerVoltage = 0.919; // 0.0126 per degree
		climbParams.rearLeftPotentiometerUpperValue = 0.0;
		climbParams.rearLeftPotentiometerLowerValue = 52.5;

		climbParams.rearRightPotentiometerUpperVoltage = 0.734; // 0 height is the upper voltage
		climbParams.rearRightPotentiometerLowerVoltage = 0.063; // 0.0126
		climbParams.rearRightPotentiometerUpperValue = 0.0;
		climbParams.rearRightPotentiometerLowerValue = 52.5;

		climbParams.foldedHeight = -2.0;

		// HatchHandler Params
		hatchHandlerParams.timeToFoldHolderAfterPushExtend = 0.00;

		// FloorIntake Params
		floorRollerParams.isMotorInverted = false;
		floorRollerParams.collectHatchPanelPower = -0.5;
		floorRollerParams.collectCargoPower = 1.0;
		floorRollerParams.deliverHatchPanelPower = 1.0;
		floorRollerParams.releaseHatchPanelPower = 1.0;
		floorRollerParams.dragPower = 0.508;

		robotStateParams.folded = new RobotConfiguration(0, 168, -100);
		robotStateParams.foldedWithCargo = new RobotConfiguration(14, 168, -112);

		robotStateParams.frontIntakeArmGapLow = robotStateParams.foldedWithCargo;// robotStateParams.folded.setLiftHeight(25);
		robotStateParams.frontIntakeArmGapHigh = robotStateParams.foldedWithCargo.setLiftHeight(30);// robotStateParams.folded.setLiftHeight(32);

		robotStateParams.climbLevel3 = robotStateParams.frontIntakeArmGapLow;
		robotStateParams.climbLevel2 = robotStateParams.frontIntakeArmGapLow;
		robotStateParams.fromAbove = robotStateParams.folded
				.setElementArmAngle(90 - robotStateParams.folded.liftArmAngle);

		robotStateParams.feederHatchStart = robotStateParams.folded.setElementArmAngle(-78);
		robotStateParams.feederHatchEnd = robotStateParams.folded.setElementArmAngle(-78);

		robotStateParams.feederCargoFromAbove = new RobotConfiguration(55, 165, -80);
		robotStateParams.feederCargoFromBelow = new RobotConfiguration(0, 90, 0);

		robotStateParams.floorCargo = new RobotConfiguration(4, 168, -50);

		robotStateParams.floorHatchStart = new RobotConfiguration(12, 168, -80);
		robotStateParams.floorHatchDelivery = robotStateParams.floorHatchStart.setElementArmAngle(-120);
		robotStateParams.floorHatchEnd = robotStateParams.floorHatchDelivery
				.setLiftHeight(robotStateParams.floorHatchDelivery.liftHeight + 15);

		robotStateParams.l1Hatch = new RobotConfiguration(0, 170, -77);

		robotStateParams.l2Hatch = new RobotConfiguration(57, 170, -100);
		robotStateParams.l2HatchFolded = robotStateParams.l2Hatch.setLiftArmAngle(168).setElementArmAngle(-105);
		robotStateParams.l3Hatch = new RobotConfiguration(45, 22, 60); // Lift height is a patch - previous height 40
																		// ElementArm angle: 90

		robotStateParams.l1Cargo = new RobotConfiguration(0, 168, -105);
		robotStateParams.l2CargoFromAbove = new RobotConfiguration(57, 167, -124);
		robotStateParams.l2CargoFromBelow = new RobotConfiguration(0, 10, 92);
		robotStateParams.l3Cargo = new RobotConfiguration(57, 7, 80); // was 60

		robotStateParams.shipCargoFromAbove = new RobotConfiguration(57, 163, -77);
		robotStateParams.shipCargoFromBelow = new RobotConfiguration(0, 40, 90);

		robotStateParams.abortCargoFloorCollect = robotStateParams.foldedWithCargo;

		// camera params
		cameraParams.unflippedDirection = Direction.BACKWARD;
		cameraParams.servoOffsetAngle = -5.0;
		cameraParams.negateServoAngles = true;
	}
}