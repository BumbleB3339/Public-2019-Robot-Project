package frc.robot.profiles.robot_profiles;

import frc.bumblelib.bumblelib_autonomous.pathing.RobotReferencePoint;
import frc.bumblelib.util.PIDPreset;
import frc.bumblelib.util.Units;
import frc.robot.RobotConfiguration;
import frc.robot.RobotGameState.Direction;
import frc.robot.profiles.RobotProfile;

public class RobotA extends RobotProfile {

	public RobotA() {
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
				* gear_output_to_encoder_reduction) * 1.01);

		autonomousParams.wheel_base_effective_width = 0.667; // calibrated white night

		// Pathfinder Params
		pathfinderParams.maxJerk = 60.0;
		pathfinderParams.deltaTime = 0.02;

		// Speed Gear Values
		pathfinderParams.speedGear.maxVel = 4.0; // Calibrated
		pathfinderParams.speedGear.maxAccel = 2.5;
		pathfinderParams.speedGear.kV = 0.2191; // calibrated
		pathfinderParams.speedGear.kA = 0.041; // Calibrated
		pathfinderParams.speedGear.vIntercept = 0.0577; // calibrated
		pathfinderParams.speedGear.gyroP = 0.01; // calibrated

		pathfinderParams.speedGear.fastPathPreset = new PIDPreset(0.5, 0.0, 0.01); // calibrated
		pathfinderParams.speedGear.slowPathPreset = new PIDPreset(0.5, 0.0, 0.01); // calibrated

		pathfinderParams.speedGear.rotMaxVel = 3.84;
		pathfinderParams.speedGear.rotGyroP = 0.0;
		pathfinderParams.speedGear.rotMaxAccel = 3.0;
		pathfinderParams.speedGear.rotKv = 3.84;
		pathfinderParams.speedGear.rotKa = 0.0;
		pathfinderParams.speedGear.rotVIntercept = 0.1;

		// Power Gear Values
		pathfinderParams.powerGear.maxVel = 2.0; // Calibrated
		pathfinderParams.powerGear.maxAccel = 2.5;
		pathfinderParams.powerGear.kV = 0.4567; // Calibrated
		pathfinderParams.powerGear.kA = 0.0202; // Calibrated
		pathfinderParams.powerGear.vIntercept = 0.0617; // Calibrated
		pathfinderParams.powerGear.gyroP = 0.01; // Calibrated

		pathfinderParams.powerGear.fastPathPreset = new PIDPreset(0.4, 0.0, 0.05); // Calibrated
		pathfinderParams.powerGear.slowPathPreset = new PIDPreset(0.4, 0.0, 0.05); // Calibrated

		pathfinderParams.powerGear.rotMaxVel = 1.5; // Calibrated
		pathfinderParams.powerGear.rotGyroP = 0.005;
		pathfinderParams.powerGear.rotMaxAccel = 2.0; // Calibrated
		pathfinderParams.powerGear.rotKv = 0.4692; // Calibrated
		pathfinderParams.powerGear.rotKa = 0.0301; // Calibrated
		pathfinderParams.powerGear.rotVIntercept = 0.0665; // Calibrated

		// Robot Reference Points
		robotReferencePoints.jaciCenterOfRotation = new RobotReferencePoint(autonomousParams.robot_width / 2.0,
				autonomousParams.robot_length / 2.0);
		robotReferencePoints.realCenterOfRotation = new RobotReferencePoint(autonomousParams.robot_width / 2.0,
				autonomousParams.robot_length / 2.0);

		robotReferencePoints.frontCenter = new RobotReferencePoint(autonomousParams.robot_width / 2.0,
				autonomousParams.robot_length);
		robotReferencePoints.rearCenter = new RobotReferencePoint(autonomousParams.robot_width / 2.0, 0.0);

		robotReferencePoints.frontPixy = new RobotReferencePoint(autonomousParams.robot_width / 2.0 + 0.2225,
				autonomousParams.robot_length / 2.0);

		robotReferencePoints.rearPixy = new RobotReferencePoint(autonomousParams.robot_width / 2.0 - 0.2225,
				autonomousParams.robot_length / 2.0);

		robotReferencePoints.elementArmCenter = new RobotReferencePoint(autonomousParams.robot_width / 2.0 - 1.5, // -1.5
																													// because
																													// the
																													// lift
																													// is
																													// a
																													// bit
																													// off
																													// centered
				autonomousParams.robot_length / 2.0);

		// Pixy params
		pixyParams.ServoAngleInZeroDegrees = -3;
		pixyParams.pixyUID = 0x02A69555;
		//pixyParams.pixyUID = 0xF71994B7;
		pixyParams.sensorAngle = 17.0;
		pixyParams.edgeServoAngle = 135.0;
		pixyParams.maxServoPWM = 2.5;
		pixyParams.minServoPWM = .5;
		pixyParams.forwardAngle = 75;
		pixyParams.backwardAngle = -75;
		pixyParams.elementArmXOffsetForward = -0.03;
		pixyParams.elementArmXOffsetBackward = 0.04;

		// Align to target
		alignToTargetParams.minSlowdownPower = 0.33;
		alignToTargetParams.maxSlowdownPower = 0.6;
		alignToTargetParams.proximityPower = 0.33;
		alignToTargetParams.alignOnTargetWallPower = 0.44;
		alignToTargetParams.p_offsetAngle_X = 0.025;
		alignToTargetParams.p_xDisplacement_X = 0.9;
		alignToTargetParams.p_offsetAngle_Chase = 0.02;
		alignToTargetParams.maxRotationPower_Chase = 0.44;
		alignToTargetParams.rotationBasePower = 0.22;
		alignToTargetParams.maxRotationPower_General = 0.66;

		// Drivetrain Params
		drivetrainParams.isPCMConnected = true;
		drivetrainParams.enableCompressor = true;
		drivetrainParams.isRightSideInverted = false;
		drivetrainParams.isLeftSideInverted = true;
		drivetrainParams.leftEncoderSensorPhase = true;
		drivetrainParams.rightEncoderSensorPhase = false;
		drivetrainParams.isVoltageCompensationEnabled = true;
		drivetrainParams.smartCurrentLimit = 40; //// 0.0 to disable
		drivetrainParams.nominalVoltage = 11;
		drivetrainParams.rampRate = 0.2; // 0.0 to disable

		bumbleDifferentialDriveParams.TRXQuickTurnCoefficient = 0.5;
		bumbleDifferentialDriveParams.slowTurnPower = 0.15;

		// IntakeArm Params
		intakeArmParams.isMotorInverted = false;
		intakeArmParams.angleToBalanced = 27;
		intakeArmParams.voltageInAngle0 = 0.529;
		intakeArmParams.voltageInAngle90 = 0.657;
		intakeArmParams.pidPreset = new PIDPreset(0.025, 0.0, 0.01);
		intakeArmParams.fPower = 0.05;
		intakeArmParams.frictionOvercomePower = 0.05;
		intakeArmParams.liftRobotPower = 0.8;
		intakeArmParams.supportRobotPower = 0.3339;
		intakeArmParams.releaseRobotPower = -0.2;
		intakeArmParams.foldedAngle = -28;
		intakeArmParams.deliverHatchPanelAngle = 45;
		intakeArmParams.collectCargoAngle = 98;
		intakeArmParams.collectHatchAngle = 156;
		intakeArmParams.prepareToClimbAngle = 27.0;
		intakeArmParams.holdRobotAngle = 150.0;

		// LiftArmParams
		liftArmParams.isMotorInverted = true;
		liftArmParams.voltageInAngle0 = 0.421;
		liftArmParams.voltageInAngle90 = 0.298; // -90 angle voltage 0.606
		liftArmParams.pidPreset_none = new PIDPreset(0.018, 0.0, 0.02);
		liftArmParams.pidPreset_cargo = new PIDPreset(0.018, 0.0, 0.02);
		liftArmParams.pidPreset_hatch = new PIDPreset(0.018, 0.0, 0.03);
		liftArmParams.gravityF_0_none = 0.14;
		liftArmParams.gravityF_90_none = 0.11;
		liftArmParams.gravityF_0_cargo = 0.14;
		liftArmParams.gravityF_90_cargo = 0.11;
		liftArmParams.gravityF_0_hatchPanel = 0.20;
		liftArmParams.gravityF_90_hatchPanel = 0.18;
		liftArmParams.frictionOvercomePower = 0.04;
		liftArmParams.rampRate = 0.1;
		liftArmParams.maxOutputUpStrong = 0.6;
		liftArmParams.angleFreedom = 3.0;
		liftArmParams.maxOutputDownStrong = 0.5;

		// ElementArm Params
		elementArmParams.isMotorInverted = false;
		elementArmParams.voltageInAngleMinus90 = 0.704;
		elementArmParams.voltageInAngle90 = 0.362; // 0.171 difference
		elementArmParams.pidPreset_none = new PIDPreset(0.04, 0.0, 0.0);
		elementArmParams.pidPreset_cargo = new PIDPreset(0.04, 0.0, 0.0);
		elementArmParams.pidPreset_hatch = new PIDPreset(0.04, 0.0, 0.0);
		elementArmParams.outputRange = 0.7;
		elementArmParams.gravityF_none = 0.0;
		elementArmParams.gravityF_hatchPanel = 0.0;
		elementArmParams.gravityF_cargo = 0.0;
		elementArmParams.frictionOvercomePower = 0.06;
		elementArmParams.gravityF_Ejecting = 0.0;
		elementArmParams.angleFreedom = 16.0;

		// CargoHandler Params
		cargoHandlerParams.isMotorInverted = false;

		// Lift Params
		liftParams.isMotorInverted = true;
		liftParams.gravityCompensationPower = 0.03339;
		liftParams.manualPowerDown = -0.4;
		liftParams.manualPowerUp = 0.5;
		liftParams.downMovementPIDPreset = new PIDPreset(0.1, 0.0, 0.07);
		liftParams.upMovementPIDPreset = new PIDPreset(0.1, 0.0, 0.05);
		liftParams.sensorPhase = true;
		liftParams.isBottomSwitchDefective = false;
		liftParams.bottomSwitchNormallyClosed = true;
		liftParams.positionToleranceHeight = 3.0;

		// Climb Params
		climbParams.isBackLeftMotorInverted = true;
		climbParams.isBackRightMotorInverted = false;
		climbParams.isFrontLeftMotorInverted = false;
		climbParams.isFrontRightMotorInverted = true;

		climbParams.frontLeftPotentiometerUpperVoltage = 0.571; // 0 height is the upper voltage
		climbParams.frontLeftPotentiometerLowerVoltage = -0.093;
		climbParams.frontLeftPotentiometerUpperValue = 0.0;
		climbParams.frontLeftPotentiometerLowerValue = 52.0;

		climbParams.frontRightPotentiometerUpperVoltage = 0.431; // 0 height is the upper voltage
		climbParams.frontRightPotentiometerLowerVoltage = 1.094;
		climbParams.frontRightPotentiometerUpperValue = 0.0;
		climbParams.frontRightPotentiometerLowerValue = 52.0;

		climbParams.rearLeftPotentiometerUpperVoltage = 0.258; // 0 height is the upper voltage
		climbParams.rearLeftPotentiometerLowerVoltage = 0.921;
		climbParams.rearLeftPotentiometerUpperValue = 0.0;
		climbParams.rearLeftPotentiometerLowerValue = 52.0;

		climbParams.rearRightPotentiometerUpperVoltage = 0.737; // 0 height is the upper voltage
		climbParams.rearRightPotentiometerLowerVoltage = 0.076;
		climbParams.rearRightPotentiometerUpperValue = 0.0;
		climbParams.rearRightPotentiometerLowerValue = 52.0;

		climbParams.foldedHeight = -3.5;

		// HatchHandler Params
		hatchHandlerParams.timeToFoldHolderAfterPushExtend = 0.04;

		// FloorIntake Params
		floorRollerParams.isMotorInverted = false;
		floorRollerParams.collectHatchPanelPower = -0.5;
		floorRollerParams.collectCargoPower = 1.0;
		floorRollerParams.deliverHatchPanelPower = 1.0;
		floorRollerParams.releaseHatchPanelPower = 1.0;
		floorRollerParams.dragPower = 0.508;

		robotStateParams.folded = new RobotConfiguration(0, 170, -90);
		robotStateParams.foldedWithCargo = new RobotConfiguration(14, 170, -112);

		robotStateParams.climbLevel3 = robotStateParams.folded;
		robotStateParams.climbLevel2 = robotStateParams.folded.setLiftHeight(10.0);
		robotStateParams.fromAbove = robotStateParams.folded
				.setElementArmAngle(90 - robotStateParams.folded.liftArmAngle);

		robotStateParams.frontIntakeArmGapLow = robotStateParams.foldedWithCargo;
		robotStateParams.frontIntakeArmGapHigh = robotStateParams.foldedWithCargo
				.setLiftHeight(robotStateParams.foldedWithCargo.liftHeight + 10.0);

		robotStateParams.feederHatchStart = robotStateParams.folded.setElementArmAngle(-76);
		robotStateParams.feederHatchEnd = robotStateParams.folded.setElementArmAngle(-100);

		robotStateParams.feederCargoFromAbove = new RobotConfiguration(38, 165, -95);
		robotStateParams.feederCargoFromBelow = new RobotConfiguration(0, 74, 15);

		robotStateParams.floorCargo = new RobotConfiguration(4, 170, -48);

		robotStateParams.floorHatchStart = new RobotConfiguration(12, 170, -85);
		robotStateParams.floorHatchDelivery = robotStateParams.floorHatchStart.setElementArmAngle(-120);
		robotStateParams.floorHatchEnd = robotStateParams.floorHatchDelivery
				.setLiftHeight(robotStateParams.floorHatchDelivery.liftHeight + 15);

		robotStateParams.l1Hatch = new RobotConfiguration(0, 165, -67);


		// robotStateParams.l2Hatch = new RobotConfiguration(57, 156, -80); // was 65
		robotStateParams.l2Hatch = new RobotConfiguration(57, 160, -87);  
		robotStateParams.l2HatchFolded = robotStateParams.l2Hatch.setLiftArmAngle(160).setElementArmAngle(-85);
		robotStateParams.l3Hatch = new RobotConfiguration(43, 8, 70); // Lift height is a patch - previous height 40
																		// ElementArm angle: 90

		robotStateParams.l1Cargo = new RobotConfiguration(0, 170, -105);
		robotStateParams.l2CargoFromAbove = new RobotConfiguration(57, 152, -108);
		robotStateParams.l2CargoFromBelow = new RobotConfiguration(0, 10, 89);
		robotStateParams.l3Cargo = new RobotConfiguration(57, 7, 70); // was 60

		robotStateParams.shipCargoFromAbove = new RobotConfiguration(57, 163, -77);
		robotStateParams.shipCargoFromBelow = new RobotConfiguration(0, 40, 90);

		robotStateParams.abortCargoFloorCollect = robotStateParams.floorCargo.setLiftArmAngle(140)
				.setElementArmAngle(-50);

		// camera params
		cameraParams.unflippedDirection = Direction.FORWARD;
		cameraParams.servoOffsetAngle = -10.0;
		cameraParams.negateServoAngles = true;
	}
}
