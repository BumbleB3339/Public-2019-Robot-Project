package frc.robot.vision;

import static frc.robot.Robot.m_pixyVision;
import static frc.robot.profiles.InitProfiles.ROBOT_PROFILE;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.bumblelib.util.StableBoolean;
import frc.bumblelib.util.maneuvers.PIDManeuver;
import frc.robot.OI;
import frc.robot.Robot;
import frc.robot.RobotGameState.Direction;
import frc.robot.RobotGameState.FieldObjective;
import frc.robot.RobotGameState.RobotAction;
import frc.robot.RobotGameState.RocketPlacementHeight;
import frc.robot.RobotGameStateManager;
import frc.robot.autonomous.commands.CommandAutoAlignToTarget;
import frc.robot.operation.commands.CommandPulsedLED;
import frc.robot.operation.commands.CommandPulsedLED.PulseType;
import frc.robot.subsystems.Drivetrain.Gear;
import jaci.pathfinder.Pathfinder;

/**
 * Main class for the control side of automatic aligning to target.
 */
public class AlignToTarget implements PIDManeuver {
    private double minSlowdownPower = ROBOT_PROFILE.alignToTargetParams.minSlowdownPower;
    private double maxSlowdownPower = ROBOT_PROFILE.alignToTargetParams.maxSlowdownPower;
    private double proximityPower = ROBOT_PROFILE.alignToTargetParams.proximityPower;
    private double alignOnTargetWallPower = ROBOT_PROFILE.alignToTargetParams.alignOnTargetWallPower;
    private double p_offsetAngle_X = ROBOT_PROFILE.alignToTargetParams.p_offsetAngle_X;
    private double p_xDisplacement_X = ROBOT_PROFILE.alignToTargetParams.p_xDisplacement_X;
    private double p_offsetAngle_Chase = ROBOT_PROFILE.alignToTargetParams.p_offsetAngle_Chase;
    private double maxRotationPower_Chase = ROBOT_PROFILE.alignToTargetParams.maxRotationPower_Chase;
    private double rotationBasePower = ROBOT_PROFILE.alignToTargetParams.rotationBasePower;
    private double maxRotationPower_General = ROBOT_PROFILE.alignToTargetParams.maxRotationPower_General;

    private final double OFFSET_ANGLE_LIMIT = 20.0;
    private final double MIN_X_DISPLACEMENT_ANGLE_TO_CHASE = 5.0;
    private final double MIN_SLOWDOWN_DISTANCE = 0.7;
    private final double MAX_SLOWDOWN_DISTANCE = 1.0;
    private final double ANGLE_LIMIT_CORRECTION_POWER = 0.35;
    private final double MIN_ANGLE_TO_X_MINIMIZATION = 30.0;
    private final double MIN_Y_DISTANCE_TO_X_MINIMIZATION = 1.2;
    private final double proximityModeDistance = 0.65;
    public static final double MIN_DISTANCE_TO_ALLOW_ONE_TARGET = 0.0;
    private final double MIN_OFFSET_ANGLE_TO_CHASE = 15.0;
    private final double MIN_OFFSET_ANGLE_TO_CHASE_CARGOSHIP = 10.0;
    private final double LEVEL3_MAX_POWER_COEFFICIENT = 0.8;
    private boolean isAutoHatchCollectionActive = true;

    private StableBoolean proximityMode = new StableBoolean(4, 1);

    private boolean isLevel3 = false;

    private Gear prevGear;

    private double rotationPower = 0.0, yPower = 0.0;

    private double proximityModeTimeStamp = -3339.0;

    private enum AlignmentPhase {
        TARGET_DETECTION, METHOD_DECISION, X_MINIMIZATION, CHASE, PROXIMITY, FORCE_ALIGNMENT, HATCH_COLLECTION,
        DRIVEAWAY;
    }

    private AlignmentPhase alignmentPhase;

    public AlignToTarget() {
    }

    public void initSmartDashboardControls() {
        SmartDashboard.putNumber("AlignToTarget/Params/minSlowdownPower", minSlowdownPower);
        SmartDashboard.putNumber("AlignToTarget/Params/maxSlowdownPower", maxSlowdownPower);
        SmartDashboard.putNumber("AlignToTarget/Params/proximityPower", proximityPower);
        SmartDashboard.putNumber("AlignToTarget/Params/alignOnTargetWallPower", alignOnTargetWallPower);
        SmartDashboard.putNumber("AlignToTarget/Params/p_offsetAngle_X", p_offsetAngle_X);
        SmartDashboard.putNumber("AlignToTarget/Params/p_xDisplacement_X", p_xDisplacement_X);
        SmartDashboard.putNumber("AlignToTarget/Params/p_offsetAngle_Chase", p_offsetAngle_Chase);
        SmartDashboard.putNumber("AlignToTarget/Params/maxRotationPower_Chase", maxRotationPower_Chase);
        SmartDashboard.putNumber("AlignToTarget/Params/rotationBasePower", rotationBasePower);
        SmartDashboard.putNumber("AlignToTarget/Params/maxRotationPower_General", maxRotationPower_General);
        SmartDashboard.putBoolean("AlignToTarget/Params/isAutoHatchCollectionActive", isAutoHatchCollectionActive);
    }

    /**
     * This method ignores the setpoint
     */
    @Override
    public void init(double setpoint) {
        prevGear = Robot.m_drivetrain.getGear();
        Robot.m_drivetrain.setGear(Gear.POWER_GEAR);
        m_pixyVision.LED.set(true);
        m_pixyVision.setEnabled(true);
        proximityMode.forceValue(false);
        proximityModeTimeStamp = -3339.0;
        hatchCollectionTimestamp = -3339;
        alignmentPhase = AlignmentPhase.TARGET_DETECTION;
        if (RobotGameStateManager.currentGameState.fieldObjective == FieldObjective.ROCKET
                && RobotGameStateManager.currentGameState.rocketPlacementHeight == RocketPlacementHeight.LEVEL3) {
            maxSlowdownPower = ROBOT_PROFILE.alignToTargetParams.maxSlowdownPower * LEVEL3_MAX_POWER_COEFFICIENT;
            proximityPower = ROBOT_PROFILE.alignToTargetParams.proximityPower * LEVEL3_MAX_POWER_COEFFICIENT;
            isLevel3 = true;
        } else {
            isLevel3 = false;
        }

        initSmartDashboardControls();
    }

    public double getYCoordinate() {
        if (m_pixyVision.areAnglesValid()) {
            return m_pixyVision.getYDisplacement();
        } else {
            return -3339.0;
        }
    }

    @Override
    public void execute() {
        isAutoHatchCollectionActive = SmartDashboard.getBoolean("AlignToTarget/Params/isAutoHatchCollectionActive",
                false);

        yPower = 0.0;
        rotationPower = 0.0;
        switch (alignmentPhase) {
        case TARGET_DETECTION:
            executeTargetDetection();
            break;

        case METHOD_DECISION:
            executeMethodDecision();
            break;

        case X_MINIMIZATION:
            executeXMinimization();
            break;

        case CHASE:
            executeChase();
            break;

        case PROXIMITY:
            executeProximity();
            break;

        case FORCE_ALIGNMENT:
            executeForceAlignment();
            break;

        case HATCH_COLLECTION:
            executeHatchCollection();
            break;

        case DRIVEAWAY:
            executeDriveaway();
            break;

        default:
            break;
        }
        // System.out.println("Alignment Phase: " + alignmentPhase.toString());

        // limit rotation power
        if (Math.abs(rotationPower) > maxRotationPower_General) {
            rotationPower = Math.copySign(maxRotationPower_General, rotationPower);
        }

        SmartDashboard.putNumber("AlignToTarget/Current yPower", yPower);
        SmartDashboard.putBoolean("AlignToTarget/Current proximityMode", proximityMode.get());
        SmartDashboard.putNumber("AlignToTarget/current rotationPower", rotationPower);
        SmartDashboard.putString("AlignToTarget/current alignmentPhase", alignmentPhase.toString());
        Robot.m_drivetrain.bumbleDrive.arcadeDrive(rotationPower,
                RobotGameStateManager.nextGameState.direction == Direction.FORWARD ? yPower : -yPower);
    }

    public boolean isInProximityMode() {
        return alignmentPhase == AlignmentPhase.PROXIMITY;
    }

    private void turnToTarget() {
        if (RobotGameStateManager.nextGameState.direction == Direction.FORWARD) {
            rotationPower = -p_offsetAngle_Chase * calculateGyroOffsetAngle();
        } else {
            rotationPower = -p_offsetAngle_Chase * Pathfinder.boundHalfDegrees(calculateGyroOffsetAngle() + 180.0);
        }

        yPower = 0;
        OI.driverController.setRumble(RumbleType.kLeftRumble, 1.0);
    }

    private void executeTargetDetection() {
        if (m_pixyVision.areAnglesValid()) {
            alignmentPhase = AlignmentPhase.METHOD_DECISION;
        } else {
            turnToTarget();
        }
    }

    private double cargoshipAngleToChase = 0.0;

    private void executeMethodDecision() {
        if (RobotGameStateManager.currentGameState.robotAction == RobotAction.FEEDER_COLLECT
                || Math.abs(m_pixyVision.getAngleToTarget()) > MIN_ANGLE_TO_X_MINIMIZATION
                        && (RobotGameStateManager.currentGameState.fieldObjective == FieldObjective.CARGOSHIP
                                && RobotGameStateManager.currentGameState.robotAction != RobotAction.FEEDER_COLLECT && !DriverStation.getInstance().isAutonomous())) {
            alignmentPhase = AlignmentPhase.X_MINIMIZATION;
        } else {
            alignmentPhase = AlignmentPhase.CHASE;
            cargoshipAngleToChase = Pathfinder.boundHalfDegrees(
                    (m_pixyVision.getOffsetAngleFromTarget() + Robot.m_drivetrain.getGyro().getYaw()));
        }
    }

    private void executeXMinimization() {
        if (!m_pixyVision.areAnglesValid()) {
            turnToTarget();
            return;
        } else {
            OI.driverController.setRumble(RumbleType.kLeftRumble, 0.0);
        }

        double offsetAngleFromTarget = m_pixyVision.getOffsetAngleFromTarget();
        double xDisplacement = m_pixyVision.getXDisplacement();
        double yDisplacemant = m_pixyVision.getYDisplacement();

        rotationPower = -p_xDisplacement_X * xDisplacement;
        if (Math.abs(offsetAngleFromTarget) > OFFSET_ANGLE_LIMIT) {
            rotationPower = -ANGLE_LIMIT_CORRECTION_POWER * Math.signum(offsetAngleFromTarget);
        }
        yPower = ((maxSlowdownPower + minSlowdownPower) / 2.0) * 1.3339;

        if (Math.abs(m_pixyVision.getAngleToTarget()) < MIN_X_DISPLACEMENT_ANGLE_TO_CHASE
                || yDisplacemant < MIN_Y_DISTANCE_TO_X_MINIMIZATION) {
            alignmentPhase = AlignmentPhase.CHASE;
        }
    }

    private void executeChase() {
        if (!m_pixyVision.areAnglesValid()) {
            turnToTarget();
            return;
        } else {
            OI.driverController.setRumble(RumbleType.kLeftRumble, 0.0);
        }

        double yDisplacemant = m_pixyVision.getYDisplacement();
        double distance = m_pixyVision.getDistance();
        double offsetAngleFromTarget = m_pixyVision.getOffsetAngleFromTarget();
        double minOffsetAngleToChase = MIN_OFFSET_ANGLE_TO_CHASE;

        if (RobotGameStateManager.currentGameState.fieldObjective == FieldObjective.CARGOSHIP
                && RobotGameStateManager.currentGameState.robotAction != RobotAction.FEEDER_COLLECT && !DriverStation.getInstance().isAutonomous()) {
            offsetAngleFromTarget = Pathfinder.boundHalfDegrees(cargoshipAngleToChase - Robot.m_drivetrain.getGyro().getYaw());
            minOffsetAngleToChase = MIN_OFFSET_ANGLE_TO_CHASE_CARGOSHIP;
        }

        rotationPower = -p_offsetAngle_Chase * offsetAngleFromTarget;
        rotationPower += Math.copySign(rotationBasePower, rotationPower);

        if (Math.abs(rotationPower) > maxRotationPower_Chase) {
            rotationPower = Math.copySign(maxRotationPower_Chase, rotationPower);
        }

        if (Math.abs(offsetAngleFromTarget) < minOffsetAngleToChase
                && !m_pixyVision.reflectiveTargetProcessor.targetProvider.isTargetSingleReflective()) {
            yPower = getDrivePower(distance);
            // if (RobotGameStateManager.currentGameState.robotAction != RobotAction.FEEDER_COLLECT) {
            //     yPower *= 0.7;
            // }
        } else {
            yPower = 0.0;
        }

        proximityMode.update(yDisplacemant < proximityModeDistance);

        if (proximityMode.get()) {
            proximityModeTimeStamp = Timer.getFPGATimestamp();
            alignmentPhase = AlignmentPhase.PROXIMITY;
        }
    }

    private StableBoolean feederDistanceBoolean = new StableBoolean(4, 1);

    private void executeProximity() {
        rotationPower = 0.0;
        yPower = proximityPower;
        if (DriverStation.getInstance().isAutonomous()
                && RobotGameStateManager.currentGameState.robotAction == RobotAction.FEEDER_COLLECT) {
            yPower *= 1.0;
        } else {
            feederDistanceBoolean.update(
                    getYCoordinate() != -3339 && getYCoordinate() < CommandAutoAlignToTarget.MAX_Y_TO_COLLECT_HATCH);
            if (RobotGameStateManager.currentGameState.robotAction == RobotAction.FEEDER_COLLECT
                    && (feederDistanceBoolean.get() || Robot.m_drivetrain.isDrivetrainStopped() || Robot.m_drivetrain
                            .getAverageCurrent() > CommandAutoAlignToTarget.MIN_CURRENT_FOR_HATCH_COLLECT)) {
                alignmentPhase = AlignmentPhase.FORCE_ALIGNMENT;
            }
        }
    }

    private final double MAX_ANGLE_TO_COLLECT_HATCH = 6.0;

    private void executeForceAlignment() {
        rotationPower = 0.0;
        yPower = alignOnTargetWallPower;
        if (Math.abs(Pathfinder.boundHalfDegrees(Robot.m_drivetrain.getGyro().getYaw()
                - (RobotGameStateManager.nextGameState.direction == Direction.FORWARD ? 180
                        : 0))) < MAX_ANGLE_TO_COLLECT_HATCH
                && isAutoHatchCollectionActive) {
            alignmentPhase = AlignmentPhase.HATCH_COLLECTION;
        }
    }

    private double hatchCollectionTimestamp = -3339;
    private final double HATCH_COLLECTION_TIMEOUT = 0.4;
    private CommandPulsedLED ledCommand = new CommandPulsedLED(PulseType.QUICK, 1.0);

    private void executeHatchCollection() {
        yPower = 0.0;
        rotationPower = 0.0;
        if (hatchCollectionTimestamp == -3339) {
            hatchCollectionTimestamp = Timer.getFPGATimestamp();
            Robot.m_hatchHandler.extendHoldSolenoid();
        }
        if (Timer.getFPGATimestamp() - hatchCollectionTimestamp > HATCH_COLLECTION_TIMEOUT) {
            alignmentPhase = AlignmentPhase.DRIVEAWAY;
            ledCommand.start();
        }
    }

    private void executeDriveaway() {
        rotationPower = 0.0;
        yPower = -0.7;
    }

    private double calculateGyroOffsetAngle() {
        return Pathfinder.boundHalfDegrees(
                m_pixyVision.getAlignmentTargetAbsoluteAngle() - Robot.m_drivetrain.getGyro().getYaw());
    }

    private double getDrivePower(double distance) {
        if (distance > MAX_SLOWDOWN_DISTANCE) {
            return maxSlowdownPower;
        } else if (distance < MIN_SLOWDOWN_DISTANCE) {
            return minSlowdownPower;
        } else {
            return minSlowdownPower
                    + ((distance - MIN_SLOWDOWN_DISTANCE) / (MAX_SLOWDOWN_DISTANCE - MIN_SLOWDOWN_DISTANCE))
                            * (maxSlowdownPower - minSlowdownPower);
        }
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end() {
        Robot.m_drivetrain.stopMotors();
        Robot.m_drivetrain.bumbleDrive.feed();
        Robot.m_drivetrain.setGear(prevGear);
        OI.driverController.setRumble(RumbleType.kLeftRumble, 0.0);
        m_pixyVision.setEnabled(false);
        m_pixyVision.LED.set(false);
        if (isLevel3) {
            maxSlowdownPower = ROBOT_PROFILE.alignToTargetParams.maxSlowdownPower;
        }
    }

    @Override
    public void initCalibration() {
        init(0.0);
    }

    /**
     * @return the proximityModeTimeStamp
     */
    public double getProximityModeTimeStamp() {
        return proximityModeTimeStamp;
    }

    @Override
    public void executeCalibration() {
        minSlowdownPower = SmartDashboard.getNumber("AlignToTarget/Params/minSlowdownPower", minSlowdownPower);
        maxSlowdownPower = SmartDashboard.getNumber("AlignToTarget/Params/maxSlowdownPower", maxSlowdownPower);
        proximityPower = SmartDashboard.getNumber("AlignToTarget/Params/proximityPower", proximityPower);
        alignOnTargetWallPower = SmartDashboard.getNumber("AlignToTarget/Params/alignOnTargetWallPower",
                alignOnTargetWallPower);
        p_offsetAngle_X = SmartDashboard.getNumber("AlignToTarget/Params/p_offsetAngle_X", p_offsetAngle_X);
        p_xDisplacement_X = SmartDashboard.getNumber("AlignToTarget/Params/p_xDisplacement_X", p_xDisplacement_X);
        p_offsetAngle_Chase = SmartDashboard.getNumber("AlignToTarget/Params/p_offsetAngle_Chase", p_offsetAngle_Chase);
        maxRotationPower_Chase = SmartDashboard.getNumber("AlignToTarget/Params/maxRotationPower_Chase",
                maxRotationPower_Chase);
        rotationBasePower = SmartDashboard.getNumber("AlignToTarget/Params/rotationBasePower", rotationBasePower);
        maxRotationPower_General = SmartDashboard.getNumber("AlignToTarget/Params/maxRotationPower_General",
                maxRotationPower_General);
        execute();
    }
}
