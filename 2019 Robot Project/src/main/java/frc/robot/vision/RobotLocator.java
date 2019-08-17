/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.vision;

import static frc.robot.profiles.InitProfiles.ROBOT_PROFILE;

import java.util.ArrayList;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.bumblelib.bumblelib_autonomous.pathing.BumbleWaypoint;
import frc.bumblelib.util.Logger;
import frc.bumblelib.util.SmartdashboardDebugging;
import frc.bumblelib.util.hardware.Gyro;
import frc.bumblelib.vision.VisualTargetProcessor;
import frc.bumblelib.vision.VisualTargetProvider.TargetSelectionPolicy;
import frc.robot.RobotGameState;
import frc.robot.RobotGameState.Direction;
import frc.robot.RobotGameStateManager;
import frc.robot.vision.TargetBank.AlignmentTarget;
import jaci.pathfinder.Pathfinder;

/**
 * This class is responsible of finding the relative-to-vision-target location of the robot using data
 * provided from a visual target provider and the gyro.
 */
public class RobotLocator implements SmartdashboardDebugging, Logger {

    private VisualTargetProcessor visualTargetProcessor;
    private Gyro gyro;

    private AlignmentTarget alignmentTarget = AlignmentTarget.FEEDER;

    private double x; // Pixy x coordinate in target coordinates system
    private double y; // Pixy y coordinate in target coordinates system
    private double distance; // Pixy distance from target
    private double angleFromTarget; // Pixy angle from target, calculated using robot gyro
    private double targetAbsoluteAngle; // The absolute angle the robot gyro should show when facing the target
    private double normalizedYaw; // The current yaw of the robot // TODO: Add 180 when backward? // TODO: Delete
                                  // the previous TODO
    private double rotationOffsetAngle; // The angle in which a Pixy aligned with the robot should rotate in order to
                                        // center the target in
                                        // the view. Positive is when the Pixy should rotate counterclockwise
    private double robotYawInTargetCoordinates; // Robot yaw in target coordinates system
    private double estimatedAlignedAngle; // The angle that the gyro should see if the robot was alignd to the center of
                                          // the target
    private BumbleWaypoint robotWaypoint; // The waypoint(x, y and yaw) of the robot in target coordinates system

    public RobotLocator(VisualTargetProcessor visualTargetProcessor, Gyro gyro) {
        this.gyro = gyro;
        this.visualTargetProcessor = visualTargetProcessor;
    }

    /**
     * @return the angle that the gyro should see if the robot was alignd to the
     *         center of the target. This is done by substracting the offset angle
     *         (from the sensor to the center of the target) and the angle between
     *         the robot and the target from the robot gyro, thus "bringing" the
     *         robot in front of the target and perpendicular to it.
     */
    public double findEstimatedAlignedAngle() {
        double normalizedYaw = gyro.getYaw();
        if (RobotGameStateManager.nextGameState.direction == RobotGameState.Direction.BACKWARD) {
            normalizedYaw = Pathfinder.boundHalfDegrees(gyro.getYaw() + 180);
        }
        return Pathfinder.boundHalfDegrees(normalizedYaw); // Only use gyro
    }

    /**
     * @return
     */
    private void updateAlignmentTargetAndSetPolicy() {
        alignmentTarget = TargetBank.getAlignmentTarget(estimatedAlignedAngle);

        // TODO: for rocket we can use heighest for cargo and lowest for hatch panels
        switch (alignmentTarget) {
        case CLOSE_LEFT_ROCKET:
        case FAR_RIGHT_ROCKET:
            visualTargetProcessor.targetProvider.setTargetSelectionPolicy(TargetSelectionPolicy.BIGGEST);
            break;
        case FAR_LEFT_ROCKET:
        case CLOSE_RIGHT_ROCKET:
            visualTargetProcessor.targetProvider.setTargetSelectionPolicy(TargetSelectionPolicy.BIGGEST);
            break;
        case RIGHT_CARGOSHIP:
        case LEFT_CARGOSHIP:
        case CARGO_SHIP_FRONT:
            visualTargetProcessor.targetProvider.setTargetSelectionPolicy(TargetSelectionPolicy.BIGGEST);
            break;
        case FEEDER:
            visualTargetProcessor.targetProvider.setTargetSelectionPolicy(TargetSelectionPolicy.BIGGEST);
            break;
        case RIGHT_ROCKET_CARGO:
        case LEFT_ROCKET_CARGO:
            visualTargetProcessor.targetProvider.setTargetSelectionPolicy(TargetSelectionPolicy.HIGHEST);
            break;
        }
    }

    public static double bound90Degrees(double angle_degrees) {
        while (angle_degrees >= 90.0)
            angle_degrees -= 180.0;
        while (angle_degrees < -90.0)
            angle_degrees += 180.0;
        return angle_degrees;
    }

    private BumbleWaypoint findPixyWaypointInTargetCoordinates() {
        normalizedYaw = gyro.getYaw();

        targetAbsoluteAngle = alignmentTarget.getAbsoluteAngle();
        if (RobotGameStateManager.nextGameState.direction == Direction.BACKWARD) {
            targetAbsoluteAngle = Pathfinder.boundHalfDegrees(targetAbsoluteAngle + 180);
        }

        distance = visualTargetProcessor.getDistanceToTarget();
        if (distance == VisualTargetProcessor.INVALID_VALUE) {
            return null;
        }

        robotYawInTargetCoordinates = Pathfinder.boundHalfDegrees(normalizedYaw - targetAbsoluteAngle - 180);

        rotationOffsetAngle = visualTargetProcessor.getRotationOffsetAngle();
        angleFromTarget = bound90Degrees(robotYawInTargetCoordinates + rotationOffsetAngle);
        x = (-1) * distance * Math.sin(Math.toRadians(angleFromTarget));
        y = distance * Math.cos(Math.toRadians(angleFromTarget));

        return new BumbleWaypoint(x, y, robotYawInTargetCoordinates,
                visualTargetProcessor.targetProvider.getSensorRobotReferncePoint());
    }

    private void updateRobotWaypointInTargetCoordinates() {
        robotWaypoint = findPixyWaypointInTargetCoordinates();
        if (robotWaypoint != null) {
            robotWaypoint.transformRobotReferencePoint(ROBOT_PROFILE.robotReferencePoints.realCenterOfRotation);
        }
    }

    /**
     * @return the robotWaypoint
     */
    public BumbleWaypoint getRobotWaypoint() {
        return robotWaypoint;
    }

    public double getTargetAbsoluteAngle() {
        return targetAbsoluteAngle;
    }

    /**
     * @return the robotYawInTargetCoordinates
     */
    public double getRobotYawInTargetCoordinates() {
        return robotYawInTargetCoordinates;
    }

    public double getAlignmentTargetAbsoluteAngle() {
        return alignmentTarget.getAbsoluteAngle();
    }

    public void update(boolean hasTargetInView) {
        estimatedAlignedAngle = findEstimatedAlignedAngle();
        updateAlignmentTargetAndSetPolicy();
        if (hasTargetInView) {
            updateRobotWaypointInTargetCoordinates();
        }
    }

    @Override
    public void sendDebuggingData() {
        SmartDashboard.putNumber("RobotLocator/Pixy x", x);
        SmartDashboard.putNumber("RobotLocator/Pixy y", y);
        SmartDashboard.putNumber("RobotLocator/Pixy distance", distance);
        SmartDashboard.putNumber("RobotLocator/Pixy angleFromTarget", angleFromTarget);
        SmartDashboard.putNumber("RobotLocator/Pixy rotationOffsetAngle", rotationOffsetAngle);
        SmartDashboard.putNumber("RobotLocator/targetAbsoluteAngle", targetAbsoluteAngle);
        SmartDashboard.putNumber("RobotLocator/normalizedYaw", normalizedYaw);
        SmartDashboard.putNumber("RobotLocator/robotAngleInTargetCoordinates", robotYawInTargetCoordinates);
        SmartDashboard.putString("RobotLocator/alignmentTarget",
                (alignmentTarget != null) ? alignmentTarget.toString() : "");
    }

    @Override
    public ArrayList<String> getLogData() {
        ArrayList<String> output = new ArrayList<String>();
        output.add(Double.toString(x));
        output.add(Double.toString(y));
        output.add(Double.toString(distance));
        output.add(Double.toString(angleFromTarget));
        output.add(Double.toString(rotationOffsetAngle));
        output.add(Double.toString(targetAbsoluteAngle));
        output.add(Double.toString(normalizedYaw));
        output.add(Double.toString(robotYawInTargetCoordinates));
        output.add((alignmentTarget != null) ? alignmentTarget.toString() : "");

        return output;
    }

    @Override
    public ArrayList<String> getLogHeaders() {
        ArrayList<String> output = new ArrayList<String>();

        output.add("PixyX");
        output.add("PixyY");
        output.add("PixyDistance");
        output.add("PixyAngleFromTarget");
        output.add("PixyRotationOffsetAngle");
        output.add("TargetAbsoluteAngle");
        output.add("NormalizedYaw");
        output.add("RobotAngleInTargetCoordinates");
        output.add("AlignmentTarget");

        return output;
    }

    @Override
    public ArrayList<String> getLogHeaderUnits() {
        ArrayList<String> output = new ArrayList<String>();

        output.add("CM");
        output.add("CM");
        output.add("CM");
        output.add("Degrees");
        output.add("Degrees");
        output.add("Degrees");
        output.add("Degrees");
        output.add("Degrees");
        output.add("Target");

        return output;
    }
}
