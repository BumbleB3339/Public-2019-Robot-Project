/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.bumblelib.vision;

import frc.robot.vision.PixyReflectiveHeightToDistance;

/**
 * This class is responsible for all mathematical and visual processing.
 */
public class VisualTargetProcessor {

    public VisualTargetProvider targetProvider;
    private PhysicalTarget physicalTarget;
    public static final double INVALID_VALUE = -3339;
    private double pixelsInDegree;

    public VisualTargetProcessor(VisualTargetProvider targetProvider, PhysicalTarget physicalTarget) {
        this.targetProvider = targetProvider;
        this.physicalTarget = physicalTarget;

        pixelsInDegree = targetProvider.getScreenWidth() / targetProvider.getHFOV();
    }

    /**
     * @return returns the distance from the pixy to target in meters
     */
    public double getDistanceToTarget() {
        if (!isTargetDetected()) {
            return INVALID_VALUE;
        }
        return PixyReflectiveHeightToDistance.getDistance(targetProvider.getTargetHeight());
    }

    /**
     * @param targetProvider the targetProvider to set
     */
    public void setTargetProvider(VisualTargetProvider targetProvider) {
        this.targetProvider = targetProvider;
    }

    /**
     * Angle is always positive This function finds the angle between the
     * perpendicular line from the target and the line that conects the sensor to
     * the target.
     * 
     * @return sensor angle from the target
     */
    public double getAngleFromTarget() {
        if (!isTargetDetected()) {
            return INVALID_VALUE;
        }

        double actualTargetWidthToHeightRatio = Math.min(targetProvider.getWidthToHeightRatio(), physicalTarget.getMeasuredWidthToHeightRatio());
        return Math.toDegrees( Math.acos( actualTargetWidthToHeightRatio / physicalTarget.getMeasuredWidthToHeightRatio() ));
    }

    /**
     * @return the angle between an imaginary line that conects the sensor and the
     *         target and the line that conects the sensor with the point that is
     *         located in the middle of the screen.
     */
    public double getRotationOffsetAngle() {
        if (!isTargetDetected()) {
            return INVALID_VALUE;
        }
        
        return ((targetProvider.getScreenWidth() / 2) - targetProvider.getTargetX()) / pixelsInDegree
                + targetProvider.getSensorAngle();
    }

    public boolean isTargetDetected() {
        return targetProvider.isTargetDetected();
    }
}