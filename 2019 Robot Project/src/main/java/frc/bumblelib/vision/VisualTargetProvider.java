/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.bumblelib.vision;

import frc.bumblelib.bumblelib_autonomous.pathing.RobotReferencePoint;

/**
 * An interface that defines how a visual sensor provides data for a specific target.
 * The interface will be implemented by every sensor (Pixy, camera).
 */
public abstract class VisualTargetProvider {

    /**
     * @return Target Center X in pixels
     */
    public abstract double getTargetX();

    /**
     * @return Target Center y in pixels
     */
    public abstract double getTargetY();

    /**
     * @return Target Width in pixels
     */
    public abstract double getTargetWidth();

    /**
     * @return Target Height in pixels
     */
    public abstract double getTargetHeight();

    /**
     * 
     * @return is the target made of a single reflective stripe 
     */
    public abstract boolean isTargetSingleReflective();

    /**
     * @return Ratio between width and height
     */
    public double getWidthToHeightRatio() {
        return getTargetWidth() / getTargetHeight();
    }

    /**
     * @return Horizontal field of view in deegres
     */
    public abstract double getHFOV();

    /**
     * @return Vertical field of view in deegres
     */
    public abstract double getVFOV();

    /**
     * @return Screen height in pixel
     */
    public abstract int getScreenHeight();

    /**
     * @return Screen width in pixel
     */
    public abstract int getScreenWidth();

    /**
     * @return Whether the target is detected in the image
     */
    public abstract boolean isTargetDetected();
    
    /**
     * @return Sensor reference point
     */
    public abstract RobotReferencePoint getSensorRobotReferncePoint();

    /**
     * This method allows to change the target selection algorithm from outside.
     * For example, select the leftmost target, highest target, etc.
     * @param targetSelectionPolicy Set selected target
     */
    public abstract void setTargetSelectionPolicy(TargetSelectionPolicy targetSelectionPolicy);

    /**
     * @return Current target selection policy.
     */ 
    public abstract TargetSelectionPolicy getTargetSelectionPolicy();

    /**
     * Stores the target properties from the current image.
     * Should be called before the data processing is done. 
     */
    public abstract void updateImage();

    /**
     *  Target Selection by position
     */
    public enum TargetSelectionPolicy {
        BIGGEST, LEFTMOST, RIGHTMOST, HIGHEST, CENTERMOST;
    }

    /**
     * When the sensor is on the same plane as the robot the angle is 0
     * @return The angle between the sensor plane and the robot plane
     */
    public abstract double getSensorAngle();
}
