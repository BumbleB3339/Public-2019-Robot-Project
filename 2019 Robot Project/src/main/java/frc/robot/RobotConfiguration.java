/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

/**
 * This class represetns params for a specific robot configuration. A robot
 * configuration consists of setpoints for Lift, LiftArm, ElementArm and
 * IntakeArm.
 */
public class RobotConfiguration {

    // public only to retrieve them, not to set them!
    public double liftHeight, liftArmAngle, elementArmAngle;

    public RobotConfiguration(double liftHeight, double liftArmAngle, double elementArmAngle) {
        this.liftHeight = liftHeight;
        this.liftArmAngle = liftArmAngle;
        this.elementArmAngle = elementArmAngle;
    }

    public RobotConfiguration setLiftHeight(double liftHeight) {
        return new RobotConfiguration(liftHeight, this.liftArmAngle, this.elementArmAngle);
    }

    public RobotConfiguration setLiftArmAngle(double liftArmAngle) {
        return new RobotConfiguration(this.liftHeight, liftArmAngle, this.elementArmAngle);
    }

    public RobotConfiguration setElementArmAngle(double elementArmAngle) {
        return new RobotConfiguration(this.liftHeight, this.liftArmAngle, elementArmAngle);
    }
}
