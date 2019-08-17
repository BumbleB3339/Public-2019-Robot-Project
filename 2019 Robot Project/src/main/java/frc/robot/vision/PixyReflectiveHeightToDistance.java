/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.vision;

/**
 * This class corrects a distance between Pixy sensor and the 2019 FIRST
 * Robotics Competition DESTINATION: DEEP SPACE Retro reflective targets.
 */

public class PixyReflectiveHeightToDistance {
    private final static double COEFFICIENT = 25.06, POWER = -0.925;
    
    public static double getDistance(double targetHeight) {
        return COEFFICIENT * Math.pow(targetHeight, POWER);  
    }
}
