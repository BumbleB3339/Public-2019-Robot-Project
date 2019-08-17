/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.bumblelib.util;

/**
 * Add your docs here.
 */
public class BasePowerCalculator {
	private static final double MIN_OUTPUT_TO_ADD_FRICTION = 0.05;

    public static double calculateModifiedOutput(double output, double gravityCompensationPower, 
        double frictionOvercomePower) {
        if (output != 0) {
			double modifiedOutput = output + gravityCompensationPower;
			if (Math.abs(output) > MIN_OUTPUT_TO_ADD_FRICTION) {
				modifiedOutput += Math.signum(output) * frictionOvercomePower;
			}
			return modifiedOutput;
		} else {
			return 0;
		}
	}
	

}
