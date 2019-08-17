/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.autonomous.sequences;

import frc.bumblelib.bumblelib_autonomous.sequence.AutoSequence;
import frc.robot.autonomous.sequences.sub_sequences.FeederToFarRocketSequence;
import frc.robot.autonomous.sequences.sub_sequences.MidPlatformCargoshipFrontFeederSequence;

/**
 * Add your docs here.
 */
public class FrontCargoshipFarRocket extends AutoSequence {

    public FrontCargoshipFarRocket() {
        super("FrontCargoshipFarRocket");

        addSequential(new MidPlatformCargoshipFrontFeederSequence());
        addSequential(new FeederToFarRocketSequence());
    }
}
