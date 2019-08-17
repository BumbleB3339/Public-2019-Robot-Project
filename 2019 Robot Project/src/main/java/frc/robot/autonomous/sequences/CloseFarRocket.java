package frc.robot.autonomous.sequences;

import frc.bumblelib.bumblelib_autonomous.sequence.AutoSequence;
import frc.robot.autonomous.sequences.sub_sequences.FeederToFarRocketSequence;
import frc.robot.autonomous.sequences.sub_sequences.PlatformCloseRocketFeederSequence;

public class CloseFarRocket extends AutoSequence {

    public CloseFarRocket() {
        super("CloseFarRocket");

        addSequential(new PlatformCloseRocketFeederSequence());
        addSequential(new FeederToFarRocketSequence());
    }
}