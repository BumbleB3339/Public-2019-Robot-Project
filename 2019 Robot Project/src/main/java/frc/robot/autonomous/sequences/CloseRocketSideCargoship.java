package frc.robot.autonomous.sequences;

import frc.bumblelib.bumblelib_autonomous.sequence.AutoSequence;
import frc.robot.autonomous.sequences.sub_sequences.FeederCloseSideCargoshipSequence;
import frc.robot.autonomous.sequences.sub_sequences.PlatformCloseRocketFeederSequence;

public class CloseRocketSideCargoship extends AutoSequence {

    public CloseRocketSideCargoship() {
        super("CloseRocketSideCargoship");

        addSequential(new PlatformCloseRocketFeederSequence());
        addSequential(new FeederCloseSideCargoshipSequence());
    }
}