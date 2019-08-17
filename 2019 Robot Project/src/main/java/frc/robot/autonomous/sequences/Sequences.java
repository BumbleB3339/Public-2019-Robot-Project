package frc.robot.autonomous.sequences;

import java.util.ArrayList;

import frc.bumblelib.bumblelib_autonomous.sequence.AutoSequence;
import frc.bumblelib.bumblelib_autonomous.sequence.SemiAutoSequence;

public class Sequences {

    public static ArrayList<AutoSequence> sequenceList = new ArrayList<AutoSequence>();

    public static AutoSequence closeFarRocket, frontCargoshipFarRocket, frontCargoshipCloseSideCargoship, closeRocketSideCargoship;
    public static SemiAutoSequence semiAutoCloseFarRocket, semiAutoDoubleFarRocket, semiAutoMidFarShip, testSequence;

    public Sequences() {

        closeFarRocket = new CloseFarRocket();
        frontCargoshipFarRocket = new FrontCargoshipFarRocket();
        frontCargoshipCloseSideCargoship = new FrontCargoshipCloseSideCargoship();
        closeRocketSideCargoship = new CloseRocketSideCargoship();

        sequenceList.add(frontCargoshipFarRocket);
        sequenceList.add(closeFarRocket);
        sequenceList.add(frontCargoshipCloseSideCargoship);
        sequenceList.add(closeRocketSideCargoship);
    }
}
