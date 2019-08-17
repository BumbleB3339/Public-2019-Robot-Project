package frc.bumblelib.bumblelib_autonomous;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import frc.bumblelib.bumblelib_autonomous.pathing.AutonomousSettings;
import frc.bumblelib.bumblelib_autonomous.pathing.enums.Alliance;
import frc.bumblelib.bumblelib_autonomous.pathing.enums.Side;
import frc.bumblelib.bumblelib_autonomous.sequence.AutoSequence;
import frc.robot.Robot;
import frc.robot.autonomous.sequences.Sequences;

public class AutoChooser {

    private SendableChooser<Side> sideChooser;
    private SendableChooser<AutoSequence> sequenceChooser;

    private ShuffleboardTab autoTab;

    public AutoChooser() {
        sideChooser = new SendableChooser<Side>();
        sequenceChooser = new SendableChooser<AutoSequence>();
    }

    public void init() {
        autoTab = Shuffleboard.getTab("AutoChooser");
         
        addSideOptions();
        addSequences();

        autoTab.add("Side", sideChooser);
        autoTab.add("Sequence", sequenceChooser);
    }

    public void runSelected() {
        if (sideChooser.getSelected() == null || sequenceChooser.getSelected() == null) {
            Robot.isAuto = false;
            return;
        }
        AutonomousSettings.setAlliance(Alliance.getDriverStationAlliance(DriverStation.getInstance().getAlliance()));
        AutonomousSettings.setSide(sideChooser.getSelected());


        sequenceChooser.getSelected().start();
    }

    private void addSideOptions(){
        sideChooser.setDefaultOption("None", null);
        sideChooser.addOption("Left", Side.LEFT);
        sideChooser.addOption("Right", Side.RIGHT);
    }

    private void addSequences() {
        for (AutoSequence sequence : Sequences.sequenceList) {
            sequenceChooser.addOption(sequence.getName(), sequence);
        }

        sequenceChooser.setDefaultOption("None", null);
    }
}