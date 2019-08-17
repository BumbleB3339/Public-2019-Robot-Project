package frc.bumblelib.bumblelib_autonomous.sequence;

import edu.wpi.first.wpilibj.command.CommandGroup;
import frc.bumblelib.bumblelib_autonomous.auto_commands.AutonomousGyroSet;
import frc.bumblelib.bumblelib_autonomous.auto_commands.DrivePath;
import frc.bumblelib.bumblelib_autonomous.pathing.path.Path;
import frc.robot.OI;
import frc.robot.Robot;
import frc.robot.subsystems.Drivetrain.DrivetrainControlState;

public abstract class AutoSequence extends CommandGroup implements Cloneable {

    private boolean isFirstPath = true;
    private final boolean resetGyroIfAbsolutePath = false;

    public AutoSequence(String sequenceName) {
        super(sequenceName);
    }

    public void addSequential(DrivePath command) {
        checkForInitialOffset(command.getPath());
        super.addSequential(command);
    }

    public void addParallel(DrivePath command) {
        checkForInitialOffset(command.getPath());
        super.addParallel(command);
    }

    private void checkForInitialOffset(Path path) {
        if (isFirstPath) {
            if (!path.isRelative() && resetGyroIfAbsolutePath) {
                addSequential(new AutonomousGyroSet(path));
            }
            isFirstPath = false;
        }
    }

    @Override
    protected boolean isFinished() {
        return OI.driverController.getAButton() || super.isFinished();
    }

    @Override
    protected Object clone() {
        try {
            return super.clone();
        } catch (CloneNotSupportedException e) {
            throw new RuntimeException("Can't clone sequence");
        }
    }

    @Override
    protected void interrupted() {
        end();
    }

    @Override
    protected void end() {
        Robot.isAuto = false;
        Robot.m_drivetrain.initDrivetrainControlState(DrivetrainControlState.OPEN_LOOP);
    }
}