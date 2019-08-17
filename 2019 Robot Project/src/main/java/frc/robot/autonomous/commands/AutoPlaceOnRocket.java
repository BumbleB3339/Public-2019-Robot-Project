package frc.robot.autonomous.commands;

import edu.wpi.first.wpilibj.command.CommandGroup;
import edu.wpi.first.wpilibj.command.ConditionalCommand;
import edu.wpi.first.wpilibj.command.StartCommand;
import frc.bumblelib.bumblelib_autonomous.auto_commands.DriveByTime;
import frc.bumblelib.bumblelib_autonomous.auto_commands.Wait;
import frc.bumblelib.bumblelib_autonomous.auto_commands.WaitAndThen;
import frc.robot.Robot;
import frc.robot.RobotGameState.RobotSystemState;
import frc.robot.commands.SetCurrentRobotSystemState;
import frc.robot.commands.hatch_handler.CommandEjectHatchPanel;
import frc.robot.subsystems.Drivetrain.Gear;
import jaci.pathfinder.Pathfinder;

public class AutoPlaceOnRocket extends CommandGroup {

    private final double ANGLE_ALIGNMENT_TOLERANCE = 3.0;
    public static boolean aligned = false;

    public AutoPlaceOnRocket(double rocketAngle) {
        addSequential(new CommandAutoAlignToTarget());
        addParallel(new StartCommand(new CommandEjectHatchPanel()));
        addSequential(new DriveByTime(0.2, 0.25, Gear.POWER_GEAR));
        //addSequential(new Wait(0.2));

        addSequential(new ConditionalCommand(new AlignToRocket()){
        
            @Override
            protected boolean condition() {
                aligned = Math.abs(Pathfinder.boundHalfDegrees(Robot.m_drivetrain.getGyro().getYaw() - rocketAngle)) > ANGLE_ALIGNMENT_TOLERANCE;
                return aligned;
            }
        });
        
    }

    @Override
    protected void initialize() {
        aligned = false;
    }

    private class AlignToRocket extends CommandGroup {
        public AlignToRocket() {
            addSequential(new Wait(0.2));
            addParallel(new SetCurrentRobotSystemState(RobotSystemState.L2_HATCH_FOLDED, false));
            addSequential(new WaitAndThen(0.2, new DriveByTime(0.3, 0.4, Gear.POWER_GEAR)));
        }
    }
}