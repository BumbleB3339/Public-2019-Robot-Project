package frc.bumblelib.bumblelib_autonomous.auto_commands;

import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.command.Command;
import frc.bumblelib.bumblelib_autonomous.pathing.AutonomousSettings;
import frc.bumblelib.bumblelib_autonomous.pathing.rotation.Rotation;
import frc.robot.Robot;
import frc.robot.profiles.InitProfiles;

public class Rotate extends Command {

    protected Rotation rotation;

    private Notifier iter;

    public Rotate(Rotation rotation) {
        requires(Robot.m_drivetrain);
        this.rotation = rotation;
    }

    @Override
    protected void initialize() {
        rotation.configAllianceSide(AutonomousSettings.getAlliance(), AutonomousSettings.getSide());
        rotation.init();

        iter = new Notifier(new Thread() {
            @Override
            public void run() {

                rotation.follow();

            }
        });
        iter.startPeriodic(InitProfiles.ROBOT_PROFILE.pathfinderParams.deltaTime);
    }

    @Override
    protected void execute() {
    }

    @Override
    protected void end() {
        iter.stop();
        Robot.m_drivetrain.stopMotors();
        rotation.end();
    }

    @Override
    protected void interrupted() {
        end();
    }

    @Override
    protected boolean isFinished() {
        return rotation.isFinished();
    }

}