package frc.bumblelib.bumblelib_autonomous.auto_commands;

import static frc.robot.profiles.InitProfiles.ROBOT_PROFILE;

import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.command.Command;
import frc.bumblelib.bumblelib_autonomous.pathing.AutonomousSettings;
import frc.bumblelib.bumblelib_autonomous.pathing.PathFollower;
import frc.bumblelib.bumblelib_autonomous.pathing.path.Path;
import frc.robot.Robot;
import frc.robot.subsystems.Drivetrain.DrivetrainControlState;

public class DrivePath extends Command {

    private Path path;
    protected PathFollower follower;

    protected Notifier iter;

    // The following code is used for statistical verification of the thread
    // interval.
    // private double currentTimeStamp;
    // private double lastTimeStamp;
    // private int iterCount = 0;
    // private double intervalSum = 0.0;
    // @SuppressWarnings("unused")
    // private double intervalAvg = 0.0;
    // private double currentTimeIntervalMs = 0.0;

    public DrivePath(Path path) {
        requires(Robot.m_drivetrain);
        this.path = path;
    }

    @Override
    protected void initialize() {
        Robot.m_drivetrain.initDrivetrainControlState(DrivetrainControlState.PATH_FOLLOWING);
        follower = new PathFollower(path, AutonomousSettings.getAlliance(), AutonomousSettings.getSide());

        follower.init();

        // The following code is used for statistical verification of the thread
        // interval.
        // currentTimeStamp = Timer.getFPGATimestamp();
        // lastTimeStamp = currentTimeStamp;
        // iterCount = 0;
        // intervalSum = 0.0;

        iter = new Notifier(new Thread() {
            @Override
            public void run() {

                // The following code is used for statistical verification of the thread
                // interval.
                // currentTimeStamp = Timer.getFPGATimestamp();

                // iterCount++;
                // // Multiplied by 1000 to convert seconds to miliseconds
                // currentTimeIntervalMs = (currentTimeStamp - lastTimeStamp) * 1000;
                // intervalSum += currentTimeIntervalMs;
                // intervalAvg = intervalSum / iterCount;

                // lastTimeStamp = currentTimeStamp;

                follower.followPath();

            }
        });
        iter.startPeriodic(ROBOT_PROFILE.pathfinderParams.deltaTime);
    }

    @Override
    protected void execute() {

    }

    @Override
    protected boolean isFinished() {
        return follower.isPathFinished();
    }

    @Override
    protected void end() {
        Robot.m_drivetrain.lastAvgPower = Robot.m_drivetrain.getMotorsPowers();
        iter.stop();
        Robot.m_drivetrain.stopMotors();
        follower.endPath();
    }

    @Override
    protected void interrupted() {
        end();
    }

    public Path getPath() {
        return path;
    }

    public String getLogData() {
        try {
            return "Progress (percents): " + follower.getCurrentPercentOfPath() + "\n" + "Distance to endpoint: "
                    + follower.getDistanceToEnd() + "\nMotor Overflow: " + follower.getMotorOverflowCount();
        } catch (NullPointerException e) {
            return "Calculating...";
        }
    }
}
