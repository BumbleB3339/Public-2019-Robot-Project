package frc.bumblelib.bumblelib_autonomous.calibration_tools;

import static frc.robot.Robot.m_drivetrain;
import static frc.robot.profiles.InitProfiles.ROBOT_PROFILE;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import frc.bumblelib.bumblelib_autonomous.pathing.AutonomousSettings;
import frc.bumblelib.bumblelib_autonomous.pathing.PathFollower;
import frc.bumblelib.bumblelib_autonomous.pathing.path.SingularPath;
import frc.robot.Robot;

public class PDGTuner extends Command {

    private Notifier iter;

    private SendableChooser<SingularPath> pathChooser;
    private PathFollower follower;

    private ShuffleboardTab autoCalibrationTab;
    private NetworkTableEntry kpEntry;
    private NetworkTableEntry kdEntry;
    private NetworkTableEntry gyroPEntry;
    private NetworkTableEntry rightError;
    private NetworkTableEntry leftError;
    private NetworkTableEntry gyroError;
    private NetworkTableEntry progress;
    private NetworkTableEntry distanceToEnd;
    private NetworkTableEntry motorOverflow;

    private double[] leftData = new double[2], rightData = new double[2];

    public PDGTuner(SingularPath... paths) {
        requires(Robot.m_drivetrain);
        pathChooser = new SendableChooser<>();

        // Init Shuffleboard
        autoCalibrationTab = Shuffleboard.getTab("Autonomous Calibration");
        autoCalibrationTab.add("Path: ", pathChooser).withPosition(0, 3);

        kpEntry = autoCalibrationTab.add("Kp: ", 0.0).withPosition(1, 3).getEntry();
        kdEntry = autoCalibrationTab.add("Kd: ", 0.0).withPosition(2, 3).getEntry();
        gyroPEntry = autoCalibrationTab.add("Gyro p: ", 0.0).withPosition(3, 3).getEntry();

        progress = autoCalibrationTab.add("Progress: ", 0.0).withPosition(5, 3).getEntry();
        distanceToEnd = autoCalibrationTab.add("Distance To End: ", 0.0).withPosition(6, 3).getEntry();
        motorOverflow = autoCalibrationTab.add("Motor Overflow: ", 0.0).withPosition(7, 3).getEntry();

        leftError = autoCalibrationTab.add("Left Erorr: ", leftData).withWidget(BuiltInWidgets.kGraph).withSize(3, 3)
                .withPosition(0, 0).getEntry();
        gyroError = autoCalibrationTab.add("Gyro Erorr: ", 0.0).withWidget(BuiltInWidgets.kGraph).withSize(3, 3)
                .withPosition(3, 0).getEntry();
        rightError = autoCalibrationTab.add("Right Erorr: ", rightData).withWidget(BuiltInWidgets.kGraph).withSize(3, 3)
                .withPosition(6, 0).getEntry();

        // Add paths to chooser
        for (SingularPath path : paths) {
            pathChooser.addOption(path.getName(), path);
        }
    }

    @Override
    protected void initialize() {
        Shuffleboard.selectTab("Autonomous Calibration");

        try {
            m_drivetrain.getGyro().startRelativeMode();
        } catch (Exception e) {
            e.printStackTrace();
        }
        follower = new PathFollower(pathChooser.getSelected(), AutonomousSettings.getAlliance(),
                AutonomousSettings.getSide());
        follower.setPDG(kpEntry.getDouble(0.0), kdEntry.getDouble(0.0), gyroPEntry.getDouble(0.0));
        follower.init();

        iter = new Notifier(new Thread() {
            @Override
            public void run() {
                follower.followPath();
            }
        });
        iter.startPeriodic(ROBOT_PROFILE.pathfinderParams.deltaTime);
    }

    @Override
    protected void execute() {
        try {
            leftData[0] = follower.getCurrentLeftPos();
            leftData[1] = follower.getWantedLeftPos();
            rightData[0] = follower.getCurrentRightPos();
            rightData[1] = follower.getWantedRightPos();

            rightError.setDoubleArray(rightData);
            leftError.setDoubleArray(leftData);
            gyroError.setDouble(follower.getCurrentGyroError());

            progress.setNumber(follower.getCurrentPercentOfPath());
            distanceToEnd.setNumber(follower.getDistanceToEnd());
            motorOverflow.setNumber(follower.getMotorOverflowCount());
        } catch (Exception e) {
            e.printStackTrace();
        }
    }

    @Override
    protected boolean isFinished() {
        return this.follower.isPathFinished();
    }

    @Override
    protected void end() {
        iter.stop();
        Robot.m_drivetrain.stopMotors();
        follower.endPath();
        m_drivetrain.getGyro().endRelativeMode();
    }

    @Override
    protected void interrupted() {
        end();
    }
}