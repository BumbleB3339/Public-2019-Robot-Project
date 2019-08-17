package frc.bumblelib.bumblelib_autonomous.calibration_tools;

import static frc.robot.Robot.m_drivetrain;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import frc.robot.Robot;

public class MaxAccelerationAndMaxVelocityTuner extends Command {

    private final double MAX_DISTANCE = 5.1; // The maximum distance the robot is allowed to travel. (meters)
    private final double DISTANCE_HALF_POWER = 0.05; // The initial distance in which to apply half motor output.
                                                     // (meters)
    private boolean isRotation;

    private ShuffleboardTab tunerCalibrationTab;
    private NetworkTableEntry maxVelocityEntry;
    private NetworkTableEntry maxAccelerationEntry;
    private double prevTime, prevVelocity, maxAcc, maxVelocity, distance, initDistance;

    public MaxAccelerationAndMaxVelocityTuner(boolean isRotation) {
        requires(m_drivetrain);
        this.isRotation = isRotation;
    }

    @Override
    protected void initialize() {
        tunerCalibrationTab = Shuffleboard.getTab("MaxVelocityAndAcceleration Calibration");
        maxVelocityEntry = tunerCalibrationTab.add("Max Velocity: ", 0.0).withPosition(1, 1).getEntry();
        maxAccelerationEntry = tunerCalibrationTab.add("Max Acceleration: ", 0.0).withPosition(2, 1).getEntry();
        Shuffleboard.selectTab("MaxVelocityAndAcceleration Calibration");

        maxAcc = 0;
        maxVelocity = 0;
        distance = 0;
        initDistance = getAverageDistance();
        prevTime = Timer.getFPGATimestamp();
        prevVelocity = getAverageVelocity();
    }

    @Override
    protected void execute() {
        double currentVelocity = getAverageVelocity();
        double currentTime = Timer.getFPGATimestamp();
        double currentAcc = (currentVelocity - prevVelocity) / (currentTime - prevTime);
        distance = getAverageDistance() - initDistance;

        if (currentAcc > maxAcc) {
            maxAcc = currentAcc;
        }

        if (currentVelocity > maxVelocity) {
            maxVelocity = currentVelocity;
        }

        double sign = isRotation ? -1.0 : 1.0;
        if (distance <= DISTANCE_HALF_POWER) {
            m_drivetrain.bumbleDrive.setLeftRightMotorOutputs(sign * 0.5, 0.5, false);
        } else {
            m_drivetrain.bumbleDrive.setLeftRightMotorOutputs(sign * 1.0, 1.0, false);
        }

        maxAccelerationEntry.setNumber(maxAcc);
        maxVelocityEntry.setNumber(maxVelocity);

        prevTime = currentTime;
        prevVelocity = currentVelocity;
    }

    private double getAverageDistance() {
        return (Math.abs(m_drivetrain.getLeftDistance()) + Math.abs(m_drivetrain.getRightDistance())) / 2.0;
    }
    
      private double getAverageVelocity() {
        return (Math.abs(m_drivetrain.getLeftVelocity()) + Math.abs(m_drivetrain.getRightVelocity())) / 2.0;
    }

    @Override
    protected boolean isFinished() {
        return distance >= MAX_DISTANCE;
    }

    @Override
    protected void end() {
        Robot.m_drivetrain.stopMotors();
    }

    @Override
    protected void interrupted() {
        end();
    }
}