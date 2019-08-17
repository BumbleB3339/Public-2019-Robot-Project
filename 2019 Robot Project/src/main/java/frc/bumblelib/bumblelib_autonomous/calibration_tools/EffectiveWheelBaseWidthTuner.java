package frc.bumblelib.bumblelib_autonomous.calibration_tools;

import static frc.robot.Robot.m_drivetrain;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;

public class EffectiveWheelBaseWidthTuner extends Command {

    private final int NUM_OF_ROTS = 10;
    private final double ANGLE_TOLERANCE = 5;
    private final double POWER_TO_APPLY = 0.3;

    private double overallDistance;
    private double initRightEncoderDist;
    private double initLeftEncoderDist;
    private double currentRightEncoderDist;
    private double currentLeftEncoderDist;

    private ShuffleboardTab tunerCalibrationTab;
    private NetworkTableEntry rotation;
    private NetworkTableEntry gyro;
    private NetworkTableEntry wheelBaseWidth;

    private int rotations;

    private boolean zeroFlag;

    public EffectiveWheelBaseWidthTuner() {
        requires(m_drivetrain);
    }

    @Override
    protected void initialize() {
        tunerCalibrationTab = Shuffleboard.getTab("EffectiveWheelBaseWidth Calibration");
        rotation = tunerCalibrationTab.add("Rotations: ", 0.0).withPosition(1, 1).getEntry();
        wheelBaseWidth = tunerCalibrationTab.add("Wheel Base Width: ", 0.0).withPosition(2, 1).getEntry();
        gyro = tunerCalibrationTab.add("Gyro: ", 0.0).withPosition(3, 1).getEntry();
        Shuffleboard.selectTab("EffectiveWheelBaseWidth Calibration");

        try {
            m_drivetrain.getGyro().startRelativeMode();
        } catch (Exception e) {
            e.printStackTrace();
        }
        zeroFlag = true;
        rotations = 0;
        initRightEncoderDist = m_drivetrain.getRightDistance();
        initLeftEncoderDist = m_drivetrain.getLeftDistance();
    }

    @Override
    protected void execute() {
        currentLeftEncoderDist = Math.abs(m_drivetrain.getLeftDistance() - initLeftEncoderDist);
        currentRightEncoderDist = Math.abs(m_drivetrain.getRightDistance() - initRightEncoderDist);
        overallDistance = (currentLeftEncoderDist + currentRightEncoderDist) / 2;

        try {
            if (Math.abs(m_drivetrain.getGyro().getRelativeYaw()) <= ANGLE_TOLERANCE) {
                if (!zeroFlag) {
                    rotations++;
                    zeroFlag = true;

                    rotation.setNumber(rotations);
                    wheelBaseWidth.setNumber(overallDistance / rotations / Math.PI);
                }
            } else {
                zeroFlag = false;
            }
        } catch (Exception e) {
            e.printStackTrace();
        }

        gyro.setNumber(m_drivetrain.getGyro().getYaw());
        m_drivetrain.setLeftRightMotorOutputs(-POWER_TO_APPLY, POWER_TO_APPLY);
    }

    @Override
    protected boolean isFinished() {
        return rotations == NUM_OF_ROTS;
    }

    @Override
    protected void end() {
        gyro.setNumber(m_drivetrain.getGyro().getYaw());
        rotation.setNumber(rotations);
        wheelBaseWidth.setNumber(overallDistance / rotations / Math.PI);

        m_drivetrain.getGyro().endRelativeMode();
        m_drivetrain.stopMotors();
    }

    @Override
    protected void interrupted() {
        end();
    }
}