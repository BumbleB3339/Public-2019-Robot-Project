/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.bumblelib.util.maneuvers;

import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.PIDSourceType;
import frc.bumblelib.util.pid_calibration_tool.BumbleMotor;
import frc.bumblelib.util.pid_calibration_tool.BumblePIDCalibration;
import frc.bumblelib.util.pid_calibration_tool.BumbleSensor;
import frc.robot.Robot;
import static frc.robot.profiles.InitProfiles.ROBOT_PROFILE;

/**
 * Add your docs here.
 */
public class DriveByDistance implements PIDManeuver {
    private BumbleSensor distanceInMeters = new BumbleSensor(PIDSourceType.kDisplacement);
    private BumbleMotor pidOutpuMotorDrive = new BumbleMotor();
    private PIDController drivePID = new PIDController(0.75, 0, 1.8, distanceInMeters, pidOutpuMotorDrive);
    private double numberOfTimesOnTarget;
    private double yawToKeep;
    private final double ANGLE_P = 0.08;
    private double ticksInRotation = ROBOT_PROFILE.autonomousParams.ticks_per_rev;
    private double wheelDiamater = ROBOT_PROFILE.autonomousParams.wheel_diameter;// in meters
    private double TOLERANCE = 3,OUTPUT_RANGE = 0.9;
    private BumblePIDCalibration drivePIDCalibration = new BumblePIDCalibration(distanceInMeters, 0.03, 0.9);

    @Override
    public void init(double setpoint) {
        yawToKeep = Robot.m_drivetrain.getGyro().getYaw();
        numberOfTimesOnTarget = 0;
        drivePID.setSetpoint(setpoint);
        drivePID.setAbsoluteTolerance(TOLERANCE);
        drivePID.setOutputRange(-OUTPUT_RANGE, OUTPUT_RANGE);
        Robot.m_drivetrain.rightEncoderController.setSelectedSensorPosition(0); // TODO: Work with an offset, don't reset.
        drivePID.setEnabled(true);
    }

    @Override
    public void execute() {
        double position = (Robot.m_drivetrain.rightEncoderController.getSelectedSensorPosition() / ticksInRotation)
                * (wheelDiamater * Math.PI);
        distanceInMeters.setSensorOutput(position);
        double errorAngle = yawToKeep - Robot.m_drivetrain.getGyro().getYaw();
        Robot.m_drivetrain.bumbleDrive.arcadeDrive(-errorAngle * ANGLE_P, pidOutpuMotorDrive.get());
    }

    @Override
    public boolean isFinished() {
        if (drivePID.onTarget()) {
            numberOfTimesOnTarget++;
        }
        return numberOfTimesOnTarget >= 5;
    }

    @Override
    public void end() {
        drivePID.disable();
        Robot.m_drivetrain.stopMotors();
    }

    @Override
    public void initCalibration() {
        yawToKeep = Robot.m_drivetrain.getGyro().getYaw();
        Robot.m_drivetrain.rightEncoderController.setSelectedSensorPosition(0);
    }

    @Override
    public void executeCalibration() {
        double position = Robot.m_drivetrain.getRightDistance();
        distanceInMeters.setSensorOutput(position);
        double errorAngle = yawToKeep - Robot.m_drivetrain.getGyro().getYaw();
        drivePIDCalibration.execute();
        Robot.m_drivetrain.bumbleDrive.arcadeDrive(-errorAngle * ANGLE_P, drivePIDCalibration.getOutput());
    }

}
