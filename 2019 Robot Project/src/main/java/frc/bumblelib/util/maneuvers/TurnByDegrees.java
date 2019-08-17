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

/**
 * Add your docs here.
 */
public class TurnByDegrees implements PIDManeuver {

    private BumbleSensor robotHeading = new BumbleSensor(PIDSourceType.kDisplacement);
    private BumbleMotor pidOutpuMotorTurn = new BumbleMotor();
    private PIDController turnPID = new PIDController(0.0078, 0, 0.034, robotHeading, pidOutpuMotorTurn);
    private double numberOfTimesOnTarget;
    private double TurnBasePower = 0.45;
    private double TOLERANCE = 3,OUTPUT_RANGE = 0.9;
    private BumblePIDCalibration turnPIDCalibration = new BumblePIDCalibration(robotHeading, 3, 0.8);
    
    @Override
    public void init(double setpoint) {
        numberOfTimesOnTarget = 0;
        if (Robot.m_drivetrain.getGyro().getYaw() + setpoint > 180) {
            turnPID.setSetpoint((Robot.m_drivetrain.getGyro().getYaw() + setpoint) - 360);
        } else {
            turnPID.setSetpoint(Robot.m_drivetrain.getGyro().getYaw() + setpoint);
        }
        turnPID.setAbsoluteTolerance(TOLERANCE);
        turnPID.setOutputRange(-OUTPUT_RANGE, OUTPUT_RANGE);
        turnPID.setInputRange(-180, 180);
        turnPID.setContinuous(true);
        turnPID.setEnabled(true);
    }

    @Override
    public void execute() {
        robotHeading.setSensorOutput(Robot.m_drivetrain.getGyro().getYaw());
        if (Math.abs(pidOutpuMotorTurn.get()) < TurnBasePower && Math.abs(pidOutpuMotorTurn.get()) > 0.1) {
            Robot.m_drivetrain.bumbleDrive.arcadeDrive(-TurnBasePower * Math.signum(pidOutpuMotorTurn.get()), 0);
        } else {
            if (Math.abs(pidOutpuMotorTurn.get()) < 0.08) {
                Robot.m_drivetrain.bumbleDrive.arcadeDrive(0, 0);
            } else {
                Robot.m_drivetrain.bumbleDrive.arcadeDrive(-pidOutpuMotorTurn.get(), 0);
            }
        }
    }

    @Override
    public boolean isFinished() {
        if (turnPID.onTarget()) {
            numberOfTimesOnTarget++;
        }
        return numberOfTimesOnTarget >= 5;
    }

    @Override
    public void end() {
        turnPID.disable();
        Robot.m_drivetrain.stopMotors();
    }

    @Override
    public void initCalibration() {

    }

    @Override
    public void executeCalibration() {
        turnPIDCalibration.execute();
        robotHeading.setSensorOutput(Robot.m_drivetrain.getGyro().getYaw());
        if (Math.abs(turnPIDCalibration.getOutput()) < 0.45 && Math.abs(turnPIDCalibration.getOutput()) > 0.1) {
            Robot.m_drivetrain.bumbleDrive.arcadeDrive(-0.45 * Math.signum(turnPIDCalibration.getOutput()), 0);
        } else {
            if (Math.abs(turnPIDCalibration.getOutput()) < 0.08) {
                Robot.m_drivetrain.bumbleDrive.arcadeDrive(0, 0);
            } else {
                Robot.m_drivetrain.bumbleDrive.arcadeDrive(-turnPIDCalibration.getOutput(), 0);
            }
        }
    }

}
