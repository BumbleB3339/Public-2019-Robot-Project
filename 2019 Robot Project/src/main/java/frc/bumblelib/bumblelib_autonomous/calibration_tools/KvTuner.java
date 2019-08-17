/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.bumblelib.bumblelib_autonomous.calibration_tools;

import static frc.robot.Robot.m_drivetrain;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Command;
import frc.bumblelib.util.SimpleCSVLogger;
import frc.robot.Robot;

/**
 * This class generates a CSV file that can be used to derive the value of the
 * velocity coefficiant. It does this by sending an increasing applied voltage
 * to the robot, waiting until the robot has finished accelerating and then
 * logging applied voltage and velocity. The relation between these values is
 * defined by the formula: Vapp = kV * velocity + kA * acceleration = kV *
 * velocity + kA * (0) Vapp = kV * velocity By graphing Vapp as a function of
 * velocity, kV could be derived (the slope of the function).
 */
public class KvTuner extends Command {

  private final double MAX_DISTANCE = 5.1; // The maximum distance the robot is allowed to travel. (meters)
  private final double DELTA_PERCENT = 0.05; // The percent to add between each measurement. (%)
  private final double ACC_TOLERANCE = 1E-4; // The accelration under which a measurement is taken. (meters per second
                                             // per second)
  private boolean isRotation;

  private double currentOutput, prevVelocity, prevTime, initDistance, distance;
  private int logCount;

  private SimpleCSVLogger logger;

  public KvTuner(boolean isRotation) {
    requires(Robot.m_drivetrain);
    
    this.isRotation = isRotation;
    logger = new SimpleCSVLogger("KvTuningLog");
    logger.init(new String[] { "vel", "output" }, new String[] { "m/s", "%" });
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    currentOutput = 0;
    prevVelocity = getAverageVelocity();
    prevTime = Timer.getFPGATimestamp();
    initDistance = getAverageDistance();
    logCount = 0;
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    logCount++;
    double currentVelocity = getAverageVelocity();
    double currentTime = Timer.getFPGATimestamp();
    double currentAcc = (currentVelocity - prevVelocity) / (currentTime - prevTime);
    distance = getAverageDistance() - initDistance;

    if (currentAcc < ACC_TOLERANCE) {
      if (getAverageVelocity() <= 1E-3) {
        currentOutput += 0.001;

      } else {
        if (logCount > 4) {
          logCount = 0;
          logger.writeData(currentVelocity, currentOutput);
          currentOutput += DELTA_PERCENT;
        }
      }
    }

    if (isRotation) {
      m_drivetrain.bumbleDrive.setLeftRightMotorOutputs(-currentOutput, currentOutput, false);
    } else {
      m_drivetrain.bumbleDrive.setLeftRightMotorOutputs(currentOutput, currentOutput, false);
    }
    
    // Update vars.
    prevVelocity = currentVelocity;
    prevTime = currentTime;
  }

  private double getAverageDistance() {
    return (Math.abs(m_drivetrain.getLeftDistance()) + Math.abs(m_drivetrain.getRightDistance())) / 2.0;
  }

  private double getAverageVelocity() {
    return (Math.abs(m_drivetrain.getLeftVelocity()) + Math.abs(m_drivetrain.getRightVelocity())) / 2.0;
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return distance >= MAX_DISTANCE;
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
    logger.close();
    m_drivetrain.stopMotors();
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
    end();
  }
}
