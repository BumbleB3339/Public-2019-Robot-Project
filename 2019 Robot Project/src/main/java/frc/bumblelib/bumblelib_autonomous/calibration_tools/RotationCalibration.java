/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.bumblelib.bumblelib_autonomous.calibration_tools;

import static frc.robot.Robot.m_drivetrain;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import frc.bumblelib.bumblelib_autonomous.pathing.enums.Alliance;
import frc.bumblelib.bumblelib_autonomous.pathing.enums.Side;
import frc.bumblelib.bumblelib_autonomous.pathing.rotation.Rotation;
import frc.bumblelib.bumblelib_autonomous.pathing.rotation.SingularAngle;
import frc.robot.Robot;
import frc.robot.profiles.InitProfiles;

public class RotationCalibration extends Command {

  private double errorGraphParams[] = {0,0};
  private NetworkTableEntry angleEntry;
  private NetworkTableEntry gyroPEntry;
  private NetworkTableEntry errorGraphEntry;

  private Rotation rotation;

  private Notifier iter;

  private ShuffleboardTab tab;

  public RotationCalibration() {
    requires(m_drivetrain);

    tab = Shuffleboard.getTab("Rotation Calibration");
    angleEntry = tab.add("Angle", 0.0).withWidget(BuiltInWidgets.kTextView).getEntry();
    gyroPEntry = tab.add("gyroP", 0.0).withWidget(BuiltInWidgets.kTextView).getEntry();
    errorGraphEntry = tab.add("Error Graph", errorGraphParams).withWidget(BuiltInWidgets.kGraph).withSize(4,4).getEntry();
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    Shuffleboard.selectTab("Rotation Calibration");
    rotation = new Rotation(m_drivetrain.getGear(), new SingularAngle(angleEntry.getNumber(0.0).intValue()), true);
    rotation.setGyroP(gyroPEntry.getDouble(0.0));

    rotation.configAllianceSide(Alliance.RED, Side.RIGHT);
    rotation.init();

    iter = new Notifier(new Thread(){
      @Override
      public void run() {
        rotation.follow();
      }
    });

    iter.startPeriodic(InitProfiles.ROBOT_PROFILE.pathfinderParams.deltaTime);
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    errorGraphParams[0] = rotation.getCurrentAngle();
    try {
      errorGraphParams[1] = rotation.getAngleSetpoint();
    } catch (NullPointerException e) {
      errorGraphParams[1] = 0.0;
    }

    errorGraphEntry.setDoubleArray(errorGraphParams);
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return rotation.isFinished();
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
    iter.stop();
    Robot.m_drivetrain.stopMotors();
    rotation.end();
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
    end();
  }
}
