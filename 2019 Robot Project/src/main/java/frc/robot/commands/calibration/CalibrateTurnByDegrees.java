/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.calibration;

import static frc.robot.Robot.m_drivetrain;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import frc.robot.Robot;
import frc.robot.subsystems.Drivetrain.DrivetrainControlState;
import frc.robot.subsystems.Drivetrain.Gear;
import jaci.pathfinder.Pathfinder;

public class CalibrateTurnByDegrees extends Command {

  private NetworkTableEntry kP;
  private NetworkTableEntry enable;
  private NetworkTableEntry disable;
  private NetworkTableEntry setpoint;
  private NetworkTableEntry currentAngle;
  private NetworkTableEntry rotationBasePower;
  private NetworkTableEntry maxRotationPower;

  private double m_kP = 0.0;
  private double m_setpoint = 0.0;
  private boolean m_isEnabled = false;
  private double m_rotationBasePower = 0.0;
  private double m_maxRotationPower = 0.0;

  public CalibrateTurnByDegrees() {
    requires(m_drivetrain);

    placeDashboardWidgets();

    DriverStation.reportError("Turn By Degrees Calibration Mode is Active!", false);
  }

  private void placeDashboardWidgets() {
    ShuffleboardTab tab = Shuffleboard.getTab("Turn By Degrees Calibration");
    kP = tab.add("kP", 0.0).withWidget(BuiltInWidgets.kTextView).getEntry();

    enable = tab.add("Enable", false).withWidget(BuiltInWidgets.kToggleButton).getEntry();
    disable = tab.add("Disable", false).withWidget(BuiltInWidgets.kToggleButton).getEntry();

    setpoint = tab.add("Setpoint", 0.0).withWidget(BuiltInWidgets.kTextView).getEntry();

    currentAngle = tab.add("Current Angle", 0.0).withWidget(BuiltInWidgets.kTextView).getEntry();

    rotationBasePower = tab.add("Rotation Base Power", 0.0).withWidget(BuiltInWidgets.kTextView).getEntry();

    maxRotationPower = tab.add("Max Rotation Power", 0.0).withWidget(BuiltInWidgets.kTextView).getEntry();
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    Robot.isOperationActive = false;
    Shuffleboard.selectTab("Turn By Degrees Calibration");
    m_drivetrain.initDrivetrainControlState(DrivetrainControlState.OPEN_LOOP);
    m_drivetrain.setGear(Gear.POWER_GEAR);
  }

  private void updateEnableDisable() {
    if (enable.getBoolean(false)) {
      enable.setBoolean(false);
      m_isEnabled = true;

      m_kP = kP.getDouble(0.0);
      m_setpoint = setpoint.getDouble(0.0);
      m_rotationBasePower = rotationBasePower.getDouble(0.0);
      m_maxRotationPower = maxRotationPower.getDouble(0.0);
    }
    if (disable.getBoolean(false)) {
      disable.setBoolean(false);
      m_isEnabled = false;

    }
  }

  private void runPID() {
    if (m_isEnabled && (Math.abs(m_setpoint - m_drivetrain.getGyro().getYaw()) > 1.0)) {
      double rotationPower = -m_kP * Pathfinder.boundHalfDegrees((m_setpoint - m_drivetrain.getGyro().getYaw()));
      rotationPower += Math.copySign(m_rotationBasePower, rotationPower);

      if (Math.abs(rotationPower) > m_maxRotationPower) {
        rotationPower = Math.copySign(m_maxRotationPower, rotationPower);
      }

      Robot.m_drivetrain.bumbleDrive.arcadeDrive(rotationPower, 0.0);
    } else {
      m_drivetrain.stopMotors();
    }
  }

  private void updateDisplayValues() {
    currentAngle.setDouble(m_drivetrain.getGyro().getYaw());
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    updateEnableDisable();
    updateDisplayValues();
    runPID();
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return false;
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
    m_drivetrain.stopMotors();
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
    end();
  }
}
