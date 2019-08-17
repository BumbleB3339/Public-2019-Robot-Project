/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.calibration;

import static frc.robot.Robot.m_lift;
import static frc.robot.profiles.InitProfiles.ROBOT_PROFILE;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import frc.robot.OI;
import frc.robot.Robot;
import frc.robot.subsystems.Lift.LiftState;

public class CalibrateLift extends Command {

  private NetworkTableEntry kPdown;
  private NetworkTableEntry kIDown;
  private NetworkTableEntry kDDown;
  private NetworkTableEntry kPUp;
  private NetworkTableEntry kIUp;
  private NetworkTableEntry kDUp;
  private NetworkTableEntry enable;
  private NetworkTableEntry isOnTarget;
  private NetworkTableEntry setpoint;
  private NetworkTableEntry heightTolerance;
  private NetworkTableEntry velocityTolerance;
  private NetworkTableEntry gravityCompensationPower;
  private NetworkTableEntry basePower;
  private NetworkTableEntry applyBasePower;
  private NetworkTableEntry rampRate;
  private NetworkTableEntry currentHeight;
  private NetworkTableEntry errorGraph;
  private NetworkTableEntry isPIDControllerEnabled;

  private double[] errorGraphParams = new double[2];

  public CalibrateLift() {
    requires(m_lift);

    placeDashboardWidgets();

    DriverStation.reportError("Lift Calibration Mode is Active!", false);
  }

  private void placeDashboardWidgets() {
    ShuffleboardTab tab = Shuffleboard.getTab("Lift Calibration");
    kPdown = tab.add("kP Down", ROBOT_PROFILE.liftParams.upMovementPIDPreset.getKp())
        .withWidget(BuiltInWidgets.kTextView).getEntry();
    kIDown = tab.add("kI Down", ROBOT_PROFILE.liftParams.upMovementPIDPreset.getKi())
        .withWidget(BuiltInWidgets.kTextView).getEntry();
    kDDown = tab.add("kD Down", ROBOT_PROFILE.liftParams.upMovementPIDPreset.getKd())
        .withWidget(BuiltInWidgets.kTextView).getEntry();

    kPUp = tab.add("kP Up", ROBOT_PROFILE.liftParams.downMovementPIDPreset.getKp()).withWidget(BuiltInWidgets.kTextView)
        .getEntry();
    kIUp = tab.add("kI Up", ROBOT_PROFILE.liftParams.downMovementPIDPreset.getKi()).withWidget(BuiltInWidgets.kTextView)
        .getEntry();
    kDUp = tab.add("kD Up", ROBOT_PROFILE.liftParams.downMovementPIDPreset.getKd()).withWidget(BuiltInWidgets.kTextView)
        .getEntry();

    setpoint = tab.add("Setpoint", 0.0).withWidget(BuiltInWidgets.kTextView).getEntry();
    heightTolerance = tab.add("Height Tolerance", m_lift.POSITION_TOLERANCE_HEIGHT).withWidget(BuiltInWidgets.kTextView)
        .getEntry();
    velocityTolerance = tab.add("Velocity Tolerance", m_lift.VELOCITY_TOLERANCE).withWidget(BuiltInWidgets.kTextView)
        .getEntry();
    gravityCompensationPower = tab.add("Gravity Compansation Power", ROBOT_PROFILE.liftParams.gravityCompensationPower)
        .withWidget(BuiltInWidgets.kTextView).getEntry();
    basePower = tab.add("Base Power", 0.0).withWidget(BuiltInWidgets.kTextView).getEntry();
    rampRate = tab.add("Ramp Rate", m_lift.RAMP_RATE).withWidget(BuiltInWidgets.kTextView).getEntry();

    enable = tab.add("Enable", false).withWidget(BuiltInWidgets.kToggleButton).getEntry();
    applyBasePower = tab.add("Apply Base Power", false).withWidget(BuiltInWidgets.kToggleButton).getEntry();

    isOnTarget = tab.add("Is On Target", false).withWidget(BuiltInWidgets.kBooleanBox).getEntry();
    currentHeight = tab.add("Current height", 0.0).withWidget(BuiltInWidgets.kTextView).getEntry();
    isPIDControllerEnabled = tab.add("Is PIDController Enabled", false).withWidget(BuiltInWidgets.kBooleanBox)
        .getEntry();

    errorGraph = tab.add("Error Graph", errorGraphParams).withWidget(BuiltInWidgets.kGraph).getEntry();
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    m_lift.setLiftState(LiftState.CALIBRATION);
    Robot.isOperationActive = false;
    Shuffleboard.selectTab("Lift Calibration");
  }

  private void applyManualPower() {
    if (Math.abs(OI.operatorController.getY(Hand.kLeft)) > 0.1 || applyBasePower.getBoolean(false)) {
      if (m_lift.pidController.isEnabled()) {
        m_lift.pidController.disable();
      }
      if (Math.abs(OI.operatorController.getY(Hand.kLeft)) > 0.1) {
        basePower.setBoolean(false);
        m_lift.liftMotor.set(OI.operatorController.getY(Hand.kLeft));
      } else {
        m_lift.liftMotor.set(basePower.getDouble(0.0));
      }
    } else if (!m_lift.pidController.isEnabled()) {
      m_lift.stop();
    }
  }

  private void updatePID() {
    if (enable.getBoolean(false)) {
      enable.setBoolean(false);
      m_lift.DOWN_MOVEMENT_PID_PRESET.setKp(kPdown.getDouble(0.0));
      m_lift.DOWN_MOVEMENT_PID_PRESET.setKi(kIDown.getDouble(0.0));
      m_lift.DOWN_MOVEMENT_PID_PRESET.setKd(kDDown.getDouble(0.0));

      m_lift.UP_MOVEMENT_PID_PRESET.setKp(kPUp.getDouble(0.0));
      m_lift.UP_MOVEMENT_PID_PRESET.setKi(kIUp.getDouble(0.0));
      m_lift.UP_MOVEMENT_PID_PRESET.setKd(kDUp.getDouble(0.0));

      m_lift.pidController.setAbsoluteTolerance(heightTolerance.getDouble(0.0));
      m_lift.VELOCITY_TOLERANCE = velocityTolerance.getDouble(0.0);
      m_lift.GRAVITY_COMPENSATION_POWER = gravityCompensationPower.getDouble(0.0);
      m_lift.RAMP_RATE = rampRate.getDouble(0.0);
      m_lift.setPIDSetpoint(setpoint.getDouble(0.0));
      m_lift.pidController.enable();
    }
  }

  private void updateDisplayValues() {
    isOnTarget.setBoolean(m_lift.isOnTarget());
    currentHeight.setDouble(m_lift.getCurrentHeight());
    isPIDControllerEnabled.setBoolean(m_lift.pidController.isEnabled());
    if (m_lift.pidController.isEnabled()) {
      errorGraphParams[0] = m_lift.pidController.getSetpoint();
      errorGraphParams[1] = m_lift.getCurrentHeight();
      errorGraph.setDoubleArray(errorGraphParams);
    }
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    applyManualPower();
    updatePID();
    updateDisplayValues();
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return false;
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
    m_lift.stop();
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
    end();
  }
}
