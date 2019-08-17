/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.calibration;

import static frc.robot.Robot.m_elementArm;
import static frc.robot.Robot.m_liftArm;
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
import frc.robot.subsystems.ElementArm.ElementArmState;
import frc.robot.subsystems.LiftArm.LiftArmState;

public class CalibrateElementArm extends Command {

  private NetworkTableEntry kP;
  private NetworkTableEntry kI;
  private NetworkTableEntry kD;
  private NetworkTableEntry enable;
  private NetworkTableEntry isOnTarget;
  private NetworkTableEntry setpoint;
  private NetworkTableEntry angleTolerance;
  private NetworkTableEntry velocityTolerance;
  private NetworkTableEntry frictionOvercomePower;
  private NetworkTableEntry basePower;
  private NetworkTableEntry applyBasePower;
  private NetworkTableEntry rampRate;
  private NetworkTableEntry currentAngle;
  private NetworkTableEntry potVoltage;
  private NetworkTableEntry errorGraph;
  private NetworkTableEntry isPIDControllerEnabled;
  private NetworkTableEntry absoluteAngle;
  private NetworkTableEntry applyGravityPower;
  private NetworkTableEntry gravityF;
  private NetworkTableEntry stickConstantPower;

  private double[] errorGraphParams = new double[2];

  public CalibrateElementArm() {
    requires(m_elementArm);
    requires(m_liftArm);

    placeDashboardWidgets();

    DriverStation.reportError("ElementArm Calibration Mode is Active!", false);
  }

  private void placeDashboardWidgets() {
    ShuffleboardTab tab = Shuffleboard.getTab("Element Arm Calibration");
    kP = tab.add("kP", ROBOT_PROFILE.elementArmParams.pidPreset_none.getKp()).withWidget(BuiltInWidgets.kTextView)
        .getEntry();
    kI = tab.add("kI", ROBOT_PROFILE.elementArmParams.pidPreset_none.getKi()).withWidget(BuiltInWidgets.kTextView)
        .getEntry();
    kD = tab.add("kD", ROBOT_PROFILE.elementArmParams.pidPreset_none.getKd()).withWidget(BuiltInWidgets.kTextView)
        .getEntry();

    setpoint = tab.add("Setpoint", 0.0).withWidget(BuiltInWidgets.kTextView).getEntry();
    angleTolerance = tab.add("Angle Tolerance", m_elementArm.POSITION_TOLERANCE_ANGLE)
        .withWidget(BuiltInWidgets.kTextView).getEntry();
    velocityTolerance = tab.add("Velocity Tolerance", m_elementArm.VELOCITY_TOLERANCE)
        .withWidget(BuiltInWidgets.kTextView).getEntry();
    frictionOvercomePower = tab.add("Friction Overcome Power", ROBOT_PROFILE.elementArmParams.frictionOvercomePower)
        .withWidget(BuiltInWidgets.kTextView).getEntry();
    basePower = tab.add("Base Power", 0.0).withWidget(BuiltInWidgets.kTextView).getEntry();
    gravityF = tab.add("Gravity F", 0.0).withWidget(BuiltInWidgets.kTextView).getEntry();
    rampRate = tab.add("Ramp Rate", m_elementArm.RAMP_RATE).withWidget(BuiltInWidgets.kTextView).getEntry();
    stickConstantPower = tab.add("Stick Constant Power", 0.3).withWidget(BuiltInWidgets.kTextView).getEntry();

    enable = tab.add("Enable", false).withWidget(BuiltInWidgets.kToggleButton).getEntry();
    applyBasePower = tab.add("Apply Base Power", false).withWidget(BuiltInWidgets.kToggleButton).getEntry();
    applyGravityPower = tab.add("Apply Gravity Power", false).withWidget(BuiltInWidgets.kToggleButton).getEntry();

    isOnTarget = tab.add("Is On Target", false).withWidget(BuiltInWidgets.kBooleanBox).getEntry();
    currentAngle = tab.add("Current Angle", 0.0).withWidget(BuiltInWidgets.kTextView).getEntry();
    potVoltage = tab.add("Potentiometer Voltage", 0.0).withWidget(BuiltInWidgets.kTextView).getEntry();
    isPIDControllerEnabled = tab.add("Is PIDController Enabled", false).withWidget(BuiltInWidgets.kBooleanBox)
        .getEntry();
    absoluteAngle = tab.add("Absolute Angle", 0.0).withWidget(BuiltInWidgets.kTextView).getEntry();

    errorGraph = tab.add("Error Graph", errorGraphParams).withWidget(BuiltInWidgets.kGraph).getEntry();
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    m_elementArm.setElementArmState(ElementArmState.CALIBRATION);
    m_liftArm.setLiftArmState(LiftArmState.CALIBRATION);
    Robot.isOperationActive = false;
    Shuffleboard.selectTab("Element Arm Calibration");
  }

  private void calibrateGravityPower() {
    if (m_elementArm.pidController.isEnabled()) {
      m_elementArm.pidController.disable();
    }

    m_elementArm.isGravityFCalibrationMode = applyGravityPower.getBoolean(false);

    basePower.setBoolean(false);

    m_elementArm.calibrationGravityF = gravityF.getDouble(0.0);
    m_elementArm.frictionOvercomePower = frictionOvercomePower.getDouble(0.0);

    if (Math.abs(OI.operatorController.getY(Hand.kRight)) > 0.3) {
      m_elementArm
          .setWithBasePower(Math.signum(OI.operatorController.getY(Hand.kRight)) * stickConstantPower.getDouble(0.0));
    } else {
      m_elementArm.setPower(0.0);
    }
  }

  private void calibrateBasePower() {
    if (m_elementArm.pidController.isEnabled()) {
      m_elementArm.pidController.disable();
    }

    m_elementArm.setPower(basePower.getDouble(0.0));
  }

  private void applyManualPower() {
    if (Math.abs(OI.operatorController.getY(Hand.kRight)) > 0.1) {
      if (m_elementArm.pidController.isEnabled()) {
        m_elementArm.pidController.disable();
      }

      m_elementArm.setPower(OI.operatorController.getY(Hand.kRight));
    } else if (!m_elementArm.pidController.isEnabled()) {
      m_elementArm.stop();
    }
  }

  private void moveLiftArm() {
    if (Math.abs(OI.operatorController.getY(Hand.kLeft)) > 0.1) {
      m_liftArm.setPower(OI.operatorController.getY(Hand.kLeft));
    } else {
      m_liftArm.stop();
    }
  }

  private void setMotorsPowers() {
    if (applyGravityPower.getBoolean(false)) { // gravity power calibration
      calibrateGravityPower();
    } else if (applyBasePower.getBoolean(false)) { // base power calibration
      calibrateBasePower();
    } else { // manual control of arm
      applyManualPower();
    }

    moveLiftArm();
  }

  private void updatePID() {
    if (enable.getBoolean(false)) {
      enable.setBoolean(false);
      m_elementArm.applyPIDPreset(kP.getDouble(0.0), kI.getDouble(0.0), kD.getDouble(0.0));
      m_elementArm.pidController.setAbsoluteTolerance(angleTolerance.getDouble(0.0));
      m_elementArm.VELOCITY_TOLERANCE = velocityTolerance.getDouble(0.0);
      m_elementArm.frictionOvercomePower = frictionOvercomePower.getDouble(0.0);
      m_elementArm.GRAVITY_F_NONE = gravityF.getDouble(0.0);
      m_elementArm.elementArmMotor.setFrictionOvercomePower(frictionOvercomePower.getDouble(0.0));
      m_elementArm.elementArmMotor.configOpenloopRamp(rampRate.getDouble(0.0));
      m_elementArm.setPIDSetpoint(setpoint.getDouble(0.0));
      m_elementArm.pidController.enable();
    }
  }

  private void updateDisplayValues() {
    isOnTarget.setBoolean(m_elementArm.isOnTarget());
    currentAngle.setDouble(m_elementArm.getActualAngle());
    potVoltage.setDouble(m_elementArm.elementArmPotentiometer.getVoltage());
    isPIDControllerEnabled.setBoolean(m_elementArm.pidController.isEnabled());
    absoluteAngle.setDouble(m_elementArm.getAbsoluteAngle());
    if (m_elementArm.pidController.isEnabled()) {
      errorGraphParams[0] = m_elementArm.pidController.getSetpoint();
      errorGraphParams[1] = m_elementArm.getActualAngle();
      errorGraph.setDoubleArray(errorGraphParams);
    }
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    setMotorsPowers();
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
    m_elementArm.stop();
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
    end();
  }
}
