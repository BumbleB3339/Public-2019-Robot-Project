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

public class CalibrateLiftArm extends Command {

  private NetworkTableEntry kP;
  private NetworkTableEntry kI;
  private NetworkTableEntry kD;
  private NetworkTableEntry enable;
  private NetworkTableEntry isOnTarget;
  private NetworkTableEntry setpoint;
  private NetworkTableEntry angleTolerance;
  private NetworkTableEntry velocityTolerance;
  private NetworkTableEntry frictionOvercomePower;
  private NetworkTableEntry gravityF_0;
  private NetworkTableEntry gravityF_90;
  private NetworkTableEntry basePower;
  private NetworkTableEntry applyBasePower;
  private NetworkTableEntry rampRate;
  private NetworkTableEntry currentAngle;
  private NetworkTableEntry potVoltage;
  private NetworkTableEntry errorGraph;
  private NetworkTableEntry isPIDControllerEnabled;
  private NetworkTableEntry actualElementArmAngle;
  private NetworkTableEntry currentVelocity;
  private NetworkTableEntry applyGravityPower;
  private NetworkTableEntry gravityPower;
  private NetworkTableEntry stickConstantPower;
  private NetworkTableEntry elementArmSetpoint;
  private NetworkTableEntry enableElementArmPID;

  private double[] errorGraphParams = new double[2];

  public CalibrateLiftArm() {
    requires(m_elementArm);
    requires(m_liftArm);

    placeDashboardWidgets();

    DriverStation.reportError("LiftArm Calibration Mode is Active!", false);
  }

  // Robot.m_liftArm.setWithBasePower(
  // Math.signum(OI.operatorController.getY(Hand.kLeft)) * 0.3);

  private void placeDashboardWidgets() {
    ShuffleboardTab tab = Shuffleboard.getTab("Lift Arm Calibration");
    kP = tab.add("kP", ROBOT_PROFILE.liftArmParams.pidPreset_none.getKp()).withWidget(BuiltInWidgets.kTextView)
        .getEntry();
    kI = tab.add("kI", ROBOT_PROFILE.liftArmParams.pidPreset_none.getKi()).withWidget(BuiltInWidgets.kTextView)
        .getEntry();
    kD = tab.add("kD", ROBOT_PROFILE.liftArmParams.pidPreset_none.getKd()).withWidget(BuiltInWidgets.kTextView)
        .getEntry();

    setpoint = tab.add("Setpoint", 0.0).withWidget(BuiltInWidgets.kTextView).getEntry();
    angleTolerance = tab.add("Angle Tolerance", m_liftArm.POSITION_TOLERANCE_ANGLE).withWidget(BuiltInWidgets.kTextView)
        .getEntry();
    velocityTolerance = tab.add("Velocity Tolerance", m_liftArm.VELOCITY_TOLERANCE).withWidget(BuiltInWidgets.kTextView)
        .getEntry();
    frictionOvercomePower = tab.add("Friction Overcome Power", ROBOT_PROFILE.liftArmParams.frictionOvercomePower)
        .withWidget(BuiltInWidgets.kTextView).getEntry();
    gravityF_0 = tab.add("gravityF_0", ROBOT_PROFILE.liftArmParams.gravityF_0_none).withWidget(BuiltInWidgets.kTextView)
        .getEntry();
    gravityF_90 = tab.add("gravityF_90", ROBOT_PROFILE.liftArmParams.gravityF_90_none)
        .withWidget(BuiltInWidgets.kTextView).getEntry();
    basePower = tab.add("Base Power", 0.0).withWidget(BuiltInWidgets.kTextView).getEntry();
    gravityPower = tab.add("Gravity Power", 0.0).withWidget(BuiltInWidgets.kTextView).getEntry();
    rampRate = tab.add("Ramp Rate", m_liftArm.RAMP_RATE).withWidget(BuiltInWidgets.kTextView).getEntry();
    stickConstantPower = tab.add("Stick Constant Power", 0.3).withWidget(BuiltInWidgets.kTextView).getEntry();
    elementArmSetpoint = tab.add("Element Arm Setpoint", 0.0).withWidget(BuiltInWidgets.kTextView).getEntry();

    enable = tab.add("Enable", false).withWidget(BuiltInWidgets.kToggleButton).getEntry();
    applyBasePower = tab.add("Apply Base Power", false).withWidget(BuiltInWidgets.kToggleButton).getEntry();
    applyGravityPower = tab.add("Apply Gravity Power", false).withWidget(BuiltInWidgets.kToggleButton).getEntry();
    enableElementArmPID = tab.add("Enable Element Arm PID", false).withWidget(BuiltInWidgets.kToggleButton).getEntry();

    isOnTarget = tab.add("Is On Target", false).withWidget(BuiltInWidgets.kBooleanBox).getEntry();
    currentAngle = tab.add("Current Angle", 0.0).withWidget(BuiltInWidgets.kTextView).getEntry();
    potVoltage = tab.add("Potentiometer Voltage", 0.0).withWidget(BuiltInWidgets.kTextView).getEntry();
    isPIDControllerEnabled = tab.add("Is PIDController Enabled", false).withWidget(BuiltInWidgets.kBooleanBox)
        .getEntry();
    actualElementArmAngle = tab.add("Actual Element Arm Angle", 0.0).withWidget(BuiltInWidgets.kTextView).getEntry();
    currentVelocity = tab.add("Current Velocity", 0.0).withWidget(BuiltInWidgets.kTextView).getEntry();

    errorGraph = tab.add("Error Graph", errorGraphParams).withWidget(BuiltInWidgets.kGraph).getEntry();
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    m_elementArm.setElementArmState(ElementArmState.CALIBRATION);
    m_liftArm.setLiftArmState(LiftArmState.CALIBRATION);
    Robot.isOperationActive = false;
    Shuffleboard.selectTab("Lift Arm Calibration");
  }

  private void calibrateGravityPower() {
    if (m_liftArm.pidController.isEnabled()) {
      m_liftArm.pidController.disable();
    }

    m_liftArm.isGravityFCalibrationMode = applyGravityPower.getBoolean(false);

    basePower.setBoolean(false);

    m_liftArm.calibrationGravityF = gravityPower.getDouble(0.0);
    m_liftArm.frictionOvercomePower = frictionOvercomePower.getDouble(0.0);

    if (Math.abs(OI.operatorController.getY(Hand.kLeft)) > 0.3) {
      m_liftArm
          .setWithBasePower(Math.signum(OI.operatorController.getY(Hand.kLeft)) * stickConstantPower.getDouble(0.0));
    } else {
      m_liftArm.setPower(0);
    }
  }

  private void calibrateBasePower() {
    if (m_liftArm.pidController.isEnabled()) {
      m_liftArm.pidController.disable();
    }

    m_liftArm.setPower(basePower.getDouble(0.0));
  }

  private void applyManualPower() {
    if (Math.abs(OI.operatorController.getY(Hand.kLeft)) > 0.2) {
      if (m_liftArm.pidController.isEnabled()) {
        m_liftArm.pidController.disable();
      }

      m_liftArm.setPower(OI.operatorController.getY(Hand.kLeft));
    } else if (!m_liftArm.pidController.isEnabled()) {
      m_liftArm.stop();
    }
  }

  private void moveElementArm() {
    if (Math.abs(OI.operatorController.getY(Hand.kRight)) > 0.2) {
      m_elementArm.pidController.disable();
      m_elementArm.setPower(OI.operatorController.getY(Hand.kRight));
    } else if (!m_elementArm.pidController.isEnabled()) {
      m_elementArm.stop();
    }
  }

  private boolean lastEnableElementArmPID = false;

  private void toggleElementArmPID() {
    if (enableElementArmPID.getBoolean(false) && !lastEnableElementArmPID) {
      m_elementArm.setPIDSetpoint(elementArmSetpoint.getDouble(0.0));
    } else if (!enableElementArmPID.getBoolean(false) && lastEnableElementArmPID) {
      m_elementArm.setElementArmState(ElementArmState.CALIBRATION);
    }
    lastEnableElementArmPID = enableElementArmPID.getBoolean(false);
  }

  private void setMotorsPowers() {
    if (applyGravityPower.getBoolean(false)) { // gravity power calibration
      calibrateGravityPower();
    } else if (applyBasePower.getBoolean(false)) { // base power calibration
      calibrateBasePower();
    } else { // manual control of arm
      applyManualPower();
    }

    moveElementArm();
  }

  private void updatePID() {
    if (enable.getBoolean(false)) {
      enable.setBoolean(false);
      m_liftArm.GRAVITY_F_0_NONE = gravityF_0.getDouble(0.0);
      m_liftArm.GRAVITY_F_90_NONE = gravityF_90.getDouble(0.0);
      m_liftArm.applyPIDPreset(kP.getDouble(0.0), kI.getDouble(0.0), kD.getDouble(0.0));
      m_liftArm.pidController.setAbsoluteTolerance(angleTolerance.getDouble(0.0));
      m_liftArm.VELOCITY_TOLERANCE = velocityTolerance.getDouble(0.0);
      m_liftArm.liftArmMotor.setFrictionOvercomePower(frictionOvercomePower.getDouble(0.0));
      m_liftArm.liftArmMotor.setOpenLoopRampRate(rampRate.getDouble(0.0));
      m_liftArm.setPIDSetpoint(setpoint.getDouble(0.0));
      m_liftArm.pidController.enable();
    }
  }

  private void updateDisplayValues() {
    isOnTarget.setBoolean(m_liftArm.isOnTarget());
    currentAngle.setDouble(m_liftArm.getActualAngle());
    potVoltage.setDouble(m_liftArm.liftArmPotentiometer.getVoltage());
    isPIDControllerEnabled.setBoolean(m_liftArm.pidController.isEnabled());
    actualElementArmAngle.setDouble(m_elementArm.getActualAngle());
    currentVelocity.setDouble(m_liftArm.getCurrentVelocity());
    if (m_liftArm.pidController.isEnabled()) {
      errorGraphParams[0] = m_liftArm.pidController.getSetpoint();
      errorGraphParams[1] = m_liftArm.getActualAngle();
      errorGraph.setDoubleArray(errorGraphParams);
    }
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    setMotorsPowers();
    updatePID();
    updateDisplayValues();
    toggleElementArmPID();
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return false;
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
    m_liftArm.stop();
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
    end();
  }
}
