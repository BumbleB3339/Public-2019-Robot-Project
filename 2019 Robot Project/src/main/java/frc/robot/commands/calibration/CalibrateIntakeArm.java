/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.calibration;

import static frc.robot.Robot.m_intakeArm;
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
import frc.robot.subsystems.IntakeArm.IntakeArmState;

public class CalibrateIntakeArm extends Command {

  private NetworkTableEntry kP;
  private NetworkTableEntry kI;
  private NetworkTableEntry kD;
  private NetworkTableEntry enable;
  private NetworkTableEntry isOnTarget;
  private NetworkTableEntry setpoint;
  private NetworkTableEntry angleTolerance;
  private NetworkTableEntry velocityTolerance;
  private NetworkTableEntry frictionOvercomePower;
  private NetworkTableEntry gravityF;
  private NetworkTableEntry basePower;
  private NetworkTableEntry applyBasePower;
  private NetworkTableEntry rampRate;
  private NetworkTableEntry currentAngle;
  private NetworkTableEntry potVoltage;
  private NetworkTableEntry errorGraph;
  private NetworkTableEntry isPIDControllerEnabled;
  private NetworkTableEntry applyGravityPower;
  private NetworkTableEntry stickConstantPower;

  private double[] errorGraphParams = new double[2];

  public CalibrateIntakeArm() {
    requires(m_intakeArm);

    placeDashboardWidgets();

    DriverStation.reportError("IntakeArm Calibration Mode is Active!", false);
  }

  private void placeDashboardWidgets() {
    ShuffleboardTab tab = Shuffleboard.getTab("Intake Arm Calibration");

    kP = tab.add("kP", ROBOT_PROFILE.intakeArmParams.pidPreset.getKp()).withWidget(BuiltInWidgets.kTextView).getEntry();
    kI = tab.add("kI", ROBOT_PROFILE.intakeArmParams.pidPreset.getKi()).withWidget(BuiltInWidgets.kTextView).getEntry();
    kD = tab.add("kD", ROBOT_PROFILE.intakeArmParams.pidPreset.getKd()).withWidget(BuiltInWidgets.kTextView).getEntry();

    setpoint = tab.add("Setpoint", 0.0).withWidget(BuiltInWidgets.kTextView).getEntry();
    angleTolerance = tab.add("Angle Tolerance", m_intakeArm.POSITION_TOLERANCE_ANGLE)
        .withWidget(BuiltInWidgets.kTextView).getEntry();
    velocityTolerance = tab.add("Velocity Tolerance", m_intakeArm.VELOCITY_TOLERANCE)
        .withWidget(BuiltInWidgets.kTextView).getEntry();
    frictionOvercomePower = tab.add("Friction Overcome Power", ROBOT_PROFILE.intakeArmParams.frictionOvercomePower)
        .withWidget(BuiltInWidgets.kTextView).getEntry();
    gravityF = tab.add("F Power", ROBOT_PROFILE.intakeArmParams.fPower).withWidget(BuiltInWidgets.kTextView).getEntry();
    basePower = tab.add("Base Power", 0.0).withWidget(BuiltInWidgets.kTextView).getEntry();
    rampRate = tab.add("Ramp Rate", m_intakeArm.RAMP_RATE).withWidget(BuiltInWidgets.kTextView).getEntry();
    stickConstantPower = tab.add("Stick Constant Power", 0.3).withWidget(BuiltInWidgets.kTextView).getEntry();

    enable = tab.add("Enable", false).withWidget(BuiltInWidgets.kToggleButton).getEntry();
    applyBasePower = tab.add("Apply Base Power", false).withWidget(BuiltInWidgets.kToggleButton).getEntry();
    applyGravityPower = tab.add("Apply Gravity Power", false).withWidget(BuiltInWidgets.kToggleButton).getEntry();

    isOnTarget = tab.add("Is On Target", false).withWidget(BuiltInWidgets.kBooleanBox).getEntry();
    currentAngle = tab.add("Current Angle", 0.0).withWidget(BuiltInWidgets.kTextView).getEntry();
    potVoltage = tab.add("Potentiometer Voltage", 0.0).withWidget(BuiltInWidgets.kTextView).getEntry();
    isPIDControllerEnabled = tab.add("Is PIDController Enabled", false).withWidget(BuiltInWidgets.kBooleanBox)
        .getEntry();

    errorGraph = tab.add("Error Graph", errorGraphParams).withWidget(BuiltInWidgets.kGraph).getEntry();
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    m_intakeArm.setPosition(IntakeArmState.CALIBRATION);
    Robot.isOperationActive = false;
    Shuffleboard.selectTab("Intake Arm Calibration");
  }

  private void calibrateGravityPower() {
    if (m_intakeArm.pidController.isEnabled()) {
      m_intakeArm.pidController.disable();
    }

    m_intakeArm.isGravityFCalibrationMode = applyGravityPower.getBoolean(false);

    basePower.setBoolean(false);

    m_intakeArm.calibrationGravityF = gravityF.getDouble(0.0);
    m_intakeArm.frictionOvercomePower = frictionOvercomePower.getDouble(0.0);

    if (Math.abs(OI.operatorController.getY(Hand.kRight)) > 0.3) {
      m_intakeArm
          .setWithBasePower(Math.signum(OI.operatorController.getY(Hand.kRight)) * stickConstantPower.getDouble(0.0));
    } else {
      m_intakeArm.setPower(0.0);
    }
  }

  private void calibrateBasePower() {
    if (m_intakeArm.pidController.isEnabled()) {
      m_intakeArm.pidController.disable();
    }

    m_intakeArm.setPower(basePower.getDouble(0.0));
  }

  private void applyManualPower() {
    if (Math.abs(OI.operatorController.getY(Hand.kRight)) > 0.1) {
      if (m_intakeArm.pidController.isEnabled()) {
        m_intakeArm.pidController.disable();
      }

      m_intakeArm.setPower(OI.operatorController.getY(Hand.kRight));
    } else if (!m_intakeArm.pidController.isEnabled()) {
      m_intakeArm.stop();
    }
  }

  private void setMotorPower() {
    if (applyGravityPower.getBoolean(false)) { // gravity power calibration
      calibrateGravityPower();
    } else if (applyBasePower.getBoolean(false)) { // base power calibration
      calibrateBasePower();
    } else { // manual control of arm
      applyManualPower();
    }
  }

  private void updatePID() {
    if (enable.getBoolean(false)) {
      enable.setBoolean(false);
      m_intakeArm.applyPIDPreset(kP.getDouble(0.0), kI.getDouble(0.0), kD.getDouble(0.0));
      m_intakeArm.pidController.setAbsoluteTolerance(angleTolerance.getDouble(0.0));
      m_intakeArm.VELOCITY_TOLERANCE = velocityTolerance.getDouble(0.0);
      m_intakeArm.frictionOvercomePower = frictionOvercomePower.getDouble(0.0);
      m_intakeArm.GRAVITY_F = gravityF.getDouble(0.0);
      m_intakeArm.intakeArmMotor.configOpenloopRamp(rampRate.getDouble(0.0));
      m_intakeArm.setPIDSetpoint(setpoint.getDouble(0.0));
      m_intakeArm.pidController.enable();
    }
  }

  private void updateDisplayValues() {
    isOnTarget.setBoolean(m_intakeArm.isOnTarget());
    currentAngle.setDouble(m_intakeArm.getActualAngle());
    potVoltage.setDouble(m_intakeArm.intakeArmPotentiometer.getVoltage());
    isPIDControllerEnabled.setBoolean(m_intakeArm.pidController.isEnabled());
    if (m_intakeArm.pidController.isEnabled()) {
      errorGraphParams[0] = m_intakeArm.pidController.getSetpoint();
      errorGraphParams[1] = m_intakeArm.getActualAngle();
      errorGraph.setDoubleArray(errorGraphParams);
    }
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    setMotorPower();
    updatePID();
    updateDisplayValues();
    if (OI.operatorController.getAButton()) {
      Robot.m_intakeRollers.dragRobot();
    } else {
      Robot.m_intakeRollers.stop();
    }
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return false;
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
    m_intakeArm.stop();
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
    end();
  }
}
