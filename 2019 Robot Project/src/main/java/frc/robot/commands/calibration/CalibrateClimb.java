/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.calibration;

import static frc.robot.Robot.m_drivetrain;
import static frc.robot.Robot.m_frontClimb;
import static frc.robot.Robot.m_intakeArm;
import static frc.robot.Robot.m_intakeRollers;
import static frc.robot.Robot.m_rearClimb;
import static frc.robot.profiles.InitProfiles.ROBOT_PROFILE;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import frc.bumblelib.util.hardware.BasePowerWPI_TalonSRX;
import frc.bumblelib.util.hardware.BasePowerWPI_VictorSPX;
import frc.robot.OI;
import frc.robot.Robot;
import frc.robot.commands.climb.AllClimb;
import frc.robot.commands.climb.CommandClimbLevel3;
import frc.robot.subsystems.Drivetrain.Gear;
import frc.robot.subsystems.IntakeArm.IntakeArmState;

public class CalibrateClimb extends Command {

  private NetworkTableEntry frontRightVoltage;
  private NetworkTableEntry frontLeftVoltage;
  private NetworkTableEntry rearRightVoltage;
  private NetworkTableEntry rearLeftVoltage;
  private NetworkTableEntry climbManualPower;
  private NetworkTableEntry startAllClimb;
  private NetworkTableEntry allClimbSetpoint;

  private NetworkTableEntry enableDrivetrain;
  private NetworkTableEntry drivetrainPower;
  private NetworkTableEntry enableIntakeArm;
  private NetworkTableEntry intakeArmPower;
  private NetworkTableEntry enableIntakeRollers;
  private NetworkTableEntry intakeRollersPower;

  private NetworkTableEntry basePower;
  private NetworkTableEntry routeKp;
  private NetworkTableEntry finalKp;
  private NetworkTableEntry rearLeftPower;
  private NetworkTableEntry frontLeftPower;
  private NetworkTableEntry rearRightPower;
  private NetworkTableEntry frontRightPower;

  private NetworkTableEntry rearLeftPIDEnabled;
  private NetworkTableEntry frontLeftPIDEnabled;
  private NetworkTableEntry rearRightPIDEnabled;
  private NetworkTableEntry frontRightPIDEnabled;

  private Command currentRunningCommand;

  public CalibrateClimb() {
    requires(m_intakeArm);
    requires(m_intakeRollers);
    requires(m_drivetrain);

    placeDashboardWidgets();

    DriverStation.reportError("Climb Calibration Mode is Active!", false);
  }

  private void placeDashboardWidgets() {
    ShuffleboardTab tab = Shuffleboard.getTab("Climb Calibration");
    frontRightVoltage = tab.add("Front Right Voltage", 0.0).withWidget(BuiltInWidgets.kTextView).getEntry();
    frontLeftVoltage = tab.add("Front Left Voltage", 0.0).withWidget(BuiltInWidgets.kTextView).getEntry();
    rearRightVoltage = tab.add("Rear Right Voltage", 0.0).withWidget(BuiltInWidgets.kTextView).getEntry();
    rearLeftVoltage = tab.add("Rear Left Voltage", 0.0).withWidget(BuiltInWidgets.kTextView).getEntry();

    climbManualPower = tab.add("Climb Manual Power", 0.0).withWidget(BuiltInWidgets.kTextView).getEntry();

    startAllClimb = tab.add("Start All Climb", false).withWidget(BuiltInWidgets.kToggleButton).getEntry();
    allClimbSetpoint = tab.add("All Climb", 0.0).withWidget(BuiltInWidgets.kTextView).getEntry();

    enableDrivetrain = tab.add("Enable Drivetrain", false).withWidget(BuiltInWidgets.kToggleButton).getEntry();
    drivetrainPower = tab.add("Drivetrain Power", CommandClimbLevel3.DRIVE_ROBOT_POWER)
        .withWidget(BuiltInWidgets.kTextView).getEntry();

    enableIntakeArm = tab.add("Enable Intake Arm", false).withWidget(BuiltInWidgets.kToggleButton).getEntry();
    intakeArmPower = tab.add("Intake Arm Power", ROBOT_PROFILE.intakeArmParams.supportRobotPower)
        .withWidget(BuiltInWidgets.kTextView).getEntry();

    enableIntakeRollers = tab.add("Enable Intake Rollers", false).withWidget(BuiltInWidgets.kToggleButton).getEntry();
    intakeRollersPower = tab.add("Intake Rollers Power", ROBOT_PROFILE.floorRollerParams.dragPower)
        .withWidget(BuiltInWidgets.kTextView).getEntry();

    rearLeftPower = tab.add("Rear Left Power", 0.0).withWidget(BuiltInWidgets.kTextView).getEntry();
    frontLeftPower = tab.add("Front Left Power", 0.0).withWidget(BuiltInWidgets.kTextView).getEntry();
    rearRightPower = tab.add("Rear Right Power", 0.0).withWidget(BuiltInWidgets.kTextView).getEntry();
    frontRightPower = tab.add("Front Right Power", 0.0).withWidget(BuiltInWidgets.kTextView).getEntry();

    basePower = tab.add("Base Power", 0.0).withWidget(BuiltInWidgets.kTextView).getEntry();
    routeKp = tab.add("Route Kp Power", m_frontClimb.routeKp).withWidget(BuiltInWidgets.kTextView).getEntry();
    finalKp = tab.add("Final Kp Power", m_frontClimb.finalKp).withWidget(BuiltInWidgets.kTextView).getEntry();

    rearLeftPIDEnabled = tab.add("Rear Left PID Enabled", false).withWidget(BuiltInWidgets.kBooleanBox).getEntry();
    frontLeftPIDEnabled = tab.add("Front Left PID Enabled", false).withWidget(BuiltInWidgets.kBooleanBox).getEntry();
    rearRightPIDEnabled = tab.add("Rear Right PID Enabled", false).withWidget(BuiltInWidgets.kBooleanBox).getEntry();
    frontRightPIDEnabled = tab.add("Front Right PID Enabled", false).withWidget(BuiltInWidgets.kBooleanBox).getEntry();
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    Robot.isOperationActive = false;
    m_intakeArm.setPosition(IntakeArmState.CALIBRATION);
    m_drivetrain.setGear(Gear.POWER_GEAR);
    Shuffleboard.selectTab("Climb Calibration");
  }

  private void updateDashboard() {
    frontRightVoltage.setDouble(m_frontClimb.rightPotentiometer.getVoltage());
    frontLeftVoltage.setDouble(m_frontClimb.leftPotentiometer.getVoltage());
    rearRightVoltage.setDouble(m_rearClimb.rightPotentiometer.getVoltage());
    rearLeftVoltage.setDouble(m_rearClimb.leftPotentiometer.getVoltage());

    frontRightPower.setDouble(((BasePowerWPI_VictorSPX) (m_frontClimb.rightMotor)).get());
    frontLeftPower.setDouble(((BasePowerWPI_VictorSPX) (m_frontClimb.leftMotor)).get());
    rearRightPower.setDouble(((BasePowerWPI_TalonSRX) (m_rearClimb.rightMotor)).get());
    rearLeftPower.setDouble(((BasePowerWPI_TalonSRX) (m_rearClimb.leftMotor)).get());

    frontRightPIDEnabled.setBoolean(m_frontClimb.rightPIDController.isEnabled());
    frontLeftPIDEnabled.setBoolean(m_frontClimb.leftPIDController.isEnabled());
    rearRightPIDEnabled.setBoolean(m_rearClimb.rightPIDController.isEnabled());
    rearLeftPIDEnabled.setBoolean(m_rearClimb.leftPIDController.isEnabled());
  }

  private void setMotorPower() {
    if (OI.operatorController.getTriggerAxis(Hand.kRight) > 0.2) {
      if (currentRunningCommand != null) {
        currentRunningCommand.cancel();
      }
      m_frontClimb.rightMotor.set(climbManualPower.getDouble(0.0));
    } else if (currentRunningCommand == null || !currentRunningCommand.isRunning()) {
      m_frontClimb.rightMotor.set(0);
    }

    if (OI.operatorController.getTriggerAxis(Hand.kLeft) > 0.2) {
      if (currentRunningCommand != null) {
        currentRunningCommand.cancel();
      }
      m_frontClimb.leftMotor.set(climbManualPower.getDouble(0.0));
    } else if (currentRunningCommand == null || !currentRunningCommand.isRunning()) {
      m_frontClimb.leftMotor.set(0);
    }

    if (OI.operatorController.getBumper(Hand.kRight)) {
      if (currentRunningCommand != null) {
        currentRunningCommand.cancel();
      }
      m_rearClimb.rightMotor.set(climbManualPower.getDouble(0.0));
    } else if (currentRunningCommand == null || !currentRunningCommand.isRunning()) {
      m_rearClimb.rightMotor.set(0);
    }

    if (OI.operatorController.getBumper(Hand.kLeft)) {
      if (currentRunningCommand != null) {
        currentRunningCommand.cancel();
      }
      m_rearClimb.leftMotor.set(climbManualPower.getDouble(0.0));
    } else if (currentRunningCommand == null || !currentRunningCommand.isRunning()) {
      m_rearClimb.leftMotor.set(0);
    }
  }

  private void startCommands() {
    if (startAllClimb.getBoolean(false)) {
      startAllClimb.setBoolean(false);
      if (currentRunningCommand != null) {
        currentRunningCommand.cancel();
      }

      m_frontClimb.finalKp = finalKp.getDouble(0.0);
      m_frontClimb.routeKp = routeKp.getDouble(0.0);
      m_rearClimb.finalKp = finalKp.getDouble(0.0);
      m_rearClimb.routeKp = routeKp.getDouble(0.0);

      m_frontClimb.airBasePower = basePower.getDouble(0.0);
      m_frontClimb.robotWeightBasePower = basePower.getDouble(0.0);
      m_rearClimb.airBasePower = basePower.getDouble(0.0);
      m_rearClimb.robotWeightBasePower = basePower.getDouble(0.0);

      currentRunningCommand = new AllClimb(allClimbSetpoint.getDouble(0.0));
      currentRunningCommand.start();
    }
  }

  private void setExternalSubsystemPowers() {
    if (enableDrivetrain.getBoolean(false)) {
      m_drivetrain.setLeftRightMotorOutputs(drivetrainPower.getDouble(0.0), drivetrainPower.getDouble(0.0));
    } else {
      m_drivetrain.setLeftRightMotorOutputs(0.0, 0.0);
    }

    if (enableIntakeArm.getBoolean(false)) {
      m_intakeArm.setPower(intakeArmPower.getDouble(0.0));
    } else {
      m_intakeArm.setPower(0.0);
    }

    if (enableIntakeRollers.getBoolean(false)) {
      m_intakeRollers.setPower(intakeRollersPower.getDouble(0.0));
    } else {
      m_intakeRollers.setPower(0.0);
    }
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    updateDashboard();
    setMotorPower();
    startCommands();
    setExternalSubsystemPowers();
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return false;
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
    m_frontClimb.stop();
    m_rearClimb.stop();
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
    end();
  }
}
