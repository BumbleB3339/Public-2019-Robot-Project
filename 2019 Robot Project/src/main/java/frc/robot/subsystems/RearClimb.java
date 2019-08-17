/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import static frc.robot.profiles.InitProfiles.ROBOT_PROFILE;

import com.ctre.phoenix.motorcontrol.NeutralMode;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.bumblelib.util.BumblePIDController;
import frc.bumblelib.util.hardware.BasePowerWPI_TalonSRX;
import frc.bumblelib.util.hardware.BumblePotentiometer;
import frc.robot.RobotMap;
import frc.robot.profiles.robot_profiles.RobotB;

/**
 * Add your docs here.
 */
public class RearClimb extends Climb {
  public RearClimb() {
    super.rightMotor = new BasePowerWPI_TalonSRX(RobotMap.ClimbPorts.REAR_RIGHT_MOTOR);
    super.rightMotor.setInverted(ROBOT_PROFILE.climbParams.isBackRightMotorInverted);
    ((BasePowerWPI_TalonSRX) super.rightMotor).setNeutralMode(NeutralMode.Brake);

    super.leftMotor = new BasePowerWPI_TalonSRX(RobotMap.ClimbPorts.REAR_LEFT_MOTOR);
    super.leftMotor.setInverted(ROBOT_PROFILE.climbParams.isBackLeftMotorInverted);
    ((BasePowerWPI_TalonSRX) super.leftMotor).setNeutralMode(NeutralMode.Brake);

    if (ROBOT_PROFILE instanceof RobotB) {
      ((BasePowerWPI_TalonSRX) super.leftMotor).setScalingFactor(18.0 / 20.0);
    }

    super.leftPotentiometer = new BumblePotentiometer(RobotMap.ClimbPorts.REAR_LEFT_POTENTIOMETER,
        ROBOT_PROFILE.climbParams.rearLeftPotentiometerUpperValue,
        ROBOT_PROFILE.climbParams.rearLeftPotentiometerUpperVoltage,
        ROBOT_PROFILE.climbParams.rearLeftPotentiometerLowerValue,
        ROBOT_PROFILE.climbParams.rearLeftPotentiometerLowerVoltage);

    super.rightPotentiometer = new BumblePotentiometer(RobotMap.ClimbPorts.REAR_RIGHT_POTENTIOMETER,
        ROBOT_PROFILE.climbParams.rearRightPotentiometerUpperValue,
        ROBOT_PROFILE.climbParams.rearRightPotentiometerUpperVoltage,
        ROBOT_PROFILE.climbParams.rearRightPotentiometerLowerValue,
        ROBOT_PROFILE.climbParams.rearRightPotentiometerLowerVoltage);

    rightPIDController = new BumblePIDController(routeKp, 0, 0, rightPotentiometer, rightMotor);
    leftPIDController = new BumblePIDController(routeKp, 0, 0, leftPotentiometer, leftMotor);

    rightPIDController.setOutputRange(-MAX_POWER_DOWN, MAX_POWER_UP);
    leftPIDController.setOutputRange(-MAX_POWER_DOWN, MAX_POWER_UP);

    rightPIDController.setAbsoluteTolerance(TOLERANCE);
    leftPIDController.setAbsoluteTolerance(TOLERANCE);
  }

  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
  }

  @Override
  public void sendDebuggingData() {
    SmartDashboard.putNumber("Debugging/Climb/RearLeftClimb Pot Voltage", leftPotentiometer.getVoltage());
    SmartDashboard.putNumber("Debugging/Climb/RearRightClimb Pot Voltage", rightPotentiometer.getVoltage());

    SmartDashboard.putNumber("Debugging/Climb/RearLeftClimb Pot Value", leftPotentiometer.get());
    SmartDashboard.putNumber("Debugging/Climb/RearRightClimb Pot Value", rightPotentiometer.get());
  }

  @Override
  public void setRobotWeightBasePower() {
    ((BasePowerWPI_TalonSRX) super.rightMotor).setGravityCompensationPower(robotWeightBasePower);
    ((BasePowerWPI_TalonSRX) super.leftMotor).setGravityCompensationPower(robotWeightBasePower);
  }

  @Override
  public void setAirBasePower() {
    ((BasePowerWPI_TalonSRX) super.rightMotor).setGravityCompensationPower(-airBasePower);
    ((BasePowerWPI_TalonSRX) super.leftMotor).setGravityCompensationPower(-airBasePower);
  }
}
