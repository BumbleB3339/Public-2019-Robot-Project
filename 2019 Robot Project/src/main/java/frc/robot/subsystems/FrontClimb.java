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
import frc.bumblelib.util.hardware.BasePowerWPI_VictorSPX;
import frc.bumblelib.util.hardware.BumblePotentiometer;
import frc.robot.RobotMap;
import frc.robot.profiles.robot_profiles.RobotB;

/**
 * Add your docs here.
 */
public class FrontClimb extends Climb {

  public FrontClimb() {
    super.rightMotor = new BasePowerWPI_VictorSPX(RobotMap.ClimbPorts.FRONT_RIGHT_MOTOR);
    super.rightMotor.setInverted(ROBOT_PROFILE.climbParams.isFrontRightMotorInverted);
    ((BasePowerWPI_VictorSPX) super.rightMotor).setNeutralMode(NeutralMode.Brake);

    super.leftMotor = new BasePowerWPI_VictorSPX(RobotMap.ClimbPorts.FRONT_LEFT_MOTOR);
    super.leftMotor.setInverted(ROBOT_PROFILE.climbParams.isFrontLeftMotorInverted);
    ((BasePowerWPI_VictorSPX) super.leftMotor).setNeutralMode(NeutralMode.Brake);
    if (ROBOT_PROFILE instanceof RobotB) {
      ((BasePowerWPI_VictorSPX) super.leftMotor).setScalingFactor(18.0 / 20.0);
    }

    super.leftPotentiometer = new BumblePotentiometer(RobotMap.ClimbPorts.FRONT_LEFT_POTENTIOMETER,
        ROBOT_PROFILE.climbParams.frontLeftPotentiometerUpperValue,
        ROBOT_PROFILE.climbParams.frontLeftPotentiometerUpperVoltage,
        ROBOT_PROFILE.climbParams.frontLeftPotentiometerLowerValue,
        ROBOT_PROFILE.climbParams.frontLeftPotentiometerLowerVoltage);

    super.rightPotentiometer = new BumblePotentiometer(RobotMap.ClimbPorts.FRONT_RIGHT_POTENTIOMETER,
        ROBOT_PROFILE.climbParams.frontRightPotentiometerUpperValue,
        ROBOT_PROFILE.climbParams.frontRightPotentiometerUpperVoltage,
        ROBOT_PROFILE.climbParams.frontRightPotentiometerLowerValue,
        ROBOT_PROFILE.climbParams.frontRightPotentiometerLowerVoltage);

    rightPIDController = new BumblePIDController(routeKp, 0.0, 0, rightPotentiometer, rightMotor);
    leftPIDController = new BumblePIDController(routeKp, 0.0, 0, leftPotentiometer, leftMotor);

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

  public double getCurrentRightHeight() {
    return rightPotentiometer.get();
  }

  @Override
  public void sendDebuggingData() {
    SmartDashboard.putNumber("Debugging/Climb/FrontLeftClimb Pot Voltage", leftPotentiometer.getVoltage());
    SmartDashboard.putNumber("Debugging/Climb/FrontRightClimb Pot Voltage", rightPotentiometer.getVoltage());

    SmartDashboard.putNumber("Debugging/Climb/FrontLeftClimb Pot Value", leftPotentiometer.get());
    SmartDashboard.putNumber("Debugging/Climb/FrontRightClimb Pot Value", rightPotentiometer.get());
  }

  @Override
  public void setRobotWeightBasePower() {
    ((BasePowerWPI_VictorSPX) super.rightMotor).setGravityCompensationPower(robotWeightBasePower);
    ((BasePowerWPI_VictorSPX) super.leftMotor).setGravityCompensationPower(robotWeightBasePower);
  }

  @Override
  public void setAirBasePower() {
    ((BasePowerWPI_VictorSPX) super.rightMotor).setGravityCompensationPower(-airBasePower);
    ((BasePowerWPI_VictorSPX) super.leftMotor).setGravityCompensationPower(-airBasePower);
  }
}
