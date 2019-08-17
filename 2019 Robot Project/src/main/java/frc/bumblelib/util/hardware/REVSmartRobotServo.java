/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.bumblelib.util.hardware;

import edu.wpi.first.hal.FRCNetComm.tResourceType;
import edu.wpi.first.hal.HAL;
import edu.wpi.first.wpilibj.PWM;
import edu.wpi.first.wpilibj.smartdashboard.SendableBuilder;


/**
 * REV Robotics Smart Robot Servo.
 */
public class REVSmartRobotServo extends PWM {
    private double kMaxServoAngle = 135.0;
    private double kMinServoAngle = -135.0;
  
    private double kMaxServoPWM = 2.5;
    private double kMinServoPWM = .5;
  
    /**
     * Constructor.<br>
     *
     * <p>By default {@value #kMaxServoPWM} ms is used as the maxPWM value<br> By default
     * {@value #kMinServoPWM} ms is used as the minPWM value<br>
     *
     * @param channel The PWM channel to which the servo is attached. 0-9 are on-board, 10-19 are on
     *                the MXP port
     */
    public REVSmartRobotServo(final int channel) {
      super(channel);
      setBounds(kMaxServoPWM, 0, 0, 0, kMinServoPWM);
      setPeriodMultiplier(PeriodMultiplier.k4X);
  
      HAL.report(tResourceType.kResourceType_Servo, getChannel());
      setName("Servo", getChannel());
    }

    /**
     * Constructor.<br>
     *
     * <p>By default {@value #kMaxServoPWM} ms is used as the maxPWM value<br> By default
     * {@value #kMinServoPWM} ms is used as the minPWM value<br>
     *
     * @param channel The PWM channel to which the servo is attached. 0-9 are on-board, 10-19 are on
     *                the MXP port
     */
    public REVSmartRobotServo(final int channel, double edgeServoAngle, double minServoPWM, double maxServoPWM) {
      super(channel);

      this.kMaxServoAngle = edgeServoAngle;
      this.kMinServoAngle = -edgeServoAngle;
      this.kMinServoPWM = minServoPWM;
      this.kMaxServoPWM = maxServoPWM;

      setBounds(kMaxServoPWM, 0, 0, 0, kMinServoPWM);
      setPeriodMultiplier(PeriodMultiplier.k4X);
  
      HAL.report(tResourceType.kResourceType_Servo, getChannel());
      setName("Servo", getChannel());
    }
  
  
    /**
     * Set the servo position.
     *
     * <p>Servo values range from 0.0 to 1.0 corresponding to the range of full left to full right.
     *
     * @param value Position from 0.0 to 1.0.
     */
    public void set(double value) {
      setPosition(value);
    }
  
    /**
     * Get the servo position.
     *
     * <p>Servo values range from 0.0 to 1.0 corresponding to the range of full left to full right.
     *
     * @return Position from 0.0 to 1.0.
     */
    public double get() {
      return getPosition();
    }
  
    /**
     * Set the servo angle.
     *
     * <p>Assume that the servo angle is linear with respect to the PWM value (big assumption, need to
     * test).
     *
     * <p>Servo angles that are out of the supported range of the servo simply "saturate" in that
     * direction In other words, if the servo has a range of (X degrees to Y degrees) than angles of
     * less than X result in an angle of X being set and angles of more than Y degrees result in an
     * angle of Y being set.
     *
     * @param degrees The angle in degrees to set the servo.
     */
    public void setAngle(double degrees) {
      if (degrees < kMinServoAngle) {
        degrees = kMinServoAngle;
      } else if (degrees > kMaxServoAngle) {
        degrees = kMaxServoAngle;
      }
  
      double positionToSet = ((degrees - kMinServoAngle)) / getServoAngleRange();

      // Because it does not accept a value of 0.0;
      if (positionToSet < 0.01) {
          positionToSet = 0.01;
      }

      setPosition(positionToSet);
    }
  
    /**
     * Get the servo angle.
     *
     * <p>Assume that the servo angle is linear with respect to the PWM value (big assumption, need to
     * test).
     *
     * @return The angle in degrees to which the servo is set.
     */
    public double getAngle() {
      return getPosition() * getServoAngleRange() + kMinServoAngle;
    }
  
    private double getServoAngleRange() {
      return kMaxServoAngle - kMinServoAngle;
    }
  
    @Override
    public void initSendable(SendableBuilder builder) {
      builder.setSmartDashboardType("Servo");
      builder.addDoubleProperty("Value", this::get, this::set);
    }
}
