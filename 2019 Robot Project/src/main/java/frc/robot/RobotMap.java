/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

/**
 * The RobotMap is a mapping from the ports sensors and actuators are wired into
 * to a variable name. This provides flexibility changing wiring, makes checking
 * the wiring easier and significantly reduces the number of magic numbers
 * floating around.
 */
public class RobotMap {

  public class DrivetrainPorts {
    public static final int FRONT_RIGHT = 20; // CAN SPARK MAX
    public static final int MIDDLE_RIGHT = 21; // CAN SPARK MAX
    public static final int REAR_RIGHT = 22; // CAN SPARK MAX

    public static final int FRONT_LEFT = 24; // CAN SPARK MAX
    public static final int MIDDLE_LEFT = 25; // CAN SPARK MAX
    public static final int REAR_LEFT = 26; // CAN SPARK MAX

    public static final int GEAR_SHIFTER = 5; // PCM solenoid
  }

  public static class VisionPorts {
    public static final int PIXY_SERVO = 9; // PWM
    public static final int CAMERA_SERVO = 8; // PWM

    public static final int LED = 1; // Digital output
  }

  public class IntakeArmPorts {
    public static final int MOTOR = 1; // CAN Talon SRX
    public static final int POTENTIMETER = 4; // Analog input
  }

  public class FloorIntakePorts {
    public static final int MOTOR = 13; // CAN Victor SPX
  }

  public class LiftPorts {
    public static final int MOTOR = 27; // CAN SPARK MAX
    public static final int BOTTOM_SWITCH = 0; // Digital input
  }

  public class CargoHandlerPorts {
    public static final int MOTOR = 10; // CAN Victor SPX
  }

  public class HatchHandlerPorts {
    public static final int HATCH_PUSH_SOLENOID = 7; // PCM solenoid
    public static final int HATCH_HOLD_SOLENOID_REVERSE = 3;
    public static final int HATCH_HOLD_SOLENOID_FORWARD = 2;
  }

  public class LiftArmPorts {
    public static final int MOTOR = 23; // CAN SPARK MAX
    public static final int POTENTIOMETER = 6; // Analog input
  }

  public class ElementArmPorts {
    public static final int MOTOR = 12; // CAN Victor SPX
    public static final int POTENTIOMETER = 5; // Analog input
    public static final int DISTANCE_SENSOR = 7; // Analog input
  }

  public class ClimbPorts {
    public static final int FRONT_RIGHT_MOTOR = 11; // CAN Victor SPX
    public static final int FRONT_LEFT_MOTOR = 14; // CAN Victor SPX

    public static final int REAR_RIGHT_MOTOR = 0; // CAN Talon SRX
    public static final int REAR_LEFT_MOTOR = 2; // CAN Talon SRX

    public static final int FRONT_RIGHT_POTENTIOMETER = 0; // Analog input
    public static final int FRONT_LEFT_POTENTIOMETER = 2; // Analog input

    public static final int REAR_RIGHT_POTENTIOMETER = 1; // Analog input
    public static final int REAR_LEFT_POTENTIOMETER = 3; // Analog input
  }

  // Controller Ports
  public static final int DRIVER_PORT = 1;
  public static final int OPERATOR_PORT = 0;
}
