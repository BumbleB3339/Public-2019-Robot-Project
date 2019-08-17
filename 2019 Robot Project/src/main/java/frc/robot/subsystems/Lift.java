/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import static frc.robot.profiles.InitProfiles.ROBOT_PROFILE;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.PIDSource;
import edu.wpi.first.wpilibj.PIDSourceType;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.bumblelib.util.BumbleTimer;
import frc.bumblelib.util.PIDPreset;
import frc.bumblelib.util.SmartdashboardDebugging;
import frc.bumblelib.util.StableBoolean;
import frc.bumblelib.util.hardware.BasePowerCANSparkMax;
import frc.bumblelib.util.hardware.BumbleSwitch;
import frc.robot.OI;
import frc.robot.Robot;
import frc.robot.RobotGameStateManager;
import frc.robot.RobotMap;
import frc.robot.commands.controller.CommandTimedControllerRumble;
import frc.robot.commands.controller.CommandTimedControllerRumble.RumbleLength;

/**
 * Add your docs here.
 */
public class Lift extends Subsystem implements SmartdashboardDebugging {

  public BasePowerCANSparkMax liftMotor;
  private BumbleSwitch bottomSwitch;

  private final int ON_TARGET_TRUE_THRESHOLD = 5;
  private final int ON_TARGET_FALSE_THRESHOLD = 5;
  private StableBoolean onTarget = new StableBoolean(ON_TARGET_TRUE_THRESHOLD, ON_TARGET_FALSE_THRESHOLD);

  private double currentHeightSetpoint = -3339;

  private final boolean IS_MOTOR_INVERTED = ROBOT_PROFILE.liftParams.isMotorInverted;
  private final IdleMode DEFAULT_IDLE_MODE = IdleMode.kBrake;
  private final int SMART_CURRENT_LIMIT = 40;

  // Ticks to cm parameters
  private final int COUNTS_PER_REVOLUTION = 4096;
  private final double GEAR_RATIO = 18.0 / 18;
  private final double HTD_TIMING_PULLEY_DIAMETER = 6; // In cm

  private WPI_TalonSRX sensorMotorController = (WPI_TalonSRX) Robot.m_rearClimb.leftMotor;

  // On target params
  public double POSITION_TOLERANCE_HEIGHT = ROBOT_PROFILE.liftParams.positionToleranceHeight;
  public double VELOCITY_TOLERANCE = 100000;

  private final double MANUAL_POWER_UP = ROBOT_PROFILE.liftParams.manualPowerUp;
  private final double MANUAL_POWER_DOWN = ROBOT_PROFILE.liftParams.manualPowerDown;
  public double GRAVITY_COMPENSATION_POWER = ROBOT_PROFILE.liftParams.gravityCompensationPower;
  private final double RESET_MODE_POWER_DOWN = ROBOT_PROFILE.liftParams.resetModePowerDown; // TODO: calibrate for real
                                                                                            // time
  public PIDPreset UP_MOVEMENT_PID_PRESET = ROBOT_PROFILE.liftParams.upMovementPIDPreset;
  public PIDPreset DOWN_MOVEMENT_PID_PRESET = ROBOT_PROFILE.liftParams.downMovementPIDPreset;

  public PIDController pidController;

  private final LiftState DEFAULT_LIFT_STATE = LiftState.MANUAL;

  private boolean isBottomSwitchDefective = ROBOT_PROFILE.liftParams.isBottomSwitchDefective;
  private final double MAX_RESET_TIME = 5; // In seconds // TODO: calibrate for real time

  public double RAMP_RATE = 0.1;

  private PIDSource liftEncoder = new PIDSource() {

    @Override
    public void setPIDSourceType(PIDSourceType pidSource) {
    }

    @Override
    public double pidGet() {
      return getCurrentHeight();
    }

    @Override
    public PIDSourceType getPIDSourceType() {
      return PIDSourceType.kDisplacement;
    }
  };

  private LiftState currentLiftState = DEFAULT_LIFT_STATE;
  private final double DEFAULT_LIFT_HEIGHT = RobotGameStateManager.currentGameState.robotSystemState.robotConfiguration.liftHeight;

  public enum LiftState {
    MANUAL, //
    CALIBRATION, //
    RESET, // In this mode, the lift automatically travels down to reset itself.
    FOLLOWING_CURRENT_STATE, SEMI_AUTOMATIC;

    public String toDashboardString() {
      switch (this) {
      case MANUAL:
        return "Manual";
      case CALIBRATION:
        return "Calibration";
      case SEMI_AUTOMATIC:
        return "Semi Automatic";
      default:
        return "Automatic";
      }
    }
  }

  public Lift() {
    liftMotor = new BasePowerCANSparkMax(RobotMap.LiftPorts.MOTOR, MotorType.kBrushless);
    liftMotor.setGravityCompensationPower(GRAVITY_COMPENSATION_POWER);
    liftMotor.restoreFactoryDefaults();
    liftMotor.setInverted(IS_MOTOR_INVERTED);
    liftMotor.setIdleMode(DEFAULT_IDLE_MODE);
    liftMotor.enableVoltageCompensation(11);
    liftMotor.setSmartCurrentLimit(SMART_CURRENT_LIMIT);
    liftMotor.setOpenLoopRampRate(RAMP_RATE);
    resetEncoderTicks();

    bottomSwitch = new BumbleSwitch(RobotMap.LiftPorts.BOTTOM_SWITCH,
        ROBOT_PROFILE.liftParams.bottomSwitchNormallyClosed);

    pidController = new PIDController(UP_MOVEMENT_PID_PRESET.getKp(), UP_MOVEMENT_PID_PRESET.getKi(),
        UP_MOVEMENT_PID_PRESET.getKd(), liftEncoder, liftMotor);
    pidController.disable();
    pidController.setAbsoluteTolerance(POSITION_TOLERANCE_HEIGHT);
    pidController.setSetpoint(DEFAULT_LIFT_HEIGHT);
    setLiftState(isBottomSwitchDefective ? DEFAULT_LIFT_STATE : LiftState.RESET);

    sensorMotorController.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder);
    sensorMotorController.setSensorPhase(ROBOT_PROFILE.liftParams.sensorPhase);
  }

  @Override
  public void initDefaultCommand() {
    // nothing
  }

  public void stop() {
    liftMotor.set(0);
  }

  public void manualLiftUp() {
    liftMotor.set(MANUAL_POWER_UP);
  }

  public void manualLiftDown() {
    liftMotor.set(MANUAL_POWER_DOWN);
  }

  public void resetModeLiftDown() {
    liftMotor.set(RESET_MODE_POWER_DOWN);
  }

  public void applyBaseBower() {
    liftMotor.set(GRAVITY_COMPENSATION_POWER);
  }

  private void applyPIDPreset(PIDPreset pidPreset) {
    pidController.setP(pidPreset.getKp());
    pidController.setI(pidPreset.getKi());
    pidController.setD(pidPreset.getKd());
  }

  public void setPIDSetpoint(double heightSetpoint) {
    if (getCurrentLiftState() == LiftState.MANUAL) {
      // don't allow changing PID setpoint on manual
      return;
    }

    // Don't toggle between up and down pid gains when close to target
    if (Math.abs(heightSetpoint - getCurrentHeight()) > POSITION_TOLERANCE_HEIGHT) {
      if (heightSetpoint >= getCurrentHeight()) {
        applyPIDPreset(UP_MOVEMENT_PID_PRESET);
      } else {
        applyPIDPreset(DOWN_MOVEMENT_PID_PRESET);
      }
    }

    if (currentHeightSetpoint != heightSetpoint) {
      onTarget.forceValue(false);
    }
    this.currentHeightSetpoint = heightSetpoint;
    pidController.setSetpoint(heightSetpoint);
  }

  public void setLiftState(LiftState liftState) {
    this.currentLiftState = liftState;
    if (getCurrentLiftState() == LiftState.FOLLOWING_CURRENT_STATE) {
      pidController.enable();
    } else {
      pidController.disable();
    }
    if (liftState == LiftState.SEMI_AUTOMATIC) {
      RobotGameStateManager.setRobotSystemStateToSemiAuto();
    }
  }

  public LiftState getCurrentLiftState() {
    return currentLiftState;
  }

  private double getHeightForTicks(double ticks) {
    double circumference = Math.PI * HTD_TIMING_PULLEY_DIAMETER;
    double bigWheelRotations = (double) ticks / COUNTS_PER_REVOLUTION * GEAR_RATIO;
    double height = bigWheelRotations * circumference;
    return height;
  }

  private int getTicksForHeight(double height) {
    double circumference = Math.PI * HTD_TIMING_PULLEY_DIAMETER;
    double bigWheelRotations = height / circumference;
    double ticks = (bigWheelRotations / GEAR_RATIO) * COUNTS_PER_REVOLUTION;
    return (int) ticks;
  }

  public double getCurrentHeight() {
    return getHeightForTicks(getCurrentTicks());
  }

  public int getCurrentTicks() {
    return sensorMotorController.getSelectedSensorPosition();
  }

  public double getCurrentVelocity() {
    return getHeightForTicks(sensorMotorController.getSelectedSensorVelocity() * 10); // Multiplied by 10 to convert to
                                                                                      // seconds from 100ms and
                                                                                      // converted to cm from ticks
  }

  private void resetEncoderTicks() {
    sensorMotorController.setSelectedSensorPosition(0);
  }

  public void listenToBottomSwitch() {
    if (isBottomSwitchPressed()) {
      resetEncoderTicks();
    }
  }

  public boolean isBottomSwitchPressed() {
    return bottomSwitch.get() && !isBottomSwitchDefective;
  }

  private void updateOnTarget() {
    onTarget.update(pidController.onTarget()); // && Math.abs(getCurrentVelocity()) < VELOCITY_TOLERANCE
  }

  public boolean isOnTarget() {
    return (getCurrentLiftState() == LiftState.MANUAL) || onTarget.get();
  }

  public void setManualMode(boolean enable) {
    if (enable) {
      setLiftState(LiftState.MANUAL);
    } else {
      setLiftState(LiftState.FOLLOWING_CURRENT_STATE);
    }
  }

  private boolean isLastReset = false;
  private double initialResetTimestamp = 0;

  private void updateReset() {
    if (DriverStation.getInstance().isDisabled() || isBottomSwitchDefective) {
      return;
    }
    if (getCurrentLiftState() == LiftState.FOLLOWING_CURRENT_STATE && pidController.getSetpoint() == 0 && isOnTarget()
        && !isBottomSwitchPressed()) {
      setLiftState(LiftState.RESET);
    }
    if (getCurrentLiftState() != LiftState.RESET) {
      return;
    }
    if (!isLastReset) {
      initialResetTimestamp = Timer.getFPGATimestamp();
    }
    if (isBottomSwitchPressed()) {
      setLiftState(DEFAULT_LIFT_STATE);
    } else if (Timer.getFPGATimestamp() - initialResetTimestamp > MAX_RESET_TIME) {
      isBottomSwitchDefective = true;
      setLiftState(DEFAULT_LIFT_STATE);
    } else {
      liftMotor.set(RESET_MODE_POWER_DOWN);
    }

    isLastReset = getCurrentLiftState() == LiftState.RESET;
  }

  private void manualControl() {
    if (getCurrentLiftState() != LiftState.MANUAL || !Robot.isOperationActive) {
      return;
    }

    controllerControl();
  }

  private void semiAutomaticControl() {
    if (getCurrentLiftState() != LiftState.SEMI_AUTOMATIC || !Robot.isOperationActive) {
      return;
    }

    controllerControl();
  }

  private void controllerControl() {
    double leftTrigger = OI.operatorController.getTriggerAxis(Hand.kLeft),
        rightTrigger = OI.operatorController.getTriggerAxis(Hand.kRight);
    if (rightTrigger > 0.2 && leftTrigger < 0.2) {
      manualLiftUp();
    } else if (leftTrigger > 0.2 && rightTrigger < 0.2) {
      manualLiftDown();
    } else {
      applyBaseBower();
    }
  }

  private CommandTimedControllerRumble manualEnterRumbleCommand;
  private CommandTimedControllerRumble manualFinishRumbleCommand;

  public void toggleManual() {
    if (manualEnterRumbleCommand == null) {
      manualEnterRumbleCommand = new CommandTimedControllerRumble(OI.operatorController, RumbleType.kRightRumble,
          RumbleLength.LONG, 1.0);
    }
    if (manualFinishRumbleCommand == null) {
      manualFinishRumbleCommand = new CommandTimedControllerRumble(OI.operatorController, RumbleType.kRightRumble,
          RumbleLength.SHORT, 1.0);
    }

    if (getCurrentLiftState() == LiftState.MANUAL) {
      setLiftState(LiftState.SEMI_AUTOMATIC);
      manualFinishRumbleCommand.start();
    } else {
      setLiftState(LiftState.MANUAL);
      manualEnterRumbleCommand.start();
    }
  }

  @Override
  public void periodic() {
    BumbleTimer.start("Lift");
    if (getCurrentLiftState() == LiftState.MANUAL) {
      manualControl();
    } else {
      if (getCurrentLiftState() == LiftState.SEMI_AUTOMATIC) {
        semiAutomaticControl();
      }
      listenToBottomSwitch();
      updateOnTarget();
      // updateReset(); // TODO: Check this before activating it
    }
    BumbleTimer.time("Lift");
  }

  @Override
  public void sendDebuggingData() {
    SmartDashboard.putNumber("Debugging/Elevator/Height", getCurrentHeight());
    SmartDashboard.putBoolean("Debugging/Elevator/Bottom Switch", bottomSwitch.get());

    SmartDashboard.putBoolean("Debugging/Elevator/isOnTarget", isOnTarget());
    SmartDashboard.putBoolean("Debugging/Elevator/PID Enabled", pidController.isEnabled());

    SmartDashboard.putNumber("Debugging/Elevator/PID Output", pidController.get());

    SmartDashboard.putString("Debugging/Elevator/State", getCurrentLiftState().toString());
  }
}
