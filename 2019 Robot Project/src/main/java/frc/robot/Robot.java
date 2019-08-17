/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import static frc.robot.profiles.InitProfiles.ROBOT_PROFILE;

import edu.wpi.cscore.CvSource;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.InstantCommand;
import edu.wpi.first.wpilibj.command.Scheduler;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.bumblelib.bumblelib_autonomous.AutoChooser;
import frc.bumblelib.bumblelib_autonomous.calibration_tools.EffectiveWheelBaseWidthTuner;
import frc.bumblelib.bumblelib_autonomous.calibration_tools.KaTuner;
import frc.bumblelib.bumblelib_autonomous.calibration_tools.KvTuner;
import frc.bumblelib.bumblelib_autonomous.calibration_tools.MaxAccelerationAndMaxVelocityTuner;
import frc.bumblelib.bumblelib_autonomous.calibration_tools.PDGTuner;
import frc.bumblelib.bumblelib_autonomous.calibration_tools.RotationCalibration;
import frc.bumblelib.bumblelib_autonomous.pathing.PathRegistry;
import frc.bumblelib.util.BumbleTimer;
import frc.bumblelib.util.SimpleCSVLogger;
import frc.bumblelib.util.SmartdashboardDebugging;
import frc.bumblelib.util.hardware.REVSmartRobotServo;
import frc.bumblelib.vision.PixyVision;
import frc.robot.RobotGameState.RobotAction;
import frc.robot.autonomous.FieldPoints;
import frc.robot.autonomous.InitialPathPoints;
import frc.robot.autonomous.paths.Paths;
import frc.robot.autonomous.sequences.Sequences;
import frc.robot.commands.calibration.CalibrateElementArm;
import frc.robot.commands.calibration.CalibrateIntakeArm;
import frc.robot.commands.calibration.CalibrateLift;
import frc.robot.commands.calibration.CalibrateLiftArm;
import frc.robot.commands.calibration.CalibrateTurnByDegrees;
import frc.robot.commands.calibration.CalibrateClimb;
import frc.robot.commands.climb.AllClimb;
import frc.robot.subsystems.Camera;
import frc.robot.subsystems.CargoHandler;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Drivetrain.DrivetrainControlState;
import frc.robot.subsystems.Drivetrain.Gear;
import frc.robot.subsystems.ElementArm;
import frc.robot.subsystems.FrontClimb;
import frc.robot.subsystems.HatchHandler;
import frc.robot.subsystems.IntakeArm;
import frc.robot.subsystems.IntakeRollers;
import frc.robot.subsystems.Lift;
import frc.robot.subsystems.LiftArm;
import frc.robot.subsystems.RearClimb;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the TimedRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot implements SmartdashboardDebugging {

  public static boolean isAuto = false;
  public static boolean isOperationActive = true; // If this is set to false, all BumbleTriggers return false in their
                                                  // conditions. Don't touch it!

  // Don't change the order of subsystem instantiating unless it is tested on the
  // robot.
  // Subsystems:
  public static FrontClimb m_frontClimb = new FrontClimb();
  public static RearClimb m_rearClimb = new RearClimb();
  public static IntakeArm m_intakeArm = new IntakeArm();
  public static LiftArm m_liftArm = new LiftArm(); // Must be instantiated before ElementArm
  public static ElementArm m_elementArm = new ElementArm();
  public static Lift m_lift = new Lift();
  public static IntakeRollers m_intakeRollers = new IntakeRollers();
  public static HatchHandler m_hatchHandler = new HatchHandler();
  public static CargoHandler m_cargoHandler = new CargoHandler();
  public static Drivetrain m_drivetrain = new Drivetrain();
  // ---

  // Vision
  public static PixyVision m_pixyVision = new PixyVision();
  // ---

  public CvSource outputStream;

  public static Camera m_camera = new Camera();

  public static OI m_oi;

  // Autonomous relatedFieldPoints
  public static FieldPoints m_fieldPoint = new FieldPoints();
  public static InitialPathPoints initialPathPoints = new InitialPathPoints();
  public static Paths paths = new Paths();
  public static Sequences sequences = new Sequences();
  public static AutoChooser autoChooser = new AutoChooser();

  // Calibrations
  public static CalibrationMode calibrationMode = CalibrationMode.OFF; // Sets the current calbration mode
  private static Command calibrationCommand;
  REVSmartRobotServo servo = new REVSmartRobotServo(7);

  /**
   * This function is run when the robot is first started up and should be used
   * for any initialization code.
   */
  @Override
  public void robotInit() {
    // This must be here because when it's run for the first time after the robot
    // boots it takes a second to proccess
    SimpleCSVLogger tempLogger = new SimpleCSVLogger("logName");
    tempLogger.init(new String[0], new String[0]);
    tempLogger.close();
    // ===========

    // NetworkTable.setUpdateRate(0.01);

    if (calibrationMode == CalibrationMode.OFF) {
      RobotGameStateManager.setSystemsToFollowCurrentState();
      m_intakeArm.resetToFolded();
      m_oi = new OI();
    } else {
      switch (calibrationMode) {
      case LIFT:
        calibrationCommand = new CalibrateLift();
        break;
      case LIFT_ARM:
        calibrationCommand = new CalibrateLiftArm();
        break;
      case INTAKE_ARM:
        calibrationCommand = new CalibrateIntakeArm();
        break;
      case ELEMENT_ARM:
        calibrationCommand = new CalibrateElementArm();
        break;
      case CLIMB:
        calibrationCommand = new CalibrateClimb();
        break;
      case KV_PATH:
        calibrationCommand = new KvTuner(false);
        break;
      case KA_PATH:
        calibrationCommand = new KaTuner(false);
        break;
      case KV_ROTATION:
        calibrationCommand = new KvTuner(true);
        break;
      case KA_ROTATION:
        calibrationCommand = new KaTuner(true);
        break;
      case EFFECTIVE_WHEELBASE_WIDTH:
        calibrationCommand = new EffectiveWheelBaseWidthTuner();
        break;
      case PDG:
        calibrationCommand = new PDGTuner(Paths.shortCalibrationPath, Paths.longCalibrationPath, Paths.curvePath);
        break;
      case ROTATE_GYRO_P:
        calibrationCommand = new RotationCalibration();
        break;
      case PATH_MAX_GAINS:
        calibrationCommand = new MaxAccelerationAndMaxVelocityTuner(false);
        break;
      case ROTATE_MAX_GAINS:
        calibrationCommand = new MaxAccelerationAndMaxVelocityTuner(true);
        break;
      case TURN_BY_DEGREES:
        calibrationCommand = new CalibrateTurnByDegrees();
        break;
      default:
        break;
      }
    }

    SmartDashboard.putNumber("SERVO", 0.0);

    m_camera.initCamera();
    m_hatchHandler.setDefaultSolenoidStates();
    SmartDashboard.putBoolean("Debugging", false);
    SmartDashboard.putBoolean("BumbleTimer", false);

    SmartDashboard.putData("Initialize climbers", new AllClimb(ROBOT_PROFILE.climbParams.foldedHeight, true, false));

    // Autonomous
    m_drivetrain.getGyro().reset();
    m_drivetrain.resetEncoders();
    PathRegistry.getInstance().generatePaths(); // generate autonomous paths
    autoChooser.init();
    m_pixyVision.LED.set(true); // Turns on LED after robot boot is complete.

    // Place dashboard buttons
    SmartDashboard.putBoolean("Reset Gyro", false);
    SmartDashboard.putBoolean("Turn Off LED", false);
    SmartDashboard.putBoolean("Enable Vision", false);
    SmartDashboard.putBoolean("Disable Vision", false);
    SmartDashboard.putBoolean("Stop Pixy Block Program", false);
    SmartDashboard.putBoolean("Start Pixy Block Program", false);
    SmartDashboard.putBoolean("Restart Pixy", false);
  }

  private boolean lastAbortCargoFloorCollect = false;

  /**
   * This function is called every robot packet, no matter the mode. Use this for
   * items like diagnostics that you want ran during disabled, autonomous,
   * teleoperated and test.
   *
   * <p>
   * This runs after the mode specific periodic functions, but before LiveWindow
   * and SmartDashboard integrated updating.
   */

  @Override
  public void robotPeriodic() {
    // Dashboard buttons
    if (SmartDashboard.getBoolean("Reset Gyro", false)) {
      SmartDashboard.putBoolean("Reset Gyro", false);
      Robot.m_drivetrain.getGyro().reset();
    }
    if (SmartDashboard.getBoolean("Turn Off LED", false)) {
      SmartDashboard.putBoolean("Turn Off LED", false);
      Robot.m_pixyVision.LED.set(false);
    }
    if (SmartDashboard.getBoolean("Enable Vision", false)) {
      SmartDashboard.putBoolean("Enable Vision", false);
      m_pixyVision.setEnabled(true);
      m_pixyVision.LED.set(true);
    }
    if (SmartDashboard.getBoolean("Disable Vision", false)) {
      SmartDashboard.putBoolean("Disable Vision", false);
      m_pixyVision.setEnabled(false);
      m_pixyVision.LED.set(false);
    }
    if (SmartDashboard.getBoolean("Stop Pixy Block Program", false)) {
      SmartDashboard.putBoolean("Stop Pixy Block Program", false);
      m_pixyVision.pixy.pixy.stopBlockProgram();
    }
    if (SmartDashboard.getBoolean("Start Pixy Block Program", false)) {
      SmartDashboard.putBoolean("Start Pixy Block Program", false);
      m_pixyVision.pixy.pixy.startBlockProgram();
    }
    if (SmartDashboard.getBoolean("Restart Pixy", false)) {
      SmartDashboard.putBoolean("Restart pixy", false);
      m_pixyVision.pixy.restartPixy();
    }
    // ==========

    servo.setAngle(SmartDashboard.getNumber("SERVO", 0.0)); // Used to quickly connect a servo and reset it
    BumbleTimer.setEnabled(SmartDashboard.getBoolean("BumbleTimer", false));
    BumbleTimer.time("Robot Loop");
    BumbleTimer.start("robotPeriodic");

    BumbleTimer.sendAsArray();

    ControlledGamePieceDetector.update();

    updateRecordingData();
    updateDashboardDebuggingData();
    if (calibrationMode == CalibrationMode.OFF) {
      RobotGameStateManager.periodic();
    }
    // double climbHeight = SmartDashboard.getNumber("Climb height", -4);
    // SmartDashboard.putNumber("Climb height", climbHeight);
    // SmartDashboard.putData("Set all climb", new AllClimb(climbHeight, true));
    DashboardManager.updateSmartDashboard();

    lastAbortCargoFloorCollect = RobotGameStateManager.currentGameState.robotAction == RobotAction.ABORT_CARGO_FLOOR_COLLECT;
    BumbleTimer.time("robotPeriodic");
  }

  private int prevState = 0;

  private void updateRecordingData() {
    DriverStation ds = DriverStation.getInstance();

    SmartDashboard.putNumber("Recording/Timestamp", ds.getMatchTime());
    SmartDashboard.putBoolean("Recording/VisionEnabled", m_pixyVision.isEnabled());

    if (ds.isAutonomous() && ds.isEnabled()) {
      prevState = 1;
    } else if (ds.isOperatorControl() && ds.isEnabled()) {
      prevState = 2;
    }

    SmartDashboard.putBoolean("Recording/IsMatch", ds.isEnabled() || prevState == 1);// && ds.isFMSAttached());
  }

  private void updateDashboardDebuggingData() {
    BumbleTimer.start("sendSensors");

    if (SmartDashboard.getBoolean("Debugging", false)) {
      RobotGameStateManager.sendDebuggingData();

      ControlledGamePieceDetector.sendDebuggingData();

      this.sendDebuggingData();

      m_liftArm.sendDebuggingData();
      m_elementArm.sendDebuggingData();
      m_lift.sendDebuggingData();
      m_intakeArm.sendDebuggingData();

      m_intakeRollers.sendDebuggingData();
      m_hatchHandler.sendDebuggingData();
      m_cargoHandler.sendDebuggingData();

      m_drivetrain.sendDebuggingData();

      m_frontClimb.sendDebuggingData();
      m_rearClimb.sendDebuggingData();

      // m_camera.sendDebuggingData();

      m_pixyVision.sendDebuggingData();
    }

    BumbleTimer.time("sendSensors");
  }

  /**
   * This function is called once each time the robot enters Disabled mode. You
   * can use it to reset any subsystem information you want to clear when the
   * robot is disabled.
   */
  @Override
  public void disabledInit() {
    isAuto = false;
  }

  @Override
  public void disabledPeriodic() {
    Scheduler.getInstance().run();
  }

  /**
   * This autonomous (along with the chooser code above) shows how to select
   * between different autonomous modes using the dashboard. The sendable chooser
   * code works with the Java SmartDashboard. If you prefer the LabVIEW Dashboard,
   * remove all of the chooser code and uncomment the getString code to get the
   * auto name from the text box below the Gyro
   *
   * <p>
   * You can add additional auto modes by adding additional commands to the
   * chooser code above (like the commented example) or additional comparisons to
   * the switch structure below with additional strings & commands.
   */
  @Override
  public void autonomousInit() {
    m_pixyVision.LED.set(false);

    if (calibrationMode != CalibrationMode.OFF) {
      // CALIBRATION MODE
      m_drivetrain.setGear(Gear.SPEED_GEAR);
      calibrationCommand.start();
    } else {
      // regular autonomous init code
      if (DriverStation.getInstance().isFMSAttached()) {
        SmartDashboard.putBoolean("Debugging", false);
        SmartDashboard.putBoolean("BumbleTimer", false);
      }

      m_hatchHandler.extendHoldSolenoid();
      isAuto = true;

      m_drivetrain.initDrivetrainControlState(DrivetrainControlState.PATH_FOLLOWING);

      autoChooser.runSelected();
      // new DrivePath(Paths.shortCalibrationPath).start();

      // TESTING CODE:
      // AutonomousSettings.setSide(Side.LEFT);
      // AutonomousSettings.setAlliance(Alliance.RED);

      // new DrivePath(Paths.farPlatformToFarShipPrePlacement).start();

      // new TestSequence().start();

      // (new RotationCalibration()).start();
      // new Rotate(new Rotation(Gear.POWER_GEAR, new SingularAngle(90),
      // true)).start();
    }
  }

  /**
   * This function is called periodically during autonomous.
   */
  @Override
  public void autonomousPeriodic() {
    Scheduler.getInstance().run();
  }

  @Override
  public void teleopInit() {
    isAuto = false;
    m_pixyVision.LED.set(false);

    if (DriverStation.getInstance().isFMSAttached()) {
      SmartDashboard.putBoolean("Debugging", false);
      SmartDashboard.putBoolean("BumbleTimer", false);
    }

    m_hatchHandler.foldPushSolenoid();
    m_drivetrain.initDrivetrainControlState(DrivetrainControlState.OPEN_LOOP);
  }

  /**
   * This function is called periodically during operator control.
   */
  @Override
  public void teleopPeriodic() {
    BumbleTimer.start("teleopPeriodic");
    Scheduler.getInstance().run();
    BumbleTimer.time("teleopPeriodic");
  }

  @Override
  public void testInit() {

  }

  /**
   * This function is called periodically during test mode.
   */
  @Override
  public void testPeriodic() {
    Scheduler.getInstance().run();
  }

  public enum CalibrationMode {
    OFF, LIFT, LIFT_ARM, INTAKE_ARM, ELEMENT_ARM, CLIMB, KV_PATH, KA_PATH, KV_ROTATION, KA_ROTATION,
    EFFECTIVE_WHEELBASE_WIDTH, PDG, ROTATE_GYRO_P, PATH_MAX_GAINS, ROTATE_MAX_GAINS, TURN_BY_DEGREES;
  }

  @Override
  public void sendDebuggingData() {
    SmartDashboard.putString("Debugging/Commands/LiftArm", m_liftArm.getCurrentCommandName());
    SmartDashboard.putString("Debugging/Commands/ElementArm", m_elementArm.getCurrentCommandName());
    SmartDashboard.putString("Debugging/Commands/IntakeArm", m_intakeArm.getCurrentCommandName());
    SmartDashboard.putString("Debugging/Commands/Elevator", m_lift.getCurrentCommandName());
    SmartDashboard.putString("Debugging/Commands/FrontClimb", m_frontClimb.getCurrentCommandName());
    SmartDashboard.putString("Debugging/Commands/RearClimb", m_rearClimb.getCurrentCommandName());
    SmartDashboard.putString("Debugging/Commands/RollersOfTheIntake", m_intakeRollers.getCurrentCommandName());
    SmartDashboard.putString("Debugging/Commands/HatchHandler", m_hatchHandler.getCurrentCommandName());
    SmartDashboard.putString("Debugging/Commands/CargoHandler", m_cargoHandler.getCurrentCommandName());
    SmartDashboard.putString("Debugging/Commands/Drivetrain", m_drivetrain.getCurrentCommandName());

    SmartDashboard.putData("Debugging/Scheduler", Scheduler.getInstance());
  }
}