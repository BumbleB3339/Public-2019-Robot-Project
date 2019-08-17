/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import static frc.robot.profiles.InitProfiles.ROBOT_PROFILE;

import edu.wpi.cscore.MjpegServer;
import edu.wpi.cscore.UsbCamera;
import edu.wpi.cscore.VideoMode;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.bumblelib.util.SmartdashboardDebugging;
import frc.bumblelib.util.hardware.REVSmartRobotServo;
import frc.robot.RobotGameState;
import frc.robot.RobotGameState.Direction;
import frc.robot.RobotGameState.FieldObjective;
import frc.robot.RobotGameState.GamePiece;
import frc.robot.RobotGameState.RobotAction;
import frc.robot.RobotGameState.RocketPlacementHeight;
import frc.robot.RobotGameStateManager;
import frc.robot.RobotMap;

/**
 * This the subsystem that controls opening the camera and moving the camera's
 * servo. All manipulation of camera stream is performed on the Driver Station
 * to reduce latency.
 */
public class Camera extends Subsystem implements SmartdashboardDebugging {

  public UsbCamera camera;
  public MjpegServer mJpegServer;

  private final int DEFAULT_CAMERA_EXPOSURE = 1;

  private CameraServoStates cameraServoState = CameraServoStates.DRIVING;

  private final int SCREEN_WIDTH = 320;
  private final int SCREEN_HEIGHT = 240;

  private final int FPS = 15;
  private final int COMPRESSION = 30;

  // Sensors and actuators
  private REVSmartRobotServo cameraServo = new REVSmartRobotServo(RobotMap.VisionPorts.CAMERA_SERVO);
  private double servoOffsetAngle = ROBOT_PROFILE.cameraParams.servoOffsetAngle;

  public enum CameraServoStates { // 0 when the camera is facing upwards
    FLOOR_COLLECT(115), //
    DRIVING(110), //
    CARGO_SHIP(90), //
    LEVEL_ONE_CARGO(75), //
    LEVEL_TWO(40), //
    LEVEL_THREE(30);

    private final double angle;

    CameraServoStates(double angle) {
      this.angle = angle;
    }

    public double getAngle(RobotGameState.Direction direction) {
      return (ROBOT_PROFILE.cameraParams.negateServoAngles ? -1 : 1)
          * (direction == Direction.FORWARD ? angle : -angle);
    }
  }

  public void initCamera() {
    camera = CameraServer.getInstance().startAutomaticCapture();
    camera.setVideoMode(VideoMode.PixelFormat.kMJPEG, SCREEN_WIDTH, SCREEN_HEIGHT, FPS);

    // As for the arguments in this method: 254 used them in their 2018 camera
    // stream, so why not :) (and it works!)
    mJpegServer = new MjpegServer("serve_USB Camera 0", 5810);

    mJpegServer.setSource(camera);
    mJpegServer.setFPS(FPS);
    mJpegServer.setCompression(COMPRESSION);
  }

  private int lastExposure = DEFAULT_CAMERA_EXPOSURE;
  private boolean lastAutoExposure = false;

  private void updateCameraExposure() {
    if (SmartDashboard.getBoolean("Camera Auto Exposure", false) != lastAutoExposure) {
      if (!lastAutoExposure) {
        camera.setExposureAuto();
      } else {
        camera.setExposureManual((int) SmartDashboard.getNumber("Camera Exposure", DEFAULT_CAMERA_EXPOSURE));
      }
    } else if (SmartDashboard.getNumber("Camera Exposure", DEFAULT_CAMERA_EXPOSURE) != lastExposure) {
      camera.setExposureManual((int) SmartDashboard.getNumber("Camera Exposure", DEFAULT_CAMERA_EXPOSURE));
    }

    lastExposure = (int) SmartDashboard.getNumber("Camera Exposure", DEFAULT_CAMERA_EXPOSURE);
    lastAutoExposure = SmartDashboard.getBoolean("Camera Auto Exposure", false);
  }

  @Override
  public void periodic() {
    updateCameraServoState();
    setAngle(cameraServoState.getAngle(RobotGameStateManager.nextGameState.direction));
    updateCameraExposure();
  }

  public CameraServoStates getCameraServoState() {
    return cameraServoState;
  }

  private void setCameraServoState(CameraServoStates cameraServoState) {
    this.cameraServoState = cameraServoState;
  }

  public void setAngle(double angle) {
    cameraServo.setAngle(servoOffsetAngle + angle);
  }

  private RobotAction lastUpdatedRobotAction = null;

  private void updateCameraServoState() {
    RobotAction currentRobotAction = RobotGameStateManager.currentGameState.robotAction;

    if (currentRobotAction == lastUpdatedRobotAction) {
      return;
    }

    switch (currentRobotAction) {
    case FOLDED:
    case FEEDER_COLLECT:
    case CLIMB:
      setCameraServoState(CameraServoStates.DRIVING);
      break;
    case PRE_PLACEMENT:
    case POST_PLACEMENT:
      if (RobotGameStateManager.currentGameState.fieldObjective == FieldObjective.CARGOSHIP) {
        // if cargoship
        setCameraServoState(CameraServoStates.CARGO_SHIP);
      } else if (RobotGameStateManager.currentGameState.rocketPlacementHeight == RocketPlacementHeight.LEVEL1
          && RobotGameStateManager.nextGameState.gamePiece == GamePiece.CARGO) {
        // if level one cargo
        setCameraServoState(CameraServoStates.LEVEL_ONE_CARGO);
      } else if (RobotGameStateManager.currentGameState.rocketPlacementHeight == RocketPlacementHeight.LEVEL1
          && RobotGameStateManager.nextGameState.gamePiece == GamePiece.HATCH_PANEL) {
        // if level one hatch panel
        setCameraServoState(CameraServoStates.DRIVING);
      } else if (RobotGameStateManager.currentGameState.rocketPlacementHeight == RocketPlacementHeight.LEVEL2) {
        // if level 2
        setCameraServoState(CameraServoStates.LEVEL_TWO);
      } else if (RobotGameStateManager.currentGameState.rocketPlacementHeight == RocketPlacementHeight.LEVEL3) {
        // if level 3
        setCameraServoState(CameraServoStates.LEVEL_THREE);
      }
      break;
    case FLOOR_COLLECT:
      setCameraServoState(CameraServoStates.FLOOR_COLLECT);
      break;
    default:
      break;
    }

    lastUpdatedRobotAction = currentRobotAction;
  }

  @Override
  public void initDefaultCommand() {
  }

  @Override
  public void sendDebuggingData() {
    SmartDashboard.putNumber("Debugging/Camera/ServoAngle", cameraServo.getAngle());
  }
}
