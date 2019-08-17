package frc.bumblelib.vision;

import static frc.robot.profiles.InitProfiles.ROBOT_PROFILE;

import java.util.ArrayList;

import edu.wpi.first.wpilibj.DigitalOutput;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.bumblelib.bumblelib_autonomous.pathing.BumbleWaypoint;
import frc.bumblelib.util.BumbleTimer;
import frc.bumblelib.util.Logger;
import frc.bumblelib.util.SimpleCSVLogger;
import frc.bumblelib.util.StableBoolean;
import frc.bumblelib.util.hardware.REVSmartRobotServo;
import frc.bumblelib.util.hardware.pixy.BumblePixy1USB;
import frc.robot.Robot;
import frc.robot.RobotGameState.Direction;
import frc.robot.RobotGameStateManager;
import frc.robot.RobotMap;
import frc.robot.profiles.InitProfiles;
import frc.robot.vision.RobotLocator;

@SuppressWarnings("unused")
public class PixyVision extends Subsystem implements Logger {

	public Subsystem LEDSubsystem = new Subsystem() {
		@Override
		protected void initDefaultCommand() {
		}
	};

	private final double SENSOR_ANGLE = ROBOT_PROFILE.pixyParams.sensorAngle; // angle towards inside of robot when
																				// looking forward

	public BumblePixy1USB pixy = new BumblePixy1USB(ROBOT_PROFILE.pixyParams.pixyUID);
	// public BumblePixy pixy = new BumblePixy2Arduino();

	private PixyTargetProvider frontTargetProvider = new PixyTargetProvider(pixy,
			InitProfiles.ROBOT_PROFILE.robotReferencePoints.frontPixy, SENSOR_ANGLE);

	private PixyTargetProvider rearTargetProvider = new PixyTargetProvider(pixy,
			InitProfiles.ROBOT_PROFILE.robotReferencePoints.rearPixy, -SENSOR_ANGLE);

	private PixyTargetProvider currentTargetProvider = frontTargetProvider;

	SimpleCSVLogger logger;

	// Offset is measured from the robot's x center. positive is right (when looking
	// with the current direction of the element arm)
	private final double elementArmXOffsetForward = ROBOT_PROFILE.pixyParams.elementArmXOffsetForward;
	private final double elementArmXOffsetBackward = ROBOT_PROFILE.pixyParams.elementArmXOffsetBackward;

	private final double REFLECTIVE_WIDTH = 0.405, REFLECTIVE_HEIGHT = 0.145; // in cm
	private final double MEASURED_WIDTH_TO_HEIGHT_RATIO = 2.90;
	private PhysicalTarget reflectiveTarget = new PhysicalTarget(REFLECTIVE_WIDTH, REFLECTIVE_HEIGHT,
			MEASURED_WIDTH_TO_HEIGHT_RATIO);

	public VisualTargetProcessor reflectiveTargetProcessor = new VisualTargetProcessor(frontTargetProvider,
			reflectiveTarget);

	private RobotLocator robotLocator = new RobotLocator(reflectiveTargetProcessor, Robot.m_drivetrain.getGyro());

	private double angleFromTarget, rotationOffsetAngle, xDisplacement, distance, alignmentTargetAbsoluteAngle,
			yDisplacement;
	private BumbleWaypoint waypoint;

	private boolean enabled = false;

	private final int TARGET_TTL = 5;
	private StableBoolean areAnglesValid = new StableBoolean(1, TARGET_TTL, false);

	// reflective pixy servo
	private REVSmartRobotServo pixyServo = new REVSmartRobotServo(RobotMap.VisionPorts.PIXY_SERVO,
			ROBOT_PROFILE.pixyParams.edgeServoAngle, ROBOT_PROFILE.pixyParams.minServoPWM,
			ROBOT_PROFILE.pixyParams.maxServoPWM);
	private PixyDirection currentPixyServoState;

	// LED relay
	public DigitalOutput LED = new DigitalOutput(RobotMap.VisionPorts.LED);

	public enum PixyDirection {
		FORWARD(ROBOT_PROFILE.pixyParams.forwardAngle), BACKWARD(ROBOT_PROFILE.pixyParams.backwardAngle);

		private final double PixyAngle;

		PixyDirection(double PixyAngle) {
			this.PixyAngle = PixyAngle;
		}

		public double getPixyAngle() {
			return PixyAngle;
		}
	}

	public PixyVision() {
		// SmartDashboard.putNumber("Servo angle", 0.0);
	}

	public void setEnabled(boolean enabled) {
		if (enabled && !this.enabled) {
			initLogger();
		} else if (!enabled && this.enabled) {
			logger.close();
		}
		this.enabled = enabled;

		areAnglesValid.forceValue(false);
	}

	public void calculateControlParameters() {
		BumbleTimer.start("calculateControlParameters");

		if (enabled) {
			BumbleTimer.start("hasTargetInView");
			boolean hasTargetInView = currentTargetProvider.hasTargetInView();
			BumbleTimer.time("hasTargetInView");

			BumbleTimer.start("updateImage");
			currentTargetProvider.updateImage();
			BumbleTimer.time("updateImage");
			BumbleTimer.start("update");
			robotLocator.update(hasTargetInView);
			BumbleTimer.time("update");
			alignmentTargetAbsoluteAngle = robotLocator.getAlignmentTargetAbsoluteAngle();
			if (hasTargetInView) {
				waypoint = robotLocator.getRobotWaypoint();
				if (waypoint != null) {
					waypoint.addX(-1 * (RobotGameStateManager.nextGameState.direction == Direction.FORWARD
							? elementArmXOffsetForward
							: elementArmXOffsetBackward));
					angleFromTarget = (-1) * Math.toDegrees(Math.atan2(waypoint.getX(), waypoint.getY()));
					rotationOffsetAngle = (-1) * RobotLocator.bound90Degrees(waypoint.getAngle() - angleFromTarget);
					xDisplacement = (-1) * waypoint.getX();
					yDisplacement = waypoint.getY();
					distance = Math.sqrt(Math.pow(waypoint.getX(), 2.0) + Math.pow(waypoint.getY(), 2.0));
					areAnglesValid.update(true);
				} else {
					areAnglesValid.update(false);
				}
			}
		}

		if (!areAnglesValid()) {
			angleFromTarget = 0.0;
			rotationOffsetAngle = 0.0;
			xDisplacement = 0.0;
			yDisplacement = 0.0;
			distance = 0.0;
		}

		BumbleTimer.time("calculateControlParameters");
	}

	public boolean areAnglesValid() {
		return areAnglesValid.get();
	}

	public void sendDebuggingData() {
		SmartDashboard.putBoolean("PixyVision/areAnglesValid", areAnglesValid());
		SmartDashboard.putNumber("PixyVision/Robot angleFromTarget", angleFromTarget);
		SmartDashboard.putNumber("PixyVision/Robot rotationOffsetAngle", rotationOffsetAngle);
		SmartDashboard.putNumber("PixyVision/Robot Distance", distance);
		if (waypoint != null) {
			SmartDashboard.putNumber("PixyVision/Robot x", waypoint.getX());
			SmartDashboard.putNumber("PixyVision/Robot y", waypoint.getY());
		}

		currentTargetProvider.sendDebuggingData();
		robotLocator.sendDebuggingData();
	}

	@Override
	protected void initDefaultCommand() {
		// nothing
	}

	public void updateServoState() {
		currentPixyServoState = RobotGameStateManager.nextGameState.direction == Direction.FORWARD
				? PixyDirection.FORWARD
				: PixyDirection.BACKWARD;
	}

	public double getServoAngle() {
		if (currentPixyServoState == PixyDirection.FORWARD) {
			return PixyDirection.FORWARD.getPixyAngle() + InitProfiles.ROBOT_PROFILE.pixyParams.ServoAngleInZeroDegrees;
		} else {
			return PixyDirection.BACKWARD.getPixyAngle()
					+ InitProfiles.ROBOT_PROFILE.pixyParams.ServoAngleInZeroDegrees;
		}
	}

	@Override
	public void periodic() {
		calculateControlParameters();
		updateServoState();
		pixyServo.setAngle(getServoAngle());
		currentTargetProvider = RobotGameStateManager.nextGameState.direction == Direction.FORWARD ? frontTargetProvider
				: rearTargetProvider;
		reflectiveTargetProcessor.setTargetProvider(currentTargetProvider);
		if (enabled) {
			logData();
		}
		// pixyServo.setAngle(SmartDashboard.getNumber("Servo angle", 0.0));

	}

	@Override
	public ArrayList<String> getLogHeaders() {
		ArrayList<String> output = new ArrayList<String>();

		output.add("areAnglesValid");
		output.add("RobotAngleFromTarget");
		output.add("RobotRotationOffsetAngle");
		output.add("RobotDistance");
		output.add("RobotX");
		output.add("RobotY");

		return output;
	}

	@Override
	public ArrayList<String> getLogHeaderUnits() {
		ArrayList<String> output = new ArrayList<String>();

		output.add("Boolean");
		output.add("Degrees");
		output.add("Degrees");
		output.add("CM");
		output.add("CM");
		output.add("CM");

		return output;
	}

	@Override
	public ArrayList<String> getLogData() {
		ArrayList<String> output = new ArrayList<String>();
		output.add(this.areAnglesValid.get() ? "1" : "0");
		output.add(Double.toString(angleFromTarget));
		output.add(Double.toString(rotationOffsetAngle));
		output.add(Double.toString(distance));
		output.add(waypoint != null ? Double.toString(waypoint.getX()) : "");
		output.add(waypoint != null ? Double.toString(waypoint.getY()) : "");

		return output;
	}

	private void initLogger() {
		logger = new SimpleCSVLogger("PixyDebuggingLog");

		ArrayList<String> headers = new ArrayList<String>();
		headers.addAll(this.getLogHeaders());
		headers.addAll(robotLocator.getLogHeaders());
		headers.addAll(currentTargetProvider.getLogHeaders());

		ArrayList<String> headerUnits = new ArrayList<String>();
		headerUnits.addAll(this.getLogHeaderUnits());
		headerUnits.addAll(robotLocator.getLogHeaderUnits());
		headerUnits.addAll(currentTargetProvider.getLogHeaderUnits());

		logger.init(headers, headerUnits);
	}

	private void logData() {
		logger.writeData(this.getLogData(), false);
		logger.writeData(robotLocator.getLogData(), false);
		logger.writeData(currentTargetProvider.getLogData(), true);
	}
	
	public boolean isEnabled() {
		return this.enabled;
	}

	/**
	 * @return the waypoint
	 */
	public BumbleWaypoint getWaypoint() {
		return waypoint;
	}

	/**
	 * @return the offsetAngleFromTarget
	 */
	public double getOffsetAngleFromTarget() {
		return rotationOffsetAngle;
	}

	/**
	 * @return the angleToTarget
	 */
	public double getAngleToTarget() {
		return angleFromTarget;
	}

	public double getXDisplacement() {
		return xDisplacement;
	}

	/**
	 * @return the yDisplacement
	 */
	public double getYDisplacement() {
		return yDisplacement;
	}

	/**
	 * @return the alignmentTargetAbsoluteAngle
	 */
	public double getAlignmentTargetAbsoluteAngle() {
		return alignmentTargetAbsoluteAngle;
	}

	/**
	 * @return the distance
	 */
	public double getDistance() {
		return distance;
	}
}
