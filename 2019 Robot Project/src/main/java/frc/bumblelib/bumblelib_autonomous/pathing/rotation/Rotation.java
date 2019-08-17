package frc.bumblelib.bumblelib_autonomous.pathing.rotation;

import static frc.robot.Robot.m_drivetrain;
import static frc.robot.profiles.InitProfiles.ROBOT_PROFILE;

import frc.bumblelib.bumblelib_autonomous.CSVManager;
import frc.bumblelib.bumblelib_autonomous.CSVSendable;
import frc.bumblelib.bumblelib_autonomous.pathing.enums.Alliance;
import frc.bumblelib.bumblelib_autonomous.pathing.enums.Side;
import frc.robot.Robot;
import frc.robot.profiles.InitProfiles;
import frc.robot.profiles.RobotProfile.PathfinderParams.GearParams;
import frc.robot.subsystems.Drivetrain.Gear;
import jaci.pathfinder.Pathfinder;
import jaci.pathfinder.Trajectory;
import jaci.pathfinder.Trajectory.Config;
import jaci.pathfinder.Trajectory.FitMethod;
import jaci.pathfinder.Waypoint;
import jaci.pathfinder.followers.EncoderFollower;
import jaci.pathfinder.modifiers.TankModifier;

public class Rotation implements CSVSendable {

    private Gear gear;
    private GearParams gearParams;
    private Angle angle;
    private boolean isRelative;

    private Alliance alliance;
    private Side side;

    private Config cfg;

    private double gyroP;

    private EncoderFollower left, right;
    private int rightOffset, leftOffset, rightPos, leftPos;
    private int motorOverflowCount;

    public Rotation(Gear gear, Angle angle, boolean isRelative) {
        this.gear = gear;
        this.gearParams = InitProfiles.ROBOT_PROFILE.pathfinderParams.getGearParams(gear);
        this.angle = angle;
        this.isRelative = isRelative;
        this.gyroP = gearParams.rotGyroP;
        
        cfg = new Trajectory.Config(FitMethod.HERMITE_QUINTIC, Trajectory.Config.SAMPLES_HIGH, InitProfiles.ROBOT_PROFILE.pathfinderParams.deltaTime, gearParams.rotMaxVel, gearParams.rotMaxAccel, InitProfiles.ROBOT_PROFILE.pathfinderParams.maxJerk);
    }

    /**
     * @param gyroP the gyroP to set
     */
    public void setGyroP(double gyroP) {
        this.gyroP = gyroP;
    }

    public void configAllianceSide(Alliance alliance, Side side) {
        this.alliance = alliance;
        this.side = side;
    }

    public void init() {
        // Switch to correct gear
        Robot.m_drivetrain.setGear(gear);

        // Get Trajectory from path based on alliance and side and generate a
        // TankModifier based on said Trajectory.
        Trajectory trajectory = CSVManager.getTrajectory(this, alliance, side);
        TankModifier modifier = new TankModifier(trajectory)
                .modify(ROBOT_PROFILE.autonomousParams.wheel_base_effective_width);

        // Initialize left and right EncoderFollowers and configure PIDVA gains.
        right = new EncoderFollower(modifier.getRightTrajectory());
        left = new EncoderFollower(modifier.getLeftTrajectory());
        right.configurePIDVA(0.0, 0.0, 0.0, gearParams.rotKv, gearParams.rotKa);
        left.configurePIDVA(0.0, 0.0, 0.0, gearParams.rotKv, gearParams.rotKa);

        // Check if path is reverse and configure initial encoder count accordingly.
        double sign = Math.signum(adjustedValue(alliance, side));
        rightOffset = (int) (sign * m_drivetrain.getRightEncoderTicks());
        leftOffset = (int) (-sign * m_drivetrain.getLeftEncoderTicks());

        // Configure EncoderFollowers.
        right.configureEncoder(rightOffset, ROBOT_PROFILE.autonomousParams.ticks_per_rev,
                ROBOT_PROFILE.autonomousParams.wheel_diameter);
        left.configureEncoder(leftOffset, ROBOT_PROFILE.autonomousParams.ticks_per_rev,
                ROBOT_PROFILE.autonomousParams.wheel_diameter);

        // Reset EncoderFollowers before running a path.
        right.reset();
        left.reset();

        try {
            m_drivetrain.getGyro().startRelativeMode();
        } catch (Exception e) {
            e.printStackTrace();
		}
    }

    public void follow() {
        // Get encoder position and invert sign if negative.
        rightPos = (int) Math.signum(adjustedValue(alliance, side)) * m_drivetrain.getRightEncoderTicks();
        leftPos = (int) Math.signum(adjustedValue(alliance, side)) * -1 * m_drivetrain.getLeftEncoderTicks();

        double angleSetpoint = Math.signum(adjustedValue(alliance, side)) * getAngleSetpoint();

        double rightPower, leftPower;
        double sign = Math.signum(adjustedValue(alliance, side));

        rightPower = sign * (gearParams.rotVIntercept + right.calculate(rightPos));
        leftPower = -sign * (gearParams.rotVIntercept + left.calculate(leftPos));

        // Calculate gyro correction by multiplying the difference between the current
        // angle and the desired angle by a constant.
        double rawHeading = 0.0;
        try {
            rawHeading = m_drivetrain.getGyro().getRelativeYaw();
        } catch (Exception e) {
            e.printStackTrace();
		}
        double gyroError = Pathfinder.boundHalfDegrees(angleSetpoint - rawHeading);
        double turn = gyroP * gyroError;

        // Count power overflow to motors.
        if (Math.abs(leftPower) > 1 || Math.abs(rightPower) > 1) {
            motorOverflowCount++;
        }

        // Apply power to motors.
        m_drivetrain.bumbleDrive.setLeftRightMotorOutputs(leftPower - turn, rightPower + turn, false);
    }

    public void end() {
        m_drivetrain.getGyro().endRelativeMode();
    }

    public boolean isFinished() {
        return right.isFinished() && left.isFinished();
    }

    @Override
    public String getName() {
        return adjustedValue(Alliance.RED, Side.RIGHT) + " Rotation, gear: " + gear;
    }

    @Override
    public String getNameTemplate(Alliance alliance, Side side) {
        return "rotations/" + adjustedValue(alliance, side) + " Rotation, gear: " + gear + ".csv";
    }

    @Override
    public String getDescription(Alliance alliance, Side side) {
        return "{angle: " + Math.abs(adjustedValue(alliance, side)) + ", maxVel: " + gearParams.rotMaxVel + ", maxAccel: " + gearParams.rotMaxAccel +"}";
    }

    @Override
	public Trajectory generateTrajectory(Alliance alliance, Side side) {
        double theta = Math.toRadians(adjustedValue(alliance, side));
        double distance = 0.5 * InitProfiles.ROBOT_PROFILE.autonomousParams.wheel_base_effective_width * theta;
        Waypoint[] waypoints = new Waypoint[] {
            new Waypoint(0.0, 0.0, 0.0),
            new Waypoint(distance, 0.0, 0.0)
        };
		return Pathfinder.generate(waypoints, cfg);
    }

    private double adjustedValue(Alliance alliance, Side side) {
        return Pathfinder.boundHalfDegrees(isRelative ? angle.getValue(alliance, side) : angle.getValue(alliance, side) - Math.round(m_drivetrain.getGyro().getYaw()));
    }

    public double getAngleSetpoint() {
        if (left.isFinished()) {
            return 0.0;
        }
        return Pathfinder.boundHalfDegrees(Math.toDegrees(2 * right.getSegment().position / ROBOT_PROFILE.autonomousParams.wheel_base_effective_width));
    }

    public double getCurrentAngle() {
        return Pathfinder.boundHalfDegrees(Math.toDegrees(2 * rightPos / ROBOT_PROFILE.autonomousParams.wheel_base_effective_width));
    }

    public int getMotorOverflowCount() {
        return motorOverflowCount;
    }

    public double getRotationAngle() {
        return adjustedValue(alliance, side);
    }
}