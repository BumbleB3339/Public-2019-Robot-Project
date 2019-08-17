package frc.bumblelib.bumblelib_autonomous.pathing;

import static frc.robot.Robot.m_drivetrain;
import static frc.robot.profiles.InitProfiles.ROBOT_PROFILE;

import frc.bumblelib.bumblelib_autonomous.CSVManager;
import frc.bumblelib.bumblelib_autonomous.pathing.enums.Alliance;
import frc.bumblelib.bumblelib_autonomous.pathing.enums.Direction;
import frc.bumblelib.bumblelib_autonomous.pathing.enums.Side;
import frc.bumblelib.bumblelib_autonomous.pathing.path.Path;
import frc.robot.Robot;
import frc.robot.profiles.RobotProfile.PathfinderParams.GearParams;
import jaci.pathfinder.Pathfinder;
import jaci.pathfinder.Trajectory;
import jaci.pathfinder.followers.EncoderFollower;
import jaci.pathfinder.modifiers.TankModifier;

/**
 * The PathFollower object is responsible for handling path generation and
 * following. This class should be instantiated in the DrivePath Command.
 */
public class PathFollower {

    private int motorOverflowCount;
    private int rightOffset, leftOffset, rightPos, leftPos;
    private double gyroError;
    private double kP, kD, gyroP;

    private Path path;
    private Alliance alliance;
    private Side side;

    private double trajectoryLength;
    private EncoderFollower right, left;

    private GearParams gearParams;

    public PathFollower(Path path, Alliance alliance, Side side) {
        this.path = path;
        this.alliance = alliance;
        this.side = side;

        gearParams = ROBOT_PROFILE.pathfinderParams.getGearParams(path.getGear());
        kP = path.getPIDPreset().getKp();
        kD = path.getPIDPreset().getKd();
        gyroP = gearParams.gyroP;
    }

    public void setPDG(double kP, double kD, double gyroP) {
        this.kP = kP;
        this.kD = kD;
        this.gyroP = gyroP;
    }

    /**
     * Initialize the PathFollower. This method should be called once before the
     * path is run.
     * 
     * @param alliance The alliance color with which to run the path
     * @param side     The field side on which the path will run
     */
    public void init() {
        // Switch to correct gear
        Robot.m_drivetrain.setGear(path.getGear());

        // Get Trajectory from path based on alliance and side and generate a
        // TankModifier based on said Trajectory.
        Trajectory trajectory = CSVManager.getTrajectory(path, alliance, side);
        TankModifier modifier = new TankModifier(trajectory)
                .modify(ROBOT_PROFILE.autonomousParams.wheel_base_effective_width);

        // Store trajectory length.
        trajectoryLength = trajectory.get(trajectory.length() - 1).position;

        // Initialize left and right EncoderFollowers and configure PIDVA gains.
        if (path.getDirection() == Direction.REVERSE) {
            right = new EncoderFollower(modifier.getLeftTrajectory());
            left = new EncoderFollower(modifier.getRightTrajectory());
        } else {
            right = new EncoderFollower(modifier.getRightTrajectory());
            left = new EncoderFollower(modifier.getLeftTrajectory());
        }
        right.configurePIDVA(kP, 0.0, kD, gearParams.kV, gearParams.kA);
        left.configurePIDVA(kP, 0.0, kD, gearParams.kV, gearParams.kA);

        // Check if path is reverse and configure initial encoder count accordingly.
        int sign = path.getDirection() == Direction.REVERSE ? -1 : 1;
        rightOffset = sign * m_drivetrain.getRightEncoderTicks();
        leftOffset = sign * m_drivetrain.getLeftEncoderTicks();

        // Configure EncoderFollowers.
        right.configureEncoder(rightOffset, ROBOT_PROFILE.autonomousParams.ticks_per_rev,
                ROBOT_PROFILE.autonomousParams.wheel_diameter);
        left.configureEncoder(leftOffset, ROBOT_PROFILE.autonomousParams.ticks_per_rev,
                ROBOT_PROFILE.autonomousParams.wheel_diameter);

        // Reset EncoderFollowers before running a path.
        right.reset();
        left.reset();

        // Start Gyro relative mode
        if (path.isRelative()) {
            try {
                m_drivetrain.getGyro().startRelativeMode();
            } catch (Exception e) {
                e.printStackTrace();
            }
        }
    }

    /**
     * Follow the path. This method should be called periodically until path
     * following is finished.
     */
    public void followPath() {
        // Get encoder position and invert sign if negative.
        rightPos = path.getDirection() == Direction.REVERSE ? -m_drivetrain.getRightEncoderTicks()
                : m_drivetrain.getRightEncoderTicks();
        leftPos = path.getDirection() == Direction.REVERSE ? -m_drivetrain.getLeftEncoderTicks()
                : m_drivetrain.getLeftEncoderTicks();

        double rightPower, leftPower;
        double sign = path.getDirection() == Direction.REVERSE ? -1 : 1;

        //double intercept = getDistanceToEnd() >= 1.0 ? gearParams.vIntercept : getDistanceToEnd() <= 0.5 ? -gearParams.vIntercept : 0.0;
        double intercept = gearParams.vIntercept;

        rightPower = sign * (intercept + right.calculate(rightPos));
        leftPower = sign * (intercept + left.calculate(leftPos));

        // Calculate gyro correction by multiplying the difference between the current
        // angle and the desired angle by a constant.
        double rawHeading = 0.0;
        try {
            rawHeading = path.isRelative() ? m_drivetrain.getGyro().getRelativeYaw() : m_drivetrain.getGyro().getYaw();
        } catch (Exception e) {
            e.printStackTrace();
        }
        double toAdd = path.getDirection() == Direction.REVERSE ? 180 : 0;
        double angleSetpoint = Pathfinder.boundHalfDegrees(Pathfinder.r2d(right.getHeading()) + toAdd);
        gyroError = Pathfinder.boundHalfDegrees(angleSetpoint - rawHeading);
        double turn = gyroP * gyroError;

        // Count power overflow to motors.
        if (Math.abs(leftPower) > 1 || Math.abs(rightPower) > 1) {
            motorOverflowCount++;
            System.out.println("MOTOR OVERFLOW WARNING: " + motorOverflowCount);
        }

        // Apply power to motors.
        m_drivetrain.bumbleDrive.setLeftRightMotorOutputs(leftPower - turn, rightPower + turn, false);
    }

    public void endPath() {
        if (path.isRelative()) {
            m_drivetrain.getGyro().endRelativeMode();
        }
    }

    /**
     * Returns true if path following has finished.
     * 
     * @return true if path following has finished.
     */
    public boolean isPathFinished() {
        return left.isFinished() && right.isFinished();
    }

    /**
     * 
     * @return The percent of path that the robot has followed.
     */
    public double getCurrentPercentOfPath() {
        try {
            return left.getSegment().position / trajectoryLength;
        } catch (NullPointerException e) {
            return 0.0;
        }
    }

    /**
     * @return the current distance to the end of the path.
     */
    public double getDistanceToEnd() {
        try {
            return trajectoryLength - left.getSegment().position;
        } catch (NullPointerException e) {
            return 3.339;
        }
    }

    /**
     * @return the number of times a value of more than one was sent to one of the
     *         motors.
     */
    public int getMotorOverflowCount() {
        return motorOverflowCount;
    }

    /**
     * @return the current error for the right side of the drivetrain.
     */
    public double getCurrentRightError() {
        return getWantedRightPos() - getCurrentRightPos();
    }

    /**
     * Gets the wanted position for the right side of the drivetrain.
     * 
     * @return the wanted position for the right side of the drivetrain.
     */
    public double getWantedRightPos() {
        return right.getSegment().position;
    }

    /**
     * Gets the current position for the right side of the drivetrain.
     * 
     * @return the current position for the right side of the drivetrain.
     */
    public double getCurrentRightPos() {
        return ((double) (rightPos - rightOffset) / ROBOT_PROFILE.autonomousParams.ticks_per_rev)
                * (ROBOT_PROFILE.autonomousParams.wheel_diameter * Math.PI);
    }

    /**
     * @return the current error for the left side of the drivetrain.
     */
    public double getCurrentLeftError() {
        return getWantedLeftPos() - getCurrentLeftPos();
    }

    /**
     * Gets the wanted position for the left side of the drivetrain.
     * 
     * @return the wanted position for the left side of the drivetrain.
     */
    public double getWantedLeftPos() {
        return left.getSegment().position;
    }

    /**
     * Gets the current position for the left side of the drivetrain.
     * 
     * @return the current position for the left side of the drivetrain.
     */
    public double getCurrentLeftPos() {
        return ((double) (leftPos - leftOffset) / ROBOT_PROFILE.autonomousParams.ticks_per_rev)
                * (ROBOT_PROFILE.autonomousParams.wheel_diameter * Math.PI);
    }

    /**
     * @return the current gyro error.
     */
    public double getCurrentGyroError() {
        return gyroError;
    }
}