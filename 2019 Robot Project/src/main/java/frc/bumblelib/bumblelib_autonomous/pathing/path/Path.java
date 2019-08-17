package frc.bumblelib.bumblelib_autonomous.pathing.path;

import static frc.robot.profiles.InitProfiles.ROBOT_PROFILE;

import java.util.ArrayList;

import frc.bumblelib.bumblelib_autonomous.CSVSendable;
import frc.bumblelib.bumblelib_autonomous.pathing.PathRegistry;
import frc.bumblelib.bumblelib_autonomous.pathing.enums.Alliance;
import frc.bumblelib.bumblelib_autonomous.pathing.enums.Direction;
import frc.bumblelib.bumblelib_autonomous.pathing.enums.Side;
import frc.bumblelib.bumblelib_autonomous.pathing.path_point.PathPoint;
import frc.bumblelib.util.PIDPreset;
import frc.robot.subsystems.Drivetrain.Gear;
import jaci.pathfinder.Pathfinder;
import jaci.pathfinder.Trajectory;
import jaci.pathfinder.Trajectory.FitMethod;
import jaci.pathfinder.Waypoint;

/**
 * This class represents a path.
 */
public abstract class Path implements CSVSendable{

    protected final FitMethod FIT_METHOD = FitMethod.HERMITE_QUINTIC;
    protected final int SAMPLE_RATE = Trajectory.Config.SAMPLES_HIGH;

    protected String pathName;
    protected ArrayList<PathPoint> pathPoints = new ArrayList<PathPoint>();
    protected boolean isRelative;
    private Direction direction;
    protected Gear gear;
    protected PIDPreset pidPreset;

    protected Trajectory.Config cfg;

    /**
     * Creates a new {@link Path} using the given parameters.
     * 
     * @param pathName    The name of the new {@link Path}.
     * @param maxVelocity The max velocity the path is allowed to request of the
     *                    {@link frc.robot.subsystems.Drivetrain}.
     * @param direction   The {@link Direction} of the {@link Path} - forward (from
     *                    the start to the end) or backward(from the end to the
     *                    start).
     * @param pidPreset
     */
    protected Path(String pathName, Gear gear, double maxVelocity, Direction direction, PIDPreset pidPreset) {
        this.pathName = pathName;
        this.direction = direction;
        this.isRelative = false;
        this.pidPreset = pidPreset;
        this.gear = gear;

        double maxAccel = ROBOT_PROFILE.pathfinderParams.getGearParams(gear).maxAccel;
        this.cfg = new Trajectory.Config(FIT_METHOD, SAMPLE_RATE, ROBOT_PROFILE.pathfinderParams.deltaTime, maxVelocity,
                maxAccel, ROBOT_PROFILE.pathfinderParams.maxJerk);

        PathRegistry.getInstance().add(this);
    }

    /**
     * Returns array of Jaci's Waypoints based on color and side.
     * 
     * @param alliance The alliance color of the path to generate
     * @param side     The side of th path
     * @return
     */
    public Waypoint[] getWaypoints(Alliance alliance, Side side) {
        Waypoint[] waypoints = new Waypoint[pathPoints.size()];

        for (int i = 0; i < waypoints.length; i++) {
            waypoints[i] = pathPoints.get(i).getWaypoint(alliance, side);
        }

        return waypoints;
    }

    /**
     * Returns a tajectory with the given alliance and side
     * 
     * @param alliance The alliance color of the path to generate
     * @param side     The side of th path
     * @return a tajectory with the given alliance and side
     */
    public Trajectory generateTrajectory(Alliance alliance, Side side) {
        return Pathfinder.generate(getWaypoints(alliance, side), getConfig());
    }

    /**
     * Returns a String description of this Path object.
     */
    public String getDescription(Alliance alliance, Side side) {
        String waypoints = "[";
        for (Waypoint point : getWaypoints(alliance, side)) {
            waypoints += "{x: " + point.x + ", y: " + point.y + ", angle: " + point.angle + "}, ";
        }
        waypoints = waypoints.substring(0, waypoints.length() - 2) + "]";

        return "{pathName: " + pathName + ", maxVelocity: " + this.getMaxVelocity() + ", direction: " + direction
                + ", isRelative: " + isRelative + ", pidPreset: " + pidPreset + ", waypoints: " + waypoints
                + ", consts: {fitMethod: " + cfg.fit + ", sampleRate: " + cfg.sample_count + ", dt: " + cfg.dt
                + ", maxAccel: " + cfg.max_acceleration + "maxJerk: " + cfg.max_jerk + "}" + "}";
    }

    /**
     * Generates the name for a CSV file according to 'pathName-COLOR-SIDE'
     * template.
     */
    public abstract String getNameTemplate(Alliance alliance, Side side);

    /**
     * Makes this path relative.
     */
    public void setRelative() {
        this.isRelative = true;
    }

    /**
     * Returns the final {@link PathPoint} of the {@link Path}.
     * 
     * @return The final {@link PathPoint} of the {@link Path}.
     */
    public PathPoint getFinalPathPoint() {
        return pathPoints.get(pathPoints.size() - 1);
    }

    /**
     * 
     * @return
     */
    public ArrayList<PathPoint> getPathPoints() {
        return pathPoints;
    }

    /**
     * 
     * @return
     */
    public PathPoint getInitialPathPoint() {
        return pathPoints.get(0);
    }

    /**
     * Gets the name of the path.
     * 
     * @return the name of the path.
     */
    public String getName() {
        return pathName;
    }

    /**
     * Gets the preset of the path.
     * 
     * @return the preset of the path.
     */
    public PIDPreset getPIDPreset() {
        return pidPreset;
    }

    /**
     * Gets the direction of the path.
     * 
     * @return the direction of the path.
     */
    public Direction getDirection() {
        return direction;
    }

    /**
     * Retunrs true if the path is relative and false if it is not.
     * 
     * @return true if the path is relative and false if it is not.
     */
    public boolean isRelative() {
        return isRelative;
    }

    /**
     * Returns the trajectory of the path.
     * 
     * @return the trajectory of the path.
     */
    public Trajectory.Config getConfig() {
        return cfg;
    }

    /**
     * Gets the Max Velocity of the path.
     * 
     * @return the Max Velocity of the path.
     */
    public double getMaxVelocity() {
        return cfg.max_velocity;
    }

    /**
     * Gets the drivetrain gear (Speed or Power).
     * 
     * @return the drivetrain gear (Speed or Power).
     */
    public Gear getGear() {
        return this.gear;
    }
}