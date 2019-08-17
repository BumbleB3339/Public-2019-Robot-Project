package frc.bumblelib.bumblelib_autonomous.pathing.path_point;

import frc.bumblelib.bumblelib_autonomous.pathing.RobotReferencePoint;
import frc.bumblelib.bumblelib_autonomous.pathing.enums.Alliance;
import frc.bumblelib.bumblelib_autonomous.pathing.enums.Side;
import jaci.pathfinder.Waypoint;

public abstract class PathPoint implements Cloneable {

    protected RobotReferencePoint robotReferencePoint;
    protected double heading; // when symmetric, heading is positive outside. when singular, heading is
                              // positive counter-clockwise

    // when symmetric, heading is positive outside. when singular, heading is
    // positive counter-clockwise
    protected PathPoint(RobotReferencePoint robotReferencePoint, double heading) {
        this.robotReferencePoint = robotReferencePoint;
        this.heading = heading;
    }

    /**
     * Gets the waypoint in Jaci's Coordinate system with an alliance and side.
     * 
     * @param alliance the alliance where the waypoint exsits.
     * @param autoSide the side where the waypoint exsits.
     * @return the waypoint in Jaci's Coordinate system with an alliance and side.
     */
    public abstract Waypoint getWaypoint(Alliance alliance, Side autoSide);

    /**
     * Rotates a path point relativly to another path point.
     * 
     * @param dTheta the angle to move.
     * @return a new point which is the given point but rotated.
     */
    public abstract PathPoint rotateRelative(double dTheta);

    /**
     * Rotates a path point with given angle.
     * 
     * @param theta the angle to rotate.
     * @return a new point which is the given point but rotated
     */
    public abstract PathPoint rotateAbsolute(double theta);

    public abstract PathPoint getClone();

    /**
     * @param robotReferencePoint the robotReferencePoint to set
     */
    public void setRobotReferencePoint(RobotReferencePoint robotReferencePoint) {
        this.robotReferencePoint = robotReferencePoint;
    }

    @Override
    protected Object clone() throws CloneNotSupportedException {
        return super.clone(); 
    }
}
