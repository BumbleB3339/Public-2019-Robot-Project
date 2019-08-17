package frc.bumblelib.bumblelib_autonomous.pathing;

import frc.robot.profiles.InitProfiles;
import jaci.pathfinder.Pathfinder;
import jaci.pathfinder.Waypoint;

public class BumbleWaypoint {
    private double x;
    private double y;
    private double angle;
    private RobotReferencePoint robotReferencePoint;

    public BumbleWaypoint(double x, double y, double angle, RobotReferencePoint robotReferencePoint) {
        this.x = x;
        this.y = y;
        this.angle = angle;
        this.robotReferencePoint = robotReferencePoint;
    }

    /**
     * @param x the x to add
     */
    public void addX(double x) {
        this.x += x;
    }

    /**
     * @return the x
     */
    public double getX() {
        return x;
    }

    /**
     * @return the y
     */
    public double getY() {
        return y;
    }

    /**
     * @return the angle
     */
    public double getAngle() {
        return angle;
    }

    public Waypoint toJaciWaypoint() {
        return new Waypoint(this.y, InitProfiles.FIELD_PROFILE.getFieldWidth() - this.x, Pathfinder.d2r(angle));
    }

    public void transformRobotReferencePoint(RobotReferencePoint targeRobotReferencePoint) {
        // x = X * cos(theta) - Y * sin(theta)
        // y = X * sin(theta) + Y * cos(theta)

        // new x calculation
        double preOffsetFieldX = this.robotReferencePoint.getX() * Math.cos(Math.toRadians(getAngle()))
                - this.robotReferencePoint.getY() * Math.sin(Math.toRadians(getAngle()));
        double fieldXOffset = getX() - preOffsetFieldX;
        double postOffsetFieldX = targeRobotReferencePoint.getX() * Math.cos(Math.toRadians(getAngle()))
                - targeRobotReferencePoint.getY() * Math.sin(Math.toRadians(getAngle())) + fieldXOffset;

        // new y calculation
        double preOffsetFieldY = this.robotReferencePoint.getX() * Math.sin(Math.toRadians(getAngle()))
                + this.robotReferencePoint.getY() * Math.cos(Math.toRadians(getAngle()));
        double fieldYOffset = getY() - preOffsetFieldY;
        double postOffsetFieldY = targeRobotReferencePoint.getX() * Math.sin(Math.toRadians(getAngle()))
                + targeRobotReferencePoint.getY() * Math.cos(Math.toRadians(getAngle())) + fieldYOffset;

        this.robotReferencePoint = targeRobotReferencePoint;
        this.x = postOffsetFieldX;
        this.y = postOffsetFieldY;
    }

    @Override
    public String toString(){
        return String.format("X: %.2f, Y: %.2f, Angle: %.1f " ,getX(), getY(), getAngle());
    }
}
