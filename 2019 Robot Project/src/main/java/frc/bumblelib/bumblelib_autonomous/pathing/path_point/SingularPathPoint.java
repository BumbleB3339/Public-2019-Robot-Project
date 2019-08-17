package frc.bumblelib.bumblelib_autonomous.pathing.path_point;

import frc.bumblelib.bumblelib_autonomous.pathing.BumbleWaypoint;
import frc.bumblelib.bumblelib_autonomous.pathing.PathPointAdjustment;
import frc.bumblelib.bumblelib_autonomous.pathing.RobotReferencePoint;
import frc.bumblelib.bumblelib_autonomous.pathing.enums.Alliance;
import frc.bumblelib.bumblelib_autonomous.pathing.enums.Side;
import frc.bumblelib.bumblelib_autonomous.pathing.field_point.SingularFieldPoint;
import frc.robot.profiles.InitProfiles;
import jaci.pathfinder.Pathfinder;
import jaci.pathfinder.Waypoint;

public final class SingularPathPoint extends PathPoint {

    private SingularFieldPoint singularFieldPoint;
    private PathPointAdjustment redAdjustment, blueAdjustment;

    /**
     * Consturcts a new Singular Path Point
     * 
     * @param singularFieldPoint  the field point that the pathpoint relates to.
     * @param heading             the angle of the robot when reaching this path
     *                            point.
     * @param robotReferencePoint the point in the robot the path point refers to.
     */
    public SingularPathPoint(SingularFieldPoint singularFieldPoint, double heading,
            RobotReferencePoint robotReferencePoint) {
        super(robotReferencePoint, heading);

        this.singularFieldPoint = singularFieldPoint;

        this.redAdjustment = new PathPointAdjustment();
        this.blueAdjustment = new PathPointAdjustment();
    }

    /**
     * Adjusts the Singular Path Point values
     * 
     * @param dX     the distance on the X axis.
     * @param dY     the distance on the Y axis.
     * @param dTheta the angle that we need to add.
     */
    public void adjustRed(double dX, double dY, double dTheta) {
        redAdjustment.addAdjustment(dX, dY, dTheta);
    }

    /**
     * Adjusts the Singular Path Point values in blue alliance only.
     * 
     * @param dX     the distance on the X axis.
     * @param dY     the distance on the Y axis.
     * @param dTheta the angle that we need to add.
     */
    public void adjustBlue(double dX, double dY, double dTheta) {
        blueAdjustment.addAdjustment(dX, dY, dTheta);
    }

    /**
     * Adjusts the Singular Path Point values in both alliances.
     * 
     * @param dX     the distance on the X axis.
     * @param dY     the distance on the Y axis.
     * @param dTheta the angle that we need to add.
     */
    public void adjustBoth(double dX, double dY, double dTheta) {
        adjustRed(dX, dY, dTheta);
        adjustBlue(dX, dY, dTheta);
    }

    /**
     * Gets the adjustment in the requested alliance.
     * 
     * @param alliance the allaince to return the adjustment from.
     * @return the adjustment in the requested alliance.
     */
    private PathPointAdjustment getAdjustment(Alliance alliance) {
        switch (alliance) {
        case RED:
            return redAdjustment;
        case BLUE:
            return blueAdjustment;
        default:
            return null;
        }
    }

    /**
     * Gets the waypoint (in BumbleB Coordinate system).
     * 
     * @param alliance the allaince to take the point from.
     * @return the waypoint (in BumbleB Coordinate system).
     */
    public BumbleWaypoint getBumbleWaypoint(Alliance alliance) {
        BumbleWaypoint outputBumbleWaypoint = new BumbleWaypoint(
                singularFieldPoint.getX(alliance) + getAdjustment(alliance).getdX(),
                singularFieldPoint.getY(alliance) + getAdjustment(alliance).getdY(),
                heading + getAdjustment(alliance).getdTheta(), robotReferencePoint);
        outputBumbleWaypoint
                .transformRobotReferencePoint(InitProfiles.ROBOT_PROFILE.robotReferencePoints.jaciCenterOfRotation);

        return outputBumbleWaypoint;
    }

    @Override
    public SingularPathPoint rotateRelative(double dTheta) {
        BumbleWaypoint red = new BumbleWaypoint(singularFieldPoint.getX(Alliance.RED),
                singularFieldPoint.getY(Alliance.RED), heading, robotReferencePoint);
        BumbleWaypoint blue = new BumbleWaypoint(singularFieldPoint.getX(Alliance.BLUE),
                singularFieldPoint.getY(Alliance.BLUE), heading, robotReferencePoint);

        red.transformRobotReferencePoint(InitProfiles.ROBOT_PROFILE.robotReferencePoints.realCenterOfRotation);
        blue.transformRobotReferencePoint(InitProfiles.ROBOT_PROFILE.robotReferencePoints.realCenterOfRotation);

        return new SingularPathPoint(new SingularFieldPoint(red.getX(), blue.getX(), red.getY(), blue.getY()),
                Pathfinder.boundHalfDegrees(heading + dTheta),
                InitProfiles.ROBOT_PROFILE.robotReferencePoints.realCenterOfRotation);
    }

    @Override
    public SingularPathPoint rotateAbsolute(double theta) {
        return rotateRelative(Pathfinder.boundHalfDegrees(theta - heading));
    }

    public Waypoint getWaypoint(Alliance alliance) {
        return getBumbleWaypoint(alliance).toJaciWaypoint();
    }

    @Override
    public Waypoint getWaypoint(Alliance alliance, Side autoSide) {
        return getWaypoint(alliance);
    }

    /**
     * WILL RETURN THIS OBJECT UPON EXCEPTION 
     */
    @Override
    public SingularPathPoint getClone() {
        
        try {
            return ((SingularPathPoint) this.clone());
        } catch (CloneNotSupportedException e) {
            return this;
        }
    }

    @Override
    protected Object clone() throws CloneNotSupportedException {
        return super.clone();
    }
}
