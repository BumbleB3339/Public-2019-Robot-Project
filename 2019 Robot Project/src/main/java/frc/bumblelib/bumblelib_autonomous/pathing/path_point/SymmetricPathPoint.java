package frc.bumblelib.bumblelib_autonomous.pathing.path_point;

import frc.bumblelib.bumblelib_autonomous.pathing.BumbleWaypoint;
import frc.bumblelib.bumblelib_autonomous.pathing.PathPointAdjustment;
import frc.bumblelib.bumblelib_autonomous.pathing.RobotReferencePoint;
import frc.bumblelib.bumblelib_autonomous.pathing.enums.AdjustmentSide;
import frc.bumblelib.bumblelib_autonomous.pathing.enums.Alliance;
import frc.bumblelib.bumblelib_autonomous.pathing.enums.Side;
import frc.bumblelib.bumblelib_autonomous.pathing.field_point.SymmetricFieldPoint;
import frc.robot.profiles.InitProfiles;
import jaci.pathfinder.Pathfinder;
import jaci.pathfinder.Waypoint;

public final class SymmetricPathPoint extends PathPoint implements Cloneable {

    private SymmetricFieldPoint symmetricFieldPoint;
    private PathPointAdjustment redRightAdjustment, redLeftAdjustment, blueRightAdjustment, blueLeftAdjustment;
    private boolean isOtherSide; // right path point uses left field point, left path point uses right field
                                 // point

    // heading positive outside
    /**
     * Constructs a new Symmetric Path Point with given values.
     * 
     * @param symmetricFieldPoint the field point that the path point relates to.
     * @param heading             the angle of the robot when reaching this path
     *                            point.
     * @param robotReferencePoint the point in the robot the path point refers to
     * @param isOtherSide         the side of the path (Left/Right).
     */
    public SymmetricPathPoint(SymmetricFieldPoint symmetricFieldPoint, double heading,
            RobotReferencePoint robotReferencePoint, boolean isOtherSide) {
        super(robotReferencePoint, heading);

        this.symmetricFieldPoint = symmetricFieldPoint;
        this.isOtherSide = isOtherSide;

        this.redRightAdjustment = new PathPointAdjustment();
        this.blueRightAdjustment = new PathPointAdjustment();

        this.redLeftAdjustment = new PathPointAdjustment();
        this.blueLeftAdjustment = new PathPointAdjustment();
    }

    /**
     * Gets the physical side of the robot (Left/Right)
     * 
     * @param autoSide the side that the robot should be.
     * @return the physical side of the robot (Left/Right)
     */
    private Side getPhysicalSide(Side autoSide) {
        return this.isOtherSide ? autoSide.getOtherSide() : autoSide;
    }

    /**
     * Gets the side which the adjustment applies to.
     * 
     * @param autoAdjustmentSide
     * @return
     */
    private AdjustmentSide getPhysicalAdjustmentSide(AdjustmentSide autoAdjustmentSide) {
        return this.isOtherSide ? autoAdjustmentSide.getOtherSide() : autoAdjustmentSide;
    }

    // Rgarding adjustment logic:
    // isOtherSide = false: angle/x is positive outside ; isOtherSide = true:
    // angle/x is positive inside (practically outside)
    // adjustmentSide is the side for which the pathpoint is condigures, not of the
    // physical side. when isOtherSide = true and you adjust the right side it
    // adjusts the left field point.

    /**
     * Adjusts the Red alliance symmetric path point
     * 
     * @param autoAdjustmentSide the side where the austment occurs.
     * @param dX                 the distance on the X axis.
     * @param dY                 the distance on the Y axis.
     * @param dTheta             the angle that we need to add.
     */
    public void adjustRed(AdjustmentSide autoAdjustmentSide, double dX, double dY, double dTheta) {
        switch (getPhysicalAdjustmentSide(autoAdjustmentSide)) {
        case RIGHT:
            redRightAdjustment.addAdjustment(dX, dY, -dTheta);
            break;
        case LEFT:
            redLeftAdjustment.addAdjustment(-dX, dY, dTheta);
            break;
        case BOTH:
            redRightAdjustment.addAdjustment(dX, dY, -dTheta);
            redLeftAdjustment.addAdjustment(-dX, dY, dTheta);
            break;
        }
    }

    /**
     * Adjusts the Blue alliance symmetric path point
     * 
     * @param side   the side where the austment occurs.
     * @param dX     the distance on the X axis.
     * @param dY     the distance on the Y axis.
     * @param dTheta the angle that we need to add.
     */
    public void adjustBlue(AdjustmentSide side, double dX, double dY, double dTheta) {
        switch (side) {
        case RIGHT:
            blueRightAdjustment.addAdjustment(dX, dY, -dTheta);
        case LEFT:
            blueLeftAdjustment.addAdjustment(-dX, dY, dTheta);
            break;
        case BOTH:
            blueRightAdjustment.addAdjustment(dX, dY, -dTheta);
            blueLeftAdjustment.addAdjustment(-dX, dY, dTheta);
            break;
        }
    }

    /**
     * Adjusts both alliances symmetric path point
     * 
     * @param side   the side where the austment occurs.
     * @param dX     the distance on the X axis.
     * @param dY     the distance on the Y axis.
     * @param dTheta the angle that we need to add.
     */
    public void adjustBoth(AdjustmentSide side, double dX, double dY, double dTheta) {
        adjustRed(side, dX, dY, dTheta);
        adjustBlue(side, dX, dY, dTheta);
    }

    /**
     * Get the adjusments to a chosen side and alliance.
     * 
     * @param alliance the alliance where the adjustment is.
     * @param autoSide the side where the adjustment is.
     * @return the adjusments to a chosen side and alliance.
     */
    private PathPointAdjustment getAdjustment(Alliance alliance, Side autoSide) {
        switch (alliance) {
        case RED:
            switch (getPhysicalSide(autoSide)) {
            case RIGHT:
                return redRightAdjustment;
            case LEFT:
                return redLeftAdjustment;
            }
        case BLUE:
            switch (getPhysicalSide(autoSide)) {
            case RIGHT:
                return blueRightAdjustment;
            case LEFT:
                return blueLeftAdjustment;
            }
        default:
            return null;
        }
    }

    /**
     * Gets the waypoint according to BumbleB Coordination system with given side
     * and alliance.
     * 
     * @param alliance the alliance where the path point is.
     * @param autoSide the side chosen.
     * @return the waypoint according to BumbleB Coordination system with given side
     *         and alliance.
     */
    public BumbleWaypoint getBumbleWaypoint(Alliance alliance, Side autoSide) {
        Side physicalSide = getPhysicalSide(autoSide);
        BumbleWaypoint outputBumbleWaypoint = new BumbleWaypoint(
                symmetricFieldPoint.getX(alliance, physicalSide) + getAdjustment(alliance, autoSide).getdX(),
                symmetricFieldPoint.getY(alliance, physicalSide) + getAdjustment(alliance, autoSide).getdY(),
                heading * (autoSide == Side.RIGHT ? -1 : 1) + getAdjustment(alliance, autoSide).getdTheta(),
                robotReferencePoint);
        outputBumbleWaypoint
                .transformRobotReferencePoint(InitProfiles.ROBOT_PROFILE.robotReferencePoints.jaciCenterOfRotation);

        return outputBumbleWaypoint;
    }

    // positive outside
    @Override
    public SymmetricPathPoint rotateRelative(double dTheta) {
        BumbleWaypoint redRight = new BumbleWaypoint(symmetricFieldPoint.getX(Alliance.RED, Side.RIGHT),
                symmetricFieldPoint.getY(Alliance.RED, Side.RIGHT), -heading, robotReferencePoint);
        BumbleWaypoint redLeft = new BumbleWaypoint(symmetricFieldPoint.getX(Alliance.RED, Side.LEFT),
                symmetricFieldPoint.getY(Alliance.RED, Side.LEFT), heading, robotReferencePoint);

        BumbleWaypoint blueRight = new BumbleWaypoint(symmetricFieldPoint.getX(Alliance.BLUE, Side.RIGHT),
                symmetricFieldPoint.getY(Alliance.BLUE, Side.RIGHT), -heading, robotReferencePoint);
        BumbleWaypoint blueLeft = new BumbleWaypoint(symmetricFieldPoint.getX(Alliance.BLUE, Side.LEFT),
                symmetricFieldPoint.getY(Alliance.BLUE, Side.LEFT), heading, robotReferencePoint);

        redRight.transformRobotReferencePoint(InitProfiles.ROBOT_PROFILE.robotReferencePoints.realCenterOfRotation);
        redLeft.transformRobotReferencePoint(InitProfiles.ROBOT_PROFILE.robotReferencePoints.realCenterOfRotation);

        blueRight.transformRobotReferencePoint(InitProfiles.ROBOT_PROFILE.robotReferencePoints.realCenterOfRotation);
        blueLeft.transformRobotReferencePoint(InitProfiles.ROBOT_PROFILE.robotReferencePoints.realCenterOfRotation);

        return new SymmetricPathPoint(
                new SymmetricFieldPoint(redRight.getX(), blueRight.getX(), redRight.getY(), blueRight.getY(),
                        redLeft.getX(), blueLeft.getX(), redLeft.getY(), blueLeft.getY()),
                Pathfinder.boundHalfDegrees(heading + dTheta),
                InitProfiles.ROBOT_PROFILE.robotReferencePoints.realCenterOfRotation, isOtherSide);
    }

    // positive outside
    @Override
    public SymmetricPathPoint rotateAbsolute(double theta) {
        return rotateRelative(Pathfinder.boundHalfDegrees(theta - heading));
    }

    @Override
    public Waypoint getWaypoint(Alliance alliance, Side autoSide) {
        return getBumbleWaypoint(alliance, autoSide).toJaciWaypoint();
    }

    /**
     * @param isOtherSide the isOtherSide to set
     */
    public void setIsOtherSide(boolean isOtherSide) {
        this.isOtherSide = isOtherSide;
    }
    
    /**
     * WILL RETURN THIS OBJECT UPON EXCEPTION 
     */
    @Override
    public SymmetricPathPoint getClone() {
        
        try {
            return ((SymmetricPathPoint) this.clone());
        } catch (CloneNotSupportedException e) {
            return this;
        }
    }

    @Override
    protected Object clone() throws CloneNotSupportedException {
        return super.clone();
    }
}
