package frc.robot.autonomous;

import static frc.robot.profiles.InitProfiles.ROBOT_PROFILE;

import frc.bumblelib.bumblelib_autonomous.pathing.path_point.SingularPathPoint;
import frc.bumblelib.bumblelib_autonomous.pathing.path_point.SymmetricPathPoint;

public class InitialPathPoints {

    // Calibration.
    public static SingularPathPoint calibrationInitial;

    // Platform.
    public static SymmetricPathPoint level1OuterBackwardFacing, level1OuterForwardFacing;
    public static SymmetricPathPoint level1Inner;

    // Post platform drop.
    public static SymmetricPathPoint postOuterDropForwardFacing, postOuterDropBackwardFacing;

    public static double robotDistanceFromLevel2EdgeBackwardFacing = 0.36;
    public static double robotDistanceFromLevel2EdgeForwardFacing = 0.36;
    public InitialPathPoints() {
        calibrationInitial = new SingularPathPoint(FieldPoints.zeroPoint, 0.0,
                ROBOT_PROFILE.robotReferencePoints.rearCenter);

        level1OuterBackwardFacing = new SymmetricPathPoint(FieldPoints.level1OuterCorner.displace(-(ROBOT_PROFILE.autonomousParams.robot_width / 2.0), robotDistanceFromLevel2EdgeBackwardFacing),  
            180.0, ROBOT_PROFILE.robotReferencePoints.rearCenter, false);

        level1OuterForwardFacing = new SymmetricPathPoint(FieldPoints.level1OuterCorner.displace(-(ROBOT_PROFILE.autonomousParams.robot_width / 2.0), robotDistanceFromLevel2EdgeForwardFacing), 
            0.0, ROBOT_PROFILE.robotReferencePoints.frontCenter, false);

        postOuterDropBackwardFacing = new SymmetricPathPoint(FieldPoints.level1OuterCorner.displace(-(ROBOT_PROFILE.autonomousParams.robot_width / 2.0), 0.3), 
            180.0, ROBOT_PROFILE.robotReferencePoints.frontCenter, false); 
        
        postOuterDropForwardFacing = new SymmetricPathPoint(FieldPoints.level1OuterCorner.displace(-(ROBOT_PROFILE.autonomousParams.robot_width / 2.0), -0.15), 
            0.0, ROBOT_PROFILE.robotReferencePoints.rearCenter, false); 
        
        level1Inner = new SymmetricPathPoint(FieldPoints.level23ConnectionOnLevel1Edge.displace((ROBOT_PROFILE.autonomousParams.robot_width / 2.0), robotDistanceFromLevel2EdgeForwardFacing), 
            0.0, ROBOT_PROFILE.robotReferencePoints.frontCenter, false);
    }
}
