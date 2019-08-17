package frc.robot.autonomous.paths;

import static frc.robot.profiles.InitProfiles.ROBOT_PROFILE;

import frc.bumblelib.bumblelib_autonomous.pathing.enums.Direction;
import frc.bumblelib.bumblelib_autonomous.pathing.path.SymmetricPath;
import frc.bumblelib.bumblelib_autonomous.pathing.path_point.SymmetricPathPoint;
import frc.robot.autonomous.FieldPoints;
import frc.robot.profiles.InitProfiles;
import frc.robot.subsystems.Drivetrain.Gear;

public class FeederToCloseShipPlacement extends SymmetricPath {

    private static final double MAX_VEL = 2.5;
    private static final double CLOSE_SHIP_PLACEMENT_ANGLE = 0.0;
    private static final double CLOSE_SHIP_PLACEMENT_DISTANCE = 1.0;
    
    private static final double MIDPOINT_Y_DISPLACE = 2.5;
    
    public FeederToCloseShipPlacement() {
        super("FeederToCloseShipPlacement", Gear.SPEED_GEAR, MAX_VEL, Direction.FORWARD,
                ROBOT_PROFILE.pathfinderParams.speedGear.fastPathPreset);

        add(new SymmetricPathPoint(FieldPoints.feederCenter, 
            0.0, InitProfiles.ROBOT_PROFILE.robotReferencePoints.rearCenter, false));
        
        add(new SymmetricPathPoint(FieldPoints.sideShipPlacementClose.displace(CLOSE_SHIP_PLACEMENT_DISTANCE, -MIDPOINT_Y_DISPLACE), 
            CLOSE_SHIP_PLACEMENT_ANGLE, InitProfiles.ROBOT_PROFILE.robotReferencePoints.jaciCenterOfRotation, false));
        
        add(new SymmetricPathPoint(FieldPoints.sideShipPlacementClose.displace(CLOSE_SHIP_PLACEMENT_DISTANCE, -0.2), 
            CLOSE_SHIP_PLACEMENT_ANGLE, InitProfiles.ROBOT_PROFILE.robotReferencePoints.jaciCenterOfRotation, false));
    }                                                                                       
}