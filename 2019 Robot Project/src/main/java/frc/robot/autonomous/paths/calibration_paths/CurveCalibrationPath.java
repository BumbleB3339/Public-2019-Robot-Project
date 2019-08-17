package frc.robot.autonomous.paths.calibration_paths;

import static frc.robot.profiles.InitProfiles.ROBOT_PROFILE;

import frc.bumblelib.bumblelib_autonomous.pathing.enums.Direction;
import frc.bumblelib.bumblelib_autonomous.pathing.path.SingularPath;
import frc.bumblelib.bumblelib_autonomous.pathing.path_point.SingularPathPoint;
import frc.robot.autonomous.FieldPoints;
import frc.robot.autonomous.InitialPathPoints;
import frc.robot.subsystems.Drivetrain.Gear;

public class CurveCalibrationPath extends SingularPath {
    public CurveCalibrationPath() {
        super("CurveCalibrationPath", Gear.SPEED_GEAR, 2.0, Direction.FORWARD,
                ROBOT_PROFILE.pathfinderParams.speedGear.slowPathPreset);
        this.add(InitialPathPoints.calibrationInitial);
        this.add(new SingularPathPoint(FieldPoints.curveCalibrationEnd, 0.0,
                ROBOT_PROFILE.robotReferencePoints.rearCenter));
        this.setRelative();
    }
}