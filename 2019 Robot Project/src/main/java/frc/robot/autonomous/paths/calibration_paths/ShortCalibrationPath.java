package frc.robot.autonomous.paths.calibration_paths;

import static frc.robot.profiles.InitProfiles.ROBOT_PROFILE;

import frc.bumblelib.bumblelib_autonomous.pathing.enums.Direction;
import frc.bumblelib.bumblelib_autonomous.pathing.path.SingularPath;
import frc.bumblelib.bumblelib_autonomous.pathing.path_point.SingularPathPoint;
import frc.robot.autonomous.FieldPoints;
import frc.robot.subsystems.Drivetrain.Gear;

public class ShortCalibrationPath extends SingularPath {
        public ShortCalibrationPath() {
                super("ShortCalibrationPath", Gear.SPEED_GEAR, 2.5, Direction.FORWARD,
                                ROBOT_PROFILE.pathfinderParams.speedGear.slowPathPreset);

                this.add(new SingularPathPoint(FieldPoints.zeroPoint, 0.0,
                                ROBOT_PROFILE.robotReferencePoints.rearCenter));
                this.add(new SingularPathPoint(FieldPoints.shortCalibrationEnd, 0.0,
                                ROBOT_PROFILE.robotReferencePoints.rearCenter));

                this.setRelative();
        }
}