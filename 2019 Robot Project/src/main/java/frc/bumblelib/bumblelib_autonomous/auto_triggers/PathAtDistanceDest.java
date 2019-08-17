/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.bumblelib.bumblelib_autonomous.auto_triggers;

import edu.wpi.first.wpilibj.buttons.Trigger;
import frc.bumblelib.bumblelib_autonomous.pathing.PathFollower;

/**
 * Add your docs here.
 */
public class PathAtDistanceDest extends Trigger {

  PathFollower follower;
  Double distance;

  public PathAtDistanceDest(PathFollower follower, Double distance) {
    this.follower = follower;
    this.distance = distance;
  }

  @Override
  public boolean get() {
    return follower.getDistanceToEnd() >= distance;
  }
}
