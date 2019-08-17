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
public class PathAtPrecentDest extends Trigger {

  PathFollower follower;
  Double percent;

  public PathAtPrecentDest(PathFollower follower, Double percent) {
    this.follower = follower;
    this.percent = percent;
  }

  @Override
  public boolean get() {
    return follower.getCurrentPercentOfPath() >= percent;
  }
}
