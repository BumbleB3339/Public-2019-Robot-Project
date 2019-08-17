/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.triggers;

import frc.bumblelib.util.BumbleTrigger;
import frc.robot.RobotGameState.RobotAction;
import frc.robot.RobotGameStateManager;

/**
 * Add your docs here.
 */
public class TriggerCanceledClimbMode extends BumbleTrigger {
    private boolean lastClimbMode = false;

    @Override
    public boolean triggerCondition() {
        boolean output = false;
        if (lastClimbMode && !(RobotGameStateManager.currentGameState.robotAction == RobotAction.CLIMB)) {
            output = true;
        }
        lastClimbMode = RobotGameStateManager.currentGameState.robotAction == RobotAction.CLIMB;
        return output;
    }
}
