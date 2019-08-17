/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.bumblelib.util;

import edu.wpi.first.wpilibj.buttons.Trigger;
import frc.robot.Robot;

/**
 * Add your docs here.
 */
public abstract class BumbleTrigger extends Trigger {

    @Override
    public final boolean get() {
        return Robot.isOperationActive && triggerCondition();
    }

    /**
     * Returns whether or not the trigger is active.
     *
     * <p>
     * This method will be called repeatedly a command is linked to the Trigger.
     *
     * @return whether or not the trigger condition is active.
     */
    public abstract boolean triggerCondition();
}
