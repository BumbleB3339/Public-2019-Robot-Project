/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.bumblelib.util;

import java.util.function.BooleanSupplier;

/**
 * Add your docs here.
 */
public class BumbleSupplierTrigger extends BumbleTrigger {

    private BooleanSupplier booleanSupplier;

    public BumbleSupplierTrigger(BooleanSupplier booleanSupplier){
        this.booleanSupplier = booleanSupplier;
    }

    @Override
    public boolean triggerCondition() {
        return booleanSupplier.getAsBoolean();
    }

    /**
     * @return the booleanSupplier
     */
    public BooleanSupplier getBooleanSupplier() {
        return booleanSupplier;
    }
}
