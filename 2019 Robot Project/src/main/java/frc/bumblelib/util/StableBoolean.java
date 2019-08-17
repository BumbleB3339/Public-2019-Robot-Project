/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.bumblelib.util;

/**
 * Add your docs here.
 */
public class StableBoolean {

    private int trueThreshold = 1; // How many iterations it takes to become true, 1 is for immidiate change
    private int falseThreshold = 1; // How many iterations it takes to become false, 1 is for immidiate change
    private int counter = 0;
    private boolean stableValue = false;

    public StableBoolean(int trueThreshold, int falseThreshold, boolean initialValue) {
        this.trueThreshold = trueThreshold;
        this.falseThreshold = falseThreshold;
        this.stableValue = initialValue;
    }

    public StableBoolean(int trueThreshold, int falseThreshold) {
        this(trueThreshold, falseThreshold, false);
    }

    public boolean get() {
        return stableValue;
    }

    public void update(boolean value) {
        if (stableValue == true) {
            if (value == true) {
                counter = 0;
            } else {
                counter++;
                if (counter >= falseThreshold) {
                    stableValue = false;
                    counter = 0;
                }
            }
        } else { 
            if (value == false) {
                counter = 0;
            } else {
                counter++;
                if (counter >= trueThreshold) {
                    stableValue = true;
                    counter = 0;
                }
            }
        }
    }

    public void forceValue(boolean value) {
        stableValue = value;
        counter = 0;
    }
}
