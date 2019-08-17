/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.bumblelib.util;

import java.util.ArrayList;

/**
 * Add your docs here.
 */

public class LimitedSizeQueue<K> extends ArrayList<K> {

    private static final long serialVersionUID = -8995913842302780059L;
    private int maxSize;

    public LimitedSizeQueue(int size) {
        this.maxSize = size;
    }

    // Add new value the queue and remove the oldest
    public boolean add(K k) {
        boolean r = super.add(k);
        if (size() > maxSize) {
            remove(0);
        }
        return r;
    }

    // Returns the youngest value
    public K getYoungest() {
        return get(size() - 1);
    }

    // Returbs the oldest value
    public K getOldest() {
        return get(0);
    }
}
