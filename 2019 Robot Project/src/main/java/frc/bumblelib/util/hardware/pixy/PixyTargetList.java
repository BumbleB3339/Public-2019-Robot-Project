package frc.bumblelib.util.hardware.pixy;

import java.util.ArrayList;

public class PixyTargetList extends ArrayList<PixyTarget> {

    // We have no idea what this "long" is, but it asks for it
    private static final long serialVersionUID = -1707639091393431922L;

    /**
     * Constructs an empty PixyTargetList.
     */
    public PixyTargetList() {
        super();
    }

    /**
     * Returns a PixyTarget for the biggest target in this PixyTargetList.
     * 
     * @return A PixyTarget. null will be returned if no such target
     *         is detected.
     */
    public PixyTarget getBiggest() {
        if (this.size() > 0) {
            PixyTarget biggestTarget = get(0);
            for (PixyTarget pixyTarget : this) {
                if (pixyTarget.getArea() > biggestTarget.getArea()) {
                    biggestTarget = pixyTarget;
                }
            }
            return biggestTarget;
        } else {
            return null;
        }
    }

    static int count = 0;

    /**
     * Returns a PixyTarget for the leftmost target in this PixyTargetList.
     * 
     * @return A PixyTarget. null will be returned if no such target
     *         is detected.
     */
    public PixyTarget getLeftmost() {

        try {
            PixyTarget leftmostTarget = get(0);
            for (PixyTarget pixyTarget : this) {
                if (pixyTarget.getX() < leftmostTarget.getX()) {
                    leftmostTarget = pixyTarget;
                }
            }
            return leftmostTarget;
        } catch (Exception e) {
            // System.out.println("No target found");
            return null;
        }
    }

    /**
     * Returns a PixyTarget for the rightmost target in this PixyTargetList.
     * 
     * @return A PixyTarget. null will be returned if no such target
     *         is detected.
     */
    public PixyTarget getRightmost() {
        try {
            PixyTarget rightmostTarget = get(0);
            for (PixyTarget pixyTarget : this) {
                if (pixyTarget.getX() > rightmostTarget.getX()) {
                    rightmostTarget = pixyTarget;
                }
            }
            return rightmostTarget;
        } catch (Exception e) {
            // System.out.println("No target found");
            return null;
        }
    }

    /**
     * Returns a PixyTarget for the highest target in this PixyTargetList.
     * 
     * @return A PixyTarget. null will be returned if no such target
     *         is detected.
     */
    public PixyTarget getHighest() {
        try {
            PixyTarget highestTarget = get(0);
            for (PixyTarget pixyTarget : this) {
                if (pixyTarget.getY() > highestTarget.getY()) {
                    highestTarget = pixyTarget;
                }
            }
            return highestTarget;
        } catch (Exception e) {
            // System.out.println("No target found");
            return null;
        }
    }

    /**
     * Returns a PixyTarget for the most centered target in this PixyTargetList.
     * 
     * @return A PixyTarget. null will be returned if no such target
     *         is detected.
     */
    public PixyTarget getCenter(int screenCenter) {
        try {
            PixyTarget centerTarget = get(0);
            for (PixyTarget pixyTarget : this) {
                if (Math.abs(pixyTarget.getX() - screenCenter) < Math.abs(centerTarget.getX() - screenCenter)) {
                    centerTarget = pixyTarget;
                }
            }
            return centerTarget;
        } catch (Exception e) {
            // System.out.println("No target found");
            return null;
        }
    }
}
