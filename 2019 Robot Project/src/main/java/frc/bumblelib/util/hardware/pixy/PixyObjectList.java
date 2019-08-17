package frc.bumblelib.util.hardware.pixy;

import java.util.ArrayList;

public class PixyObjectList extends ArrayList<PixyObject> {

    // We have no idea what this "long" is, but it asks for it
    private static final long serialVersionUID = -1707639091393431922L;

    /**
     * Constructs an empty PixyObjectList.
     */
    public PixyObjectList() {
        super();
    }

    /**
     * Constructs an empty PixyObjectList.
     */
    public PixyObjectList(ArrayList<PixyBlock> blocks, int signature, int maxTTL) {
        super();

        for (PixyBlock pixyBlock : blocks) {
            if (pixyBlock.signature == signature) {
                this.add(new PixyObject(pixyBlock, maxTTL));
            }
        }
    }

    public PixyObjectList(String pixyString, int signature, int maxTTL) {
        /*
         * pixyString looks like: "2 1,114,51,228,102,149;1,136,175,172,64,109;" The
         * string starts with the number of blocks followed by space. Blocks are
         * separated with ";"
         */
        super();
        String[] split1 = new String[2];
        String[] blocks;
        int numBlocks = 0;
        // Splits the string to the number of blocks and to the blocks
        split1 = pixyString.split(" ");
        try {
            numBlocks = Integer.parseInt(split1[0]);
            blocks = new String[numBlocks];
            blocks = split1[1].split(";");
            for (int i = 0; i < numBlocks; i++) {
                PixyObject pixyObject = new PixyObject(blocks[i], maxTTL);
                if (pixyObject.getSignature() == signature) {
                    this.add(pixyObject);
                }
            }
        } catch (Exception e) {

        }
    }

    public void invertY(int screenHeight) {
        for (PixyObject pixyObject : this) {
            pixyObject.setY(screenHeight - pixyObject.getY());
        }
    }

    @Override
    public PixyObjectList clone() {
        PixyObjectList clonedList = new PixyObjectList();

        clonedList.addAll(this);

        return clonedList;
    }

    /**
     * Returns a PixyObject for the biggest object in this PixyObjectList.
     * 
     * @return A PixyObject. A zeroed PixyObject will be returned if no such object
     *         is detected.
     */
    public PixyObject getBiggest() {
        if (this.size() > 0) {
            PixyObject biggestObject = get(0);
            for (PixyObject pixyObject : this) {
                if (pixyObject.getArea() > biggestObject.getArea()) {
                    biggestObject = pixyObject;
                }
            }
            return biggestObject;
        } else {
            return null;
        }
    }

    static int count = 0;

    /**
     * Returns a PixyObject for the leftmost object in this PixyObjectList.
     * 
     * @return A PixyObject. A zeroed PixyObject will be returned if no such object
     *         is detected.
     */
    public PixyObject getLeftmost() {

        try {
            PixyObject leftmostObject = get(0);
            for (PixyObject pixyObject : this) {
                if (pixyObject.getX() < leftmostObject.getX()) {
                    leftmostObject = pixyObject;
                }
            }
            return leftmostObject;
        } catch (Exception e) {
            // System.out.println("No object found");
            return new PixyObject();
        }
    }

    /**
     * Returns a PixyObject for the rightmost object in this PixyObjectList.
     * 
     * @return A PixyObject. A zeroed PixyObject will be returned if no such object
     *         is detected.
     */
    public PixyObject getRightmost() {
        try {
            PixyObject rightmostObject = get(0);
            for (PixyObject pixyObject : this) {
                if (pixyObject.getX() > rightmostObject.getX()) {
                    rightmostObject = pixyObject;
                }
            }
            return rightmostObject;
        } catch (Exception e) {
            // System.out.println("No object found");
            return new PixyObject();
        }
    }

    @Override
    public String toString() {
        String result = "";
        for (int i = 0; i < this.size(); i++) {
            result += this.get(i).toString() + "\n";
        }
        return result;
    }
}
