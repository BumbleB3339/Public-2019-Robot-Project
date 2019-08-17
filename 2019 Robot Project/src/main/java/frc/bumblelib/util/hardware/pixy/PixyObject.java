package frc.bumblelib.util.hardware.pixy;

public class PixyObject implements Comparable<PixyObject> {

    private int signature, x, y, height, width, index;
    private int maxTTL;
    private int ttl; // Time to Live, how many iterations of not receiving new data until it is
                     // zeroed

    /**
     * Construct a PixyObject.
     * 
     * @param signature  the signature number of the object.
     * @param pixyBlock the original PixyPacket to base this object on.
     * @param maxTTL     iterations of now receiving data before this object it
     *                   removed (time to live).
     */

    public PixyObject(PixyBlock pixyBlock, int maxTTL) {
        this.maxTTL = maxTTL;
        updateData(pixyBlock);
    }

    public PixyObject(String pixyBlockString, int maxTTL){
        /*pixyBlockString looks like: "1,114,51,228,102,149" where the order of fields is:
          signature, x, y, width, height, index 
        */

        String[] strValues = new String[6];
        strValues = pixyBlockString.split(",");
        if(pixyBlockString.split(",").length == 6){
        this.signature =Integer.parseInt(strValues[0]);
        this.x =Integer.parseInt(strValues[1]);
        this.y =Integer.parseInt(strValues[2]);
        this.width =Integer.parseInt(strValues[3]);
        this.height =Integer.parseInt(strValues[4]);
        this.index =Integer.parseInt(strValues[5]);
        }
    }
    

    /**
     * Update data in this PixyObject and reset ttl.
     * 
     * @param pixyBlock
     */
    public void updateData(PixyBlock pixyBlock) {
        this.signature = pixyBlock.signature;
        this.x = pixyBlock.x;
        this.y = pixyBlock.y;
        this.height = pixyBlock.height;
        this.width = pixyBlock.width;
        this.ttl = this.maxTTL;
        this.index = -3339;
    }

    /**
     * Update data when no new info is received from Pixy. this will just lower the
     * timeToLive by one.
     */
    public void updateData() {
        this.ttl--;
    }

    /**
     * Construct a zeroed PixyObject (all params set to 0). This is an empty object.
     */
    public PixyObject() {
        this.signature = 0;
        this.x = 0;
        this.y = 0;
        this.height = 0;
        this.width = 0;
    }

    @Override
	public int compareTo(PixyObject arg0) {
		return (int) Math.signum(x - arg0.x);
	}

    /**
     * @return the area
     */
    public int getArea() {
        return width * height;
    }

    /**
     * @return the height
     */
    public int getHeight() {
        return height;
    }

    /**
     * @return the signature
     */
    public int getSignature() {
        return signature;
    }

    /**
     * @return the width
     */
    public int getWidth() {
        return width;
    }

    /**
     * x cooridinate is counted from the left side of the Pixy view.
     * 
     * @return the x
     */
    public int getX() {
        return x;
    }

    /**
     * y cooridinate is counted from the bottom side of the Pixy view.
     * 
     * @return the y
     */
    public int getY() {
        return y;
    }

    /**
     * @return the value of ttl (time to live)
     */
    public int getTtl() {
        return ttl;
    }

    /**
     * @return the index
     */
    public int getIndex() {
        return index;
    }
    
    /**
     * @param x the x to set
     */
    public void setX(int x) {
        this.x = x;
    }

    /**
     * @param y the y to set
     */
    public void setY(int y) {
        this.y = y;
    }

    /**
     * @param height the height to set
     */
    public void setHeight(int height) {
        this.height = height;
    }

    /**
     * @param width the width to set
     */
    public void setWidth(int width) {
        this.width = width;
    }

    @Override
    public String toString() {
        return "Signature: " + getSignature() + " X: " + getX() 
        + " Y: " + getY() + " Width: " + getWidth() + " Height: " + getHeight() + " Index: " + getIndex();
    }
}
