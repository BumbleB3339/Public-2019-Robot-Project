package frc.bumblelib.util.hardware.pixy;

public class PixyTarget {

    private double x, y, height, width;
    private boolean isSingleReflective;

    /**
     * @param x
     * @param y
     * @param height
     * @param width
     */
    public PixyTarget(double x, double y, double width, double height) {
        this.x = x;
        this.y = y;
        this.width = width;
        this.height = height;
        this.isSingleReflective = false;
    }

    /**
     * Construct a PixyObject.
     * 
     * @param pixyBlock the original PixyPacket to base this object on.
     */
    public PixyTarget(PixyBlock pixyBlock) {
        this(pixyBlock.x, pixyBlock.y, pixyBlock.height, pixyBlock.width);
    }

    public PixyTarget(PixyObject pixyObject) {
        this(pixyObject.getX(), pixyObject.getY(), pixyObject.getHeight(), pixyObject.getWidth());
    }

    /**
     * @return the area
     */
    public double getArea() {
        return width * height;
    }

    /**
     * @return the height
     */
    public double getHeight() {
        return height;
    }

    /**
     * @return the width
     */
    public double getWidth() {
        return width;
    }

    /**
     * x cooridinate is counted from the left side of the Pixy view.
     * 
     * @return the x
     */
    public double getX() {
        return x;
    }

    /**
     * y cooridinate is counted from the bottom side of the Pixy view.
     * 
     * @return the y
     */
    public double getY() {
        return y;
    }

    /**
     * @param isSingleReflective the isSingleReflective to set
     */
    public void setSingleReflective(boolean isSingleReflective) {
        this.isSingleReflective = isSingleReflective;
    }

    public boolean getIsSingleReflective(){
        return isSingleReflective;
    }


    @Override
    public String toString() {
        return "X: " + getX() + " Y: " + getY();
    }
}
