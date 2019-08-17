package frc.bumblelib.bumblelib_autonomous.pathing;

public class PathPointAdjustment {
    private double dX;
    private double dY;
    private double dTheta;

    public PathPointAdjustment() {
        this.dX = 0.0;
        this.dY = 0.0;
        this.dTheta = 0.0;
    }

    public void addAdjustment(double dX, double dY, double dTheta) {
        this.dX += dX;
        this.dY += dY;
        this.dTheta += dTheta;
    }

    /**
     * @return the dX
     */
    public double getdX() {
        return dX;
    }

    /**
     * @return the dY
     */
    public double getdY() {
        return dY;
    }

    /**
     * @return the dTheta
     */
    public double getdTheta() {
        return dTheta;
    }
}
