package frc.bumblelib.util.hardware;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.SPI.Port;


public class NavX extends Gyro {
    private AHRS navx;
    private double navxStopRate;
    private double rollRecalibration = 0;
    private double pitchRecalibration = 0;

    public NavX() {
        navx = new AHRS(Port.kMXP);
    }

    public boolean isConnected() {
        return navx.isConnected();
    }

    @Override
    public double getRawYaw() {
        return -navx.getYaw();
    }

    public void setNavxStopRate(double navxStopRate) {
        this.navxStopRate = navxStopRate;
    }

    public boolean isRotating() {
        return navx.getRate() > navxStopRate;
    }

    @Override
    public void reset() {
        navx.reset();
    }

    public boolean isMoving() {
        return navx.isMoving();
    }

    //Roll and Pitch are reversed beacuse of the placement of the RoboRIO
    public double getRoll() {
        return navx.getPitch() - rollRecalibration;
    }

    //Roll and Pitch are reversed beacuse of the placement of the RoboRIO
    public double getPitch() {
        return navx.getRoll() - pitchRecalibration;
    }

    //Roll and Pitch are reversed beacuse of the placement of the RoboRIO
    public void resetRoll() {
        rollRecalibration = navx.getPitch();
    }

    //Roll and Pitch are reversed beacuse of the placement of the RoboRIO
    public void resetPitch() {
        pitchRecalibration = navx.getRoll();
    }

    public void reCalibrateAxis(){
        resetPitch();
        resetRoll();
        navx.zeroYaw();
    }
}
