/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.bumblelib.vision;

public class VisionUtils {
    private final static double kD = 0.872463643 * 0.999982553; // Distance calculation gets multiplied by this number

    /**
     * The method gets xCenter of target, fov of camera in degrees and image width
     * of camera in pixels and convert the xCenter of the target to degree error
     * from the target
     * 
     * @param xCenter    the pixel in the center of the target that you want to
     *                   convert to degrees
     * @param fov        the field of view of the camera
     * @param imageWidth the width of the image in pixels
     **/
    public static double convertPixelsToAngles(double xCenter, int fov, int imageWidth) {
        double pixelsInDegree = imageWidth / fov;
        return (xCenter - (imageWidth / 2)) / pixelsInDegree; // Degrees to the target
    }

    public static double widthOfTargetIn0Degrees(double realwidth, double distance, double pixyHorizentalFieldOfView,
            int screenWidth) {
        return (realwidth * screenWidth) / (2 * distance * Math.tan(Math.toRadians(pixyHorizentalFieldOfView / 2)));
    }

    public static double findDistance(double realHeight, int pixelsHeight, int screenHeight,
            double pixyVerticalFieldOfView) {
        return kD * ((realHeight * screenHeight)
                / (2 * pixelsHeight * Math.tan(Math.toRadians(pixyVerticalFieldOfView / 2))));
    }

    public static double getAngleShiftFromTarget(double widthOfTargetIn0Degrees, double pixelsWidth) {
        return Math.toDegrees(Math.acos((pixelsWidth / widthOfTargetIn0Degrees)));
    }

    public static double calculateDistanceToDrive(double distance, double distanceToStopBeforeTarget,
            double angleToTarget) {
        double x2 = Math.pow(distance, 2) + Math.pow(distanceToStopBeforeTarget, 2)
                - 2 * distanceToStopBeforeTarget * distance * Math.cos(Math.toRadians(angleToTarget));
        return Math.sqrt(x2);
    }

    public static double angleToSpin(double distance, double distanceToStopBeforeTarget, double angleToTarget,
            double xCenter, int fov, int imageWidth) {
        double angleToSpinbBeforeCorrection, angleToSpinbAfterCorrection;
        angleToSpinbBeforeCorrection = Math
                .toDegrees(Math.acos((distanceToStopBeforeTarget * Math.sin(Math.toRadians(angleToTarget))
                        / calculateDistanceToDrive(distance, distanceToStopBeforeTarget, angleToTarget))));
        angleToSpinbAfterCorrection = angleToSpinbBeforeCorrection + convertPixelsToAngles(xCenter, fov, imageWidth);
        return angleToSpinbAfterCorrection;
    }

}
