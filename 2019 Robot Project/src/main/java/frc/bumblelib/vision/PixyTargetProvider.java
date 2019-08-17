/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.bumblelib.vision;

import java.util.ArrayList;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.bumblelib.bumblelib_autonomous.pathing.RobotReferencePoint;
import frc.bumblelib.util.KalmanFilter;
import frc.bumblelib.util.Logger;
import frc.bumblelib.util.SmartdashboardDebugging;
import frc.bumblelib.util.hardware.pixy.BumblePixy;
import frc.bumblelib.util.hardware.pixy.PixyObject;
import frc.bumblelib.util.hardware.pixy.PixyObjectList;
import frc.bumblelib.util.hardware.pixy.PixyTarget;
import frc.bumblelib.util.hardware.pixy.PixyTargetList;
import frc.robot.vision.MergeTargets;

/**
 * An implementation of VisualTargetProvider for Pixy. Returns all Pixy data for
 * a specific target. This class also handles data buffering and smoothing of
 * the final outputs.
 */
public class PixyTargetProvider extends VisualTargetProvider implements SmartdashboardDebugging, Logger {
    // BIGGEST is the default policy
    private TargetSelectionPolicy targetSelectionPolicy = TargetSelectionPolicy.BIGGEST;
    private BumblePixy pixy;
    private final int TARGET_MARGIN = 5; // Require 5 pixels at list from each side of the target.
    private RobotReferencePoint pixyRobotReferencePoint;
    private PixyTarget pixyTarget = null;
    private PixyObjectList pixyObjectList;
    private MergeTargets mergeTargets = new MergeTargets();
    private double pixyAngle = 0;

    // Kalman filter
    private final int PIXY_KALMAN_WINDOW_SIZE = 3;
    private KalmanFilter targetX = new KalmanFilter(PIXY_KALMAN_WINDOW_SIZE);
    private KalmanFilter targetY = new KalmanFilter(PIXY_KALMAN_WINDOW_SIZE);
    private KalmanFilter targetWidth = new KalmanFilter(PIXY_KALMAN_WINDOW_SIZE);
    private KalmanFilter targetHeight = new KalmanFilter(PIXY_KALMAN_WINDOW_SIZE);
    private boolean isTargetSingleReflective;

    // private double targetX, targetY, targetWidth, targetHeight;
    private boolean lastHasTargetInView = false, hasTargetInView = false;

    /**
     * 
     * @param pixy                    - The specific Pixy sensor.
     * @param signature               - A specific target type.
     * @param pixyRobotReferencePoint - Location of the Pixy on the robot.
     */
    public PixyTargetProvider(BumblePixy pixy, RobotReferencePoint pixyRobotReferencePoint, double sensorAngle) {
        this.pixy = pixy;
        this.pixyRobotReferencePoint = pixyRobotReferencePoint;
        this.pixyAngle = sensorAngle;
    }

    /**
     * Add new target data to the different filters to be considered in the
     * smoothing process.
     */
    private void updatePixyTargetProperties() {
        if (pixyTarget == null) {
            hasTargetInView = false;

            if (hasTargetInView != lastHasTargetInView) {
                resetKalmanFilters();
            }
        } else {
            targetX.addValue(pixyTarget.getX());
            targetY.addValue(pixyTarget.getY());
            targetHeight.addValue(pixyTarget.getHeight());
            targetWidth.addValue(pixyTarget.getWidth());
            isTargetSingleReflective = pixyTarget.getIsSingleReflective();

            hasTargetInView = true;
        }

        lastHasTargetInView = hasTargetInView;
    }

    private void resetKalmanFilters() {
        targetX.reset();
        targetY.reset();
        targetHeight.reset();
        targetWidth.reset();
    }

    /**
     * @return the hasTargetInView
     */
    public boolean hasTargetInView() {
        return hasTargetInView;
    }

    // The following four functions return the smooth values of the target
    // properites.
    @Override
    public double getTargetX() {
        return targetX.getSmoothValue();
        // return targetX;
    }

    @Override
    public double getTargetY() {
        return targetY.getSmoothValue();
        // return targetY;
    }

    @Override
    public double getTargetWidth() {
        return targetWidth.getSmoothValue();
        // return targetWidth;
    }

    @Override
    public double getTargetHeight() {
        return targetHeight.getSmoothValue();
        // return targetHeight;
    }



    @Override
    public double getHFOV() {
        return BumblePixy.PIXY1_HFOV;
    }

    @Override
    public double getVFOV() {
        return BumblePixy.PIXY1_VFOV;
    }

    @Override
    public int getScreenHeight() {
        return BumblePixy.PIXY1_FRAME_HEIGHT;
    }

    @Override
    public int getScreenWidth() {
        return BumblePixy.PIXY1_FRAME_WIDTH;
    }

    @Override
    public boolean isTargetDetected() {
        return isTargetFullyDetected();
    }

    /**
     * @return Returns if the target is fully in the frame by checking whether there
     *         are minimal Margings around it.
     */
    private boolean isTargetFullyDetected() {
        return ((getTargetX() - (getTargetWidth() / 2) > TARGET_MARGIN)
                && (getTargetX() + (getTargetWidth() / 2) < getScreenWidth() - TARGET_MARGIN)
                && (getTargetY() - (getTargetHeight() / 2) > TARGET_MARGIN)
                && (getTargetY() + (getTargetHeight() / 2) < getScreenHeight() - TARGET_MARGIN));
    }

    @Override
    public RobotReferencePoint getSensorRobotReferncePoint() {
        return pixyRobotReferencePoint;
    }

    @Override
    public void setTargetSelectionPolicy(TargetSelectionPolicy targetSelectionPolicy) {
        this.targetSelectionPolicy = targetSelectionPolicy;
    }

    @Override
    public TargetSelectionPolicy getTargetSelectionPolicy() {
        return targetSelectionPolicy;
    }

    /**
     * This function is called periodically and stores the current target properties
     * in local variables to be used by other functions
     */
    @Override
    public void updateImage() {
        pixyObjectList = pixy.getObjects();

        pixyObjectList.invertY(getScreenHeight());

        PixyTargetList mergedTargetsList = mergeTargets.getMergedTargets(pixyObjectList.clone());

        switch (targetSelectionPolicy) {
        case HIGHEST:
            pixyTarget = mergedTargetsList.getHighest();
            break;
        case LEFTMOST:
            pixyTarget = mergedTargetsList.getLeftmost();
            break;
        case RIGHTMOST:
            pixyTarget = mergedTargetsList.getRightmost();
            break;
        case CENTERMOST:
            pixyTarget = mergedTargetsList.getCenter(getScreenWidth() / 2);
        case BIGGEST:
        default:
            pixyTarget = mergedTargetsList.getBiggest();
            break;
        }
        
        updatePixyTargetProperties();
    }

    @Override
    public double getSensorAngle() {
        return pixyAngle;
    }

    public void setSensorAngle(double pixyAngle) {
        this.pixyAngle = pixyAngle;
    }

    @Override
    public ArrayList<String> getLogHeaders() {
        ArrayList<String> output = new ArrayList<String>();

        output.add("targetX");
        output.add("targetY");
        output.add("targetWidth");
        output.add("targetHeight");
        output.add("targetRatio");
        output.add("hasTargetInView");
        output.add("targetSelectionPolicy");

        for (int i = 1; i <= 6; i++) {
            output.add("X" + i);
            output.add("Y" + i);
            output.add("Width" + i);
            output.add("Height" + i);
        }

        return output;

    }

    @Override
    public ArrayList<String> getLogHeaderUnits() {
        ArrayList<String> output = new ArrayList<String>();

        output.add("Pixels");
        output.add("Pixels");
        output.add("Pixels");
        output.add("Pixels");
        output.add("Number");
        output.add("Boolean");
        output.add("Policy");

        for (int i = 1; i <= 4 * 6; i++) {
            output.add("Pixels");
        }

        return output;
    }

    @Override
    public ArrayList<String> getLogData() {
        ArrayList<String> output = new ArrayList<String>();
        output.add(Double.toString(getTargetX()));
        output.add(Double.toString(getTargetY()));
        output.add(Double.toString(getTargetWidth()));
        output.add(Double.toString(getTargetHeight()));
        output.add(Double.toString(getWidthToHeightRatio()));
        output.add(hasTargetInView() ? "1" : "0");
        output.add(getTargetSelectionPolicy().toString());

        if (pixyObjectList != null) {
            int arrSize = pixyObjectList.size();
            PixyObject pixyObject;
            for (int i = 0; i < arrSize; i++) {
                pixyObject = pixyObjectList.get(i);
                output.add(Double.toString(pixyObject.getX()));
                output.add(Double.toString(pixyObject.getY()));
                output.add(Double.toString(pixyObject.getWidth()));
                output.add(Double.toString(pixyObject.getHeight()));
            }
        }

        return output;
    }

    @Override
    public void sendDebuggingData() {
        SmartDashboard.putNumber("PixyTargetProvider/x", getTargetX());
        SmartDashboard.putNumber("PixyTargetProvider/y", getTargetY());
        SmartDashboard.putNumber("PixyTargetProvider/width", getTargetWidth());
        SmartDashboard.putNumber("PixyTargetProvider/height", getTargetHeight());
        SmartDashboard.putNumber("PixyTargetProvider/ratio", getWidthToHeightRatio());
        SmartDashboard.putBoolean("PixyTargetProvider/hasTargetInView", hasTargetInView);
        SmartDashboard.putString("PixyTargetProvider/targetSelectionPolicy", targetSelectionPolicy.toString());

        double[] xArr, yArr, widthArr, heightArr, indexArr;
        if (pixyObjectList != null) {
            int arrSize = pixyObjectList.size();
            xArr = new double[arrSize];
            yArr = new double[arrSize];
            widthArr = new double[arrSize];
            heightArr = new double[arrSize];
            indexArr = new double[arrSize];

            PixyObject pixyObject;
            for (int i = 0; i < arrSize; i++) {
                pixyObject = pixyObjectList.get(i);

                xArr[i] = pixyObject.getX();
                yArr[i] = pixyObject.getY();
                widthArr[i] = pixyObject.getWidth();
                heightArr[i] = pixyObject.getHeight();
                indexArr[i] = pixyObject.getIndex();
            }
        } else {
            xArr = new double[0];
            yArr = new double[0];
            widthArr = new double[0];
            heightArr = new double[0];
            indexArr = new double[0];
        }

        SmartDashboard.putNumberArray("pixyObjectList/x", xArr);
        SmartDashboard.putNumberArray("pixyObjectList/y", yArr);
        SmartDashboard.putNumberArray("pixyObjectList/width", widthArr);
        SmartDashboard.putNumberArray("pixyObjectList/height", heightArr);
        SmartDashboard.putNumberArray("pixyObjectList/index", indexArr);
    }

    @Override
    public boolean isTargetSingleReflective() {
        return isTargetSingleReflective;
    }
}
