/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.bumblelib.vision;

/**
 * This class describes the physical target. All measurements are in Meters.
 */
public class PhysicalTarget {
    
    private double width; 
    private double height;
    private double widthToHeightRatio;
    private double measuredWidthToHeightRatio;

    public PhysicalTarget (double width, double height){
        this(width, height, width/height);
    }

    public PhysicalTarget (double width, double height, double measuredWidthToHeightRatio){
        this.width = width;
        this.height = height;
        this.measuredWidthToHeightRatio = measuredWidthToHeightRatio;

        widthToHeightRatio = width / height;
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
     * @return width to height ratio
     */
    public double getWidthToHeightRatio() {
        return widthToHeightRatio;
    }

    /**
     * @return the measuredWidthToHeightRatio
     */
    public double getMeasuredWidthToHeightRatio() {
        return measuredWidthToHeightRatio;
    }
}
