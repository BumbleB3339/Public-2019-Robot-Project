/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.bumblelib.util;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.PIDSource;
import edu.wpi.first.wpilibj.PIDSourceType;

/**
 * Add your docs here.
 */
public class LambdaPIDSource implements PIDSource {

    private PIDSourceType pidSource = PIDSourceType.kDisplacement;
    private DoubleSupplier doubleSupplier;

    public LambdaPIDSource(DoubleSupplier doubleSupplier) {
        this.doubleSupplier = doubleSupplier;
    }

    @Override
    public void setPIDSourceType(PIDSourceType pidSource) {
        this.pidSource = pidSource;
    }

    @Override
    public PIDSourceType getPIDSourceType() {
        return pidSource;
    }

    @Override
    public double pidGet() {
        return doubleSupplier.getAsDouble();
    }
}
