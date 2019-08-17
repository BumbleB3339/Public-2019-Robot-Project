package frc.robot.profiles;

import frc.bumblelib.bumblelib_autonomous.pathing.field_dimension.SingularFieldDimension;
import frc.bumblelib.bumblelib_autonomous.pathing.field_dimension.SymmetricFieldDimension;
import frc.bumblelib.util.Units;

public abstract class FieldProfile {

    protected static final double DEFAULT_FIELD_WIDTH = Units.feet_to_meters(27.0);
    
    /**
     * 
     */
    public abstract double getFieldWidth();

    /**
     * Isn't measurment.
     */
    public double absoluteRocketAngle() {
        return 61.0;
    }

    /**
     * Isn't measurment.
     */
    public double closeRocketAngle() {
        return 29.0;
    }

    /**
     * Isn't measurment.
     */
    public double farRocketAngle() {
        return 151.0;
    }

    /**
     * 
     */
    public abstract SymmetricFieldDimension sideWallToLevel1FlatEdge();

    /**
     * 
     */
    public abstract SymmetricFieldDimension sideWallToRocket();

    /**
     * 
     */
    public abstract SymmetricFieldDimension allianceWallToCloseRocketCorner();
   
    /**
     * 
     */
    public abstract SymmetricFieldDimension sideWallToMidFeeder();
    
    /**
     * 
     */
    public abstract SingularFieldDimension level1FlatEdgeToFrontCargoShipFace();

    /**
     * 
     */
    public abstract SymmetricFieldDimension sideWallToCargoshipSide();
    
    /**
     * Isn't measurment.
     */
    public SingularFieldDimension rocketDepth(){
        SingularFieldDimension sfd = new SingularFieldDimension();
        sfd.setBlueSize(0.47);
        sfd.setRedSize(0.47);
        return sfd;     
    }
    
    /**
     * Isn't measurment.
     */
    public SingularFieldDimension rocketLength(){
        SingularFieldDimension sfd = new SingularFieldDimension();
        sfd.setBlueSize(1.00);
        sfd.setRedSize(1.00);
        return sfd;    
    }

    /**
     * Isn't measurment.
     */
    public SingularFieldDimension level2Depth(){
        SingularFieldDimension sfd = new SingularFieldDimension();
        sfd.setBlueSize(1.22);
        sfd.setRedSize(1.22);
        return sfd;    
    }

    /**
     * Isn't measurment.
     */
    public SingularFieldDimension allianceWallToLevel1FlatEdge(){
        SingularFieldDimension sfd = new SingularFieldDimension();
        sfd.setBlueSize(2.14);
        sfd.setRedSize(2.14);
        return sfd;    
    }

    /**
     * Isn't measurment.
     */
    public SingularFieldDimension cargoShipSideToFrontHatch(){ // TODO: find out actual value
        SingularFieldDimension sfd = new SingularFieldDimension();
        sfd.setBlueSize(0.45);
        sfd.setRedSize(0.45);
        return sfd;      
    }

    /**
     * Isn't measurment.
     */
    public SingularFieldDimension frontShipFaceToCloseHatchPlacement(){
        SingularFieldDimension sfd = new SingularFieldDimension();
        sfd.setBlueSize(1.03);
        sfd.setRedSize(1.03);
        return sfd;    
    }

    /**
     * Isn't measurment.
     */
    public SingularFieldDimension frontShipFaceToMidHatchPlacement(){
        SingularFieldDimension sfd = new SingularFieldDimension();
        sfd.setBlueSize(1.58);
        sfd.setRedSize(1.58);
        return sfd;    
    }

    /**
     * Isn't measurment.
     */
    public SingularFieldDimension frontShipFaceToFarHatchPlacement(){
        SingularFieldDimension sfd = new SingularFieldDimension();
        sfd.setBlueSize(2.13);
        sfd.setRedSize(2.13);
        return sfd;      
    }

    /**
     * Isn't measurment.
     */
    public SingularFieldDimension level2Width(){
        SingularFieldDimension sfd = new SingularFieldDimension();
        sfd.setBlueSize(1.02);
        sfd.setRedSize(1.02);
        return sfd;      
    }

    /**
     * Isn't measurment.
     */
    public SymmetricFieldDimension sideWallToLevel3Edge() {
        SymmetricFieldDimension sfd = new SymmetricFieldDimension();
        sfd.addFieldDimension(level2Width());
        sfd.addFieldDimension(sideWallToLevel1FlatEdge());
        return sfd;      
    }    
}
