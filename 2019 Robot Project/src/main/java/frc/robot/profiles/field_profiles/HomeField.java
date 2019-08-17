package frc.robot.profiles.field_profiles;

import frc.bumblelib.bumblelib_autonomous.pathing.field_dimension.SingularFieldDimension;
import frc.bumblelib.bumblelib_autonomous.pathing.field_dimension.SymmetricFieldDimension;
import frc.robot.profiles.FieldProfile;

public class HomeField extends FieldProfile {

    @Override
    public double getFieldWidth() {
        return 8.23;
    }

    @Override
    public double absoluteRocketAngle() {
        return 61.0;
    }

    @Override
    public SymmetricFieldDimension sideWallToLevel1FlatEdge() {
        SymmetricFieldDimension sfd = new SymmetricFieldDimension();
        sfd.setBlueLeftSize(2.51);
        sfd.setBlueRightSize(2.48);
        sfd.setRedLeftSize(2.51);
        sfd.setRedRightSize(2.48);
        return sfd;  
    }

    @Override
    public SymmetricFieldDimension sideWallToRocket() {
        SymmetricFieldDimension sfd = new SymmetricFieldDimension();
        sfd.setBlueLeftSize(0.191);
        sfd.setBlueRightSize(0.189);
        sfd.setRedLeftSize(0.191);
        sfd.setRedRightSize(0.189);
        return sfd;  
    }

    @Override
    public SymmetricFieldDimension allianceWallToCloseRocketCorner() {
        SymmetricFieldDimension sfd = new SymmetricFieldDimension();
        sfd.setBlueLeftSize(5.25);
        sfd.setBlueRightSize(5.25);
        sfd.setRedLeftSize(5.25);
        sfd.setRedRightSize(5.25);
        return sfd;  
    }

    @Override
    public SymmetricFieldDimension sideWallToMidFeeder() {
        SymmetricFieldDimension sfd = new SymmetricFieldDimension();
        sfd.setBlueLeftSize(0.64);
        sfd.setBlueRightSize(0.66);
        sfd.setRedLeftSize(0.64);
        sfd.setRedRightSize(0.66);
        return sfd;  
    }

    @Override
    public SingularFieldDimension level1FlatEdgeToFrontCargoShipFace() {
        SingularFieldDimension sfd = new SingularFieldDimension();
        sfd.setBlueSize(3.25);
        sfd.setRedSize(3.25);
        return sfd; 
    }

    @Override
    public SymmetricFieldDimension sideWallToCargoshipSide() {
        SymmetricFieldDimension sfd = new SymmetricFieldDimension();
        sfd.setBlueLeftSize(3.41);
        sfd.setBlueRightSize(3.41);
        sfd.setRedLeftSize(3.41);
        sfd.setRedRightSize(3.41);
        return sfd;
    }

    @Override
    public SingularFieldDimension rocketDepth() {
        SingularFieldDimension sfd = new SingularFieldDimension();
        sfd.setBlueSize(0.50);
        sfd.setRedSize(0.50);
        return sfd; 
    }

    @Override
    public SingularFieldDimension rocketLength() {
        SingularFieldDimension sfd = new SingularFieldDimension();
        sfd.setBlueSize(1.0);
        sfd.setRedSize(1.0);
        return sfd; 
    }

    @Override
    public SingularFieldDimension level2Depth() {
        SingularFieldDimension sfd = new SingularFieldDimension();
        sfd.setBlueSize(1.22);
        sfd.setRedSize(1.22);
        return sfd; 
    }

    @Override
    public SingularFieldDimension allianceWallToLevel1FlatEdge() {
        SingularFieldDimension sfd = new SingularFieldDimension();
        sfd.setBlueSize(2.14);
        sfd.setRedSize(2.14);
        return sfd; 
    }

    @Override
    public SingularFieldDimension cargoShipSideToFrontHatch() {
        SingularFieldDimension sfd = new SingularFieldDimension();
        sfd.setBlueSize(0.45);
        sfd.setRedSize(0.45);
        return sfd; 
    }

    @Override
    public SingularFieldDimension frontShipFaceToCloseHatchPlacement() {
        SingularFieldDimension sfd = new SingularFieldDimension();
        sfd.setBlueSize(1.0);
        sfd.setRedSize(1.0);
        return sfd; 
    }

    @Override
    public SingularFieldDimension frontShipFaceToMidHatchPlacement() {
        SingularFieldDimension sfd = new SingularFieldDimension();
        sfd.setBlueSize(1.6);
        sfd.setRedSize(1.6);
        return sfd; 
    }

    @Override
    public SingularFieldDimension frontShipFaceToFarHatchPlacement() {
        SingularFieldDimension sfd = new SingularFieldDimension();
        sfd.setBlueSize(2.2);
        sfd.setRedSize(2.2);
        return sfd; 
    }

    @Override
    public double closeRocketAngle() {
        return 29;
    }

    @Override
    public double farRocketAngle() {
        return 151;
    }
}