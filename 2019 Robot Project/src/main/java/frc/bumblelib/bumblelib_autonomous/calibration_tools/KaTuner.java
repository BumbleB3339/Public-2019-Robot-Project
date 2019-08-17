package frc.bumblelib.bumblelib_autonomous.calibration_tools;

import static frc.robot.Robot.m_drivetrain;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Command;
import frc.bumblelib.util.SimpleCSVLogger;
import frc.robot.profiles.InitProfiles;
import frc.robot.profiles.RobotProfile.PathfinderParams.GearParams;

/**
 * This class generates a CSV file that can be used to derive the value of the
 * acceleration coefficiant. It does this by sending a constant applied voltage
 * to drivetrain motors and calculating the percent voltage that goes towards
 * acceleration: Vacc = Vapp - kV * velocity - Vintercept and graphs that as a
 * function of the actual acceleration. The slope of this graph is kA. Vacc = kA
 * * acceleration
 */
public class KaTuner extends Command {

    
    private final double VOLTAGE_PERCENT = 0.7; // The percent voltage to be sent to the motors during the test. (%)
    private final double MAX_DISTANCE = 5.0; // The maximum distance the robot is allowed to travel. (meters)
    private boolean isRotation;
    
    private double prevTime, prevVelocity, initDistance, distance;
    private SimpleCSVLogger logger;
    
    private GearParams gearParams;
    
    public KaTuner(boolean isRotation) {
        requires(m_drivetrain);

        this.isRotation = isRotation;
        logger = new SimpleCSVLogger("KaTuningLog");
        logger.init(new String[] { "acc", "acc_volt_percent" }, new String[] { "m/s/s", "%" });
    }

    @Override
    protected void initialize() {
        gearParams = InitProfiles.ROBOT_PROFILE.pathfinderParams.getGearParams(m_drivetrain.getGear());

        distance = 0;
        initDistance = getAverageDistance();
        prevTime = Timer.getFPGATimestamp();
        prevVelocity = getAverageVelocity();
    }

    @Override
    protected void execute() {
        double currentVelocity = getAverageVelocity();
        double currentTime = Timer.getFPGATimestamp();
        double currentAcc = (currentVelocity - prevVelocity) / (currentTime - prevTime);
        distance = getAverageDistance() - initDistance;

        if (isRotation) {
            double accVPercent = VOLTAGE_PERCENT - gearParams.rotVIntercept - gearParams.rotKv * currentVelocity;
            logger.writeData(currentAcc, accVPercent);

            m_drivetrain.bumbleDrive.setLeftRightMotorOutputs(-VOLTAGE_PERCENT, VOLTAGE_PERCENT, false);
        } else {
            double accVPercent = VOLTAGE_PERCENT - gearParams.vIntercept - gearParams.kV * currentVelocity;
            logger.writeData(currentAcc, accVPercent);

            m_drivetrain.bumbleDrive.setLeftRightMotorOutputs(VOLTAGE_PERCENT, VOLTAGE_PERCENT, false);
        }

        // Update vars.
        prevVelocity = currentVelocity;
        prevTime = currentTime;
    }

    private double getAverageDistance() {
        return (Math.abs(m_drivetrain.getLeftDistance()) + Math.abs(m_drivetrain.getRightDistance())) / 2.0;
    }
    
    private double getAverageVelocity() {
        return (Math.abs(m_drivetrain.getLeftVelocity()) + Math.abs(m_drivetrain.getRightVelocity())) / 2.0;
    }

    @Override
    protected boolean isFinished() {
        return distance >= MAX_DISTANCE;
    }

    @Override
    protected void end() {
        logger.close();
        m_drivetrain.stopMotors();
    }

    @Override
    protected void interrupted() {
        end();
    }
}