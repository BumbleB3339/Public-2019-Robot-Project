package frc.bumblelib.util.hardware;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.command.Subsystem;

public class BumbleController extends XboxController {

	private ControllerSubsystem controllerSubsystem;

	/**
   	 * Construct an instance of a joystick. The joystick index is the USB port on the drivers
     * station.
     *
     * @param port The port on the Driver Station that the joystick is plugged into.
     */
	public BumbleController(int port) {
		super(port);
		this.controllerSubsystem = new ControllerSubsystem();
	}

	/**
     * Get the Y axis value of the controller.
     *
     * @param hand Side of controller whose value should be returned.
     * @return The Y axis value of the controller (adjusted so that 1 is up and -1 is down).
     */
	@Override
	public double getY(Hand hand) {
		return -super.getY(hand);
	}

	/**
   	 * Read the value of the POV_UP button on the controller.
   	 *
   	 * @return The state of the button.
   	 */
	public boolean getPOV_Up() {
		int POV = super.getPOV(0);
		return POV > 270 || (POV >= 0 && POV < 90);
	}

	/**
   	 * Read the value of the POV_DOWN button on the controller.
   	 *
   	 * @return The state of the button.
   	 */
	public boolean getPOV_Down() {
		int POV = super.getPOV(0);
		return POV > 90 && POV < 270;
	}

	/**
   	 * Read the value of the POV_RIGHT button on the controller.
   	 *
   	 * @return The state of the button.
   	 */
	public boolean getPOV_Right() {
		int POV = super.getPOV(0);
		return POV > 0 && POV < 180;
	}

	/**
   	 * Read the value of the POV_LEFT button on the controller.
   	 *
   	 * @return The state of the button.
   	 */
	public boolean getPOV_Left() {
		int POV = super.getPOV(0);
		return POV > 180;
	}

	/**
	 * @return the controllerSubsystem
	 */
	public ControllerSubsystem getControllerSubsystem() {
		return controllerSubsystem;
	}

	class ControllerSubsystem extends Subsystem {
		// Put methods for controlling this subsystem
		// here. Call these from Commands.
	  
		@Override
		public void initDefaultCommand() {
		  // nothing
		}
	  }
}
