package frc.bumblelib.bumblelib_autonomous.auto_commands;

import edu.wpi.first.wpilibj.command.Command;

/**
 *
 */
public class Wait extends Command {

	private double seconds;

	public Wait(double seconds) {
		this.seconds = seconds;
	}

	@Override
	protected void initialize() {
		super.setTimeout(seconds);
	}

	@Override
	protected void execute() {
	}

	@Override
	protected boolean isFinished() {
		return isTimedOut();
	}

	@Override
	protected void end() {
	}

	@Override
	protected void interrupted() {
	}
}
