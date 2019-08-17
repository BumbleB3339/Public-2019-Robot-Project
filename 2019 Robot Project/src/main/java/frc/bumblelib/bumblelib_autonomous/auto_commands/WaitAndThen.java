package frc.bumblelib.bumblelib_autonomous.auto_commands;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.CommandGroup;
import edu.wpi.first.wpilibj.command.StartCommand;

/**
 *
 */
public class WaitAndThen extends CommandGroup {

	public WaitAndThen(double time, Command andThen) {
		this(time, andThen, true);
	}

	public WaitAndThen(double time, Command andThen, boolean requireSubsystem) {
		addSequential(new Wait(time));
		if (requireSubsystem) {
			addSequential(andThen);
		} else {
			addSequential(new StartCommand(andThen));
		}
	}
}
