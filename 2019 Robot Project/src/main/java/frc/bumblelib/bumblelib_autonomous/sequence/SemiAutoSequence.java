/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.bumblelib.bumblelib_autonomous.sequence;

import java.util.ArrayList;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;

public abstract class SemiAutoSequence extends SendableChooser<Command> implements Cloneable {

  private String sequenceName;
  private int commandIndex;
  private Command currentCommand;
  private static boolean isFirstRun = true;  
  private ArrayList<Command> commands;

  public SemiAutoSequence(String sequenceName) {
    this.sequenceName = sequenceName;
    commands = new ArrayList<Command>();
    commandIndex = 0;
  }

  protected void add(AutoSequence sequence) {
    commands.add(sequence);
    addOption(sequence.getName(), sequence);
  }

  private void runFirst() {
    currentCommand = commands.get(commandIndex);
    commandIndex++;
    currentCommand.start();
  }

  public void runNext() {
    if(isFirstRun){
      runFirst();
      isFirstRun = false;
      return;
    }

    if (commandIndex >= commands.size() || currentCommand.isRunning()) {
      return;    
    }
    
    currentCommand = commands.get(commandIndex);
    commandIndex++;
    currentCommand.start();
  }

  public String getSequenceName() {
    return sequenceName;
  }
  
  @Override
  protected Object clone() {
    try {
      return super.clone();
    } catch (CloneNotSupportedException e) {
      throw new RuntimeException("Can't clone sequence");
    }
  }
}