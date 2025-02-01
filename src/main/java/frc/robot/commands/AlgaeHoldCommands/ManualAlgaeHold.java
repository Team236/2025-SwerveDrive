// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.AlgaeHoldCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.AlgaeHold;

public class ManualAlgaeHold extends Command {
  private AlgaeHold algaeHold;
  private double speed;

  public ManualAlgaeHold(AlgaeHold algaeHold, double speed) {
    this.algaeHold = algaeHold;
    this.speed = speed;
    
    addRequirements(this.algaeHold);
  }

  @Override
  public void initialize() {}
  
  @Override
  public void execute() {
    this.algaeHold.setAlgaeHoldSpeed(speed);
  }
  
  @Override
  public void end(boolean interrupted) {
    this.algaeHold.stopAlgaeHold();
  }
  @Override
  public boolean isFinished() {
    return false;
  }
}