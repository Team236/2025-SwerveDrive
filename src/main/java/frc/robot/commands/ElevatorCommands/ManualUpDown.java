// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.ElevatorCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.AlgaePivot;
import frc.robot.subsystems.Elevator;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class ManualUpDown extends Command {

  private Elevator elevator;
  private double speed;

  /** Creates a new ManualUpDown. */
  public ManualUpDown(Elevator elevator, double speed) {
    this.elevator = elevator;
    this.speed = speed;

    addRequirements(this.elevator);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    //may need to use -speed?
    elevator.setElevSpeed(speed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    elevator.stopElevator();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    //TODO subsystem check(s) verify elvator is clear of obstructing components 
    // verify both CORALPIVOT and ALGAE_PIVOT are not stowed. new method to verify isClear()  
    // method should checking Rotation(s) is past some specified constants value(s)  
    
    return false;
  }
}
