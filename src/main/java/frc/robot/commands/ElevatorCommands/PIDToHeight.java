// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.ElevatorCommands;

import frc.robot.Constants;
import frc.robot.subsystems.*;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
// import edu.wpi.first.wpilibj2.command.PIDCommand;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class PIDToHeight extends Command {
  private Elevator elevator;
  private double desiredHeight; //desired height in inches
  private final PIDController pidController;
  private double kP = Constants.Elevator.KP_ELEV;
  private double kI = Constants.Elevator.KI_ELEV;
  private double kD = Constants.Elevator.KD_ELEV;

  /** Creates a new SetElevatorHeight. */
  public PIDToHeight(Elevator elevator, double desiredHeight) {
    pidController = new PIDController(kP, kI, kD);
    this.elevator = elevator;
    this.desiredHeight = desiredHeight;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(elevator);

    pidController.setSetpoint(this.desiredHeight);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    pidController.reset();
  }

  // Called every time the scheduler runs while the command is scheduled.

  public void execute() {
    elevator.setElevSpeed(pidController.calculate(elevator.getElevatorHeight()));
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    elevator.stopElevator();
  }


  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}