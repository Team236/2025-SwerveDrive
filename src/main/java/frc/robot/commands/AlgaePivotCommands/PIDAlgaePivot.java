// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.AlgaePivotCommands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.AlgaePivot;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class PIDAlgaePivot extends Command {
  /** Creates a new PIDAlgaePivot. */
  private AlgaePivot algaePivot;
  //private double revs;
  private final PIDController pidController;
  private double kP = Constants.AlgaePivot.KP;
  private double kI = Constants.AlgaePivot.KI;
  private double kD = Constants.AlgaePivot.KD;

  public PIDAlgaePivot(AlgaePivot algaePivot, double revs) {
    pidController = new PIDController(kP, kI, kD);
    this.algaePivot = algaePivot;

    addRequirements(this.algaePivot);
    pidController.setSetpoint(revs);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    pidController.reset();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    algaePivot.setAlgaePivotSpeed(pidController.calculate(algaePivot.getPivotEncoder()));
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    algaePivot.stopAlgaePivot();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
