// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class PathFollowingCmd extends Command {
  
  /* contructer */
    public void PathFollowingCmd() {
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // assign the specified path

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    /* parallel commands group 
    *  1a. drivePath1 from BlueOne to reef 60-A position 
    *  1b. coralPivot to ready to Top_coral_deploy (so elevator is free to move)
    *  1c. algaePivot to unstowed                  (so elevator is free to move) */
    
    // 2. elavator to Top_coral_deploy position
    // 3. CoralDeploy 

    /* parallel commands group 
    * 4a. elevator goto retrieve_coral_position
    * 4b. coralPivot to retrieve_coral_position  
    * 4c. drivePath from reef 60-A to CoralSupply  */
    
    // 5. CoralIntake_and_Hold
    
    /* parallel commands
    *  6a. drivePath from CoralSupply to reef 60b 
    *  6b. coralPivot to ready to Top_coral_deploy 
    *  6c. elavator raise to Top_coral_deploy position  */

    // 7. CoralDeploy

    /* parallel commands group 
    * 8a. elevator goto retrieve_coral_position
    * 8b. coralPivot to retrieve_coral_position  
    * 8c. drivePath from reef 60-B to CoralSupply  */
  
    // 9. CoralIntake_and_Hold
    
    /* parallel commands
    *  10a. drivePath from CoralSupply to reef 60b 
    *  10b. coralPivot to ready to Top_coral_deploy 
    *  10c. elavator to Top_coral_deploy position  */

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    // call stop elevator
    // call stop coralIntakeHold
    // stop Coralpivot and AlgaePivot at current position
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
