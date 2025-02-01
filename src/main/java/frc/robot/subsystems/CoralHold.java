// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
//import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
//import com.ctre.phoenix.motorcontrol.can.TalonSRX;
//import com.ctre.phoenix.motorcontrol.can.TalonSRXConfiguration;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj.Counter;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;


public class CoralHold extends SubsystemBase {
/** Creates a new CoralHold. */
  // // TALON SRX
  // private TalonSRX coralMotor;
  // private TalonSRXConfiguration coralConfig;
  // private TalonSRXControlMode controlMode = TalonSRXControlMode.PercentOutput;

  // SPARK MAX
  private SparkMax coralHoldMotor;
  private SparkMaxConfig coralHoldConfig;

  // COUNTER
  public Counter counter;
  public boolean isCounterUnplugged = false;

 
  public CoralHold() {
// // TALON SRX
    // coralMotor = new TalonSRX(Constants.MotorControllers.ID_CORALMOTOR);
    // coralMotor.setInverted(true);

    // // 3 lines below may not be necessary
    // // coralConfig = new TalonSRXConfiguration();
    // // coralConfig.peakCurrentLimit = Constants.MotorControllers.SMART_CURRENT_LIMIT;
    // // coralMotor.configAllSettings(coralConfig);
    

    // SPARK MAX
    coralHoldMotor = new SparkMax(Constants.MotorControllers.ID_CORAL_HOLD_MOTOR , MotorType.kBrushless);

    coralHoldConfig = new SparkMaxConfig();
    coralHoldConfig.inverted(true);
    coralHoldConfig.smartCurrentLimit(Constants.MotorControllers.SMART_CURRENT_LIMIT);

    coralHoldMotor.configure(coralHoldConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);

    try {
      counter = new Counter();
      counter.setUpSource(Constants.CoralHold.DIO_COUNTER);
      counter.reset();
    }

    catch (Exception e) {
      isCounterUnplugged = true;
    }

    SmartDashboard.putBoolean("is counter unplugged:", isCounterUnplugged);
    counter.reset(); //sets counter to zero
  }

  //METHOSD START HERE

  public void resetCount() {
    // automaticaly sets counter to 0 at start 
    counter.reset();
  }

  public void setCoralHSpeed(double speed) {
    coralHoldMotor.set(speed);
  }

  public void coralHStop () {
    // // TALON SRX
    // coralMotor.set(controlMode, 0);

    // SPARK MAX
    coralHoldMotor.set(0);
  }

  public int getCoralHCount() {
    int count;
    if (isCounterUnplugged) {
      count = 0;
      SmartDashboard.putBoolean("Intake counter unplugged:", isCounterUnplugged);
    } else {
      count =  counter.get();
    }
    return count;
  }

  public boolean isCoralSpinning() {
    boolean  spin;
    
    // // TALON SRX
    // if (Math.abs(coralMotor.getMotorOutputPercent()) > 0.1) {
    //   spin = true;
    // }
    // else {
    //   spin = false;
    // }
    // return spin;

    // SPARK MAX
    if (Math.abs(coralHoldMotor.get()) >0.08) {
      spin = true;
    }
    else {
      spin = false;
    }
    return spin;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Intake periodic count is:", getCoralHCount());
   SmartDashboard.putBoolean("HasNote: ", counter.get()>0);
  }

}
