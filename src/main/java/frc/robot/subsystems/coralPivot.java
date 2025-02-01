// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;


//Some of this code is 2024 code, and may not be relevant
//if you see a 'tilt' varible, it is old code
//if you see a 'coral' variable, it is new code
//TODO make tilt varibles into coral varibles
//TODO verify this is brushed or brushless motor 

public class coralPivot extends SubsystemBase {
  
  private SparkMax coralPivotMotor;
  private SparkBaseConfig coralPivotConfig;
  private Encoder coralPivotEncoder;
  private boolean isCoralPivotExtException, isCoralPivotRetException;
  private DigitalInput CoralExtLimit, CoralRetLimit;
 

    /** Creates a new CoralPivot. */
    public coralPivot() {
    coralPivotMotor = new SparkMax(Constants.MotorControllers.ID_CORAL_PIVOT, MotorType.kBrushed);
    coralPivotEncoder = new Encoder(Constants.CoralPivot.DIO_ENC_A, Constants.CoralPivot.DIO_ENC_B);

    coralPivotConfig.inverted(false);//WAS TRUE - NOW USE NEGATIVE ENC VALUES TO TILT
    coralPivotConfig.smartCurrentLimit(Constants.MotorControllers.SMART_CURRENT_LIMIT);
    coralPivotMotor.configure(coralPivotConfig,SparkBase.ResetMode.kResetSafeParameters ,SparkBase.PersistMode.kPersistParameters);
        
    try {
      //  This tries to make a new digital input, and if it fails, throws an error 
      CoralExtLimit = new DigitalInput(Constants.CoralPivot.DIO_EXT_LIMIT);
    } catch (Exception e) {
       isCoralPivotExtException = true;
      SmartDashboard.putBoolean("exception thrown for Coral Extend limit: ", isCoralPivotExtException);
    }
    try {
      //  This sets a bottom limit for the coral, and if it fails, throws an error
      CoralRetLimit = new DigitalInput(Constants.CoralPivot.DIO_RET_LIMIT);
    } catch (Exception e) {
      isCoralPivotRetException = true;
      SmartDashboard.putBoolean("exception thrown for Coral Retract limit: ", isCoralPivotRetException);
    }
}

// methods start here
public double getCoralEncoder() {  //gives encoder reading in Revs
  return coralPivotEncoder.getRaw();
}

public void resetCoralEncoder() {
  coralPivotEncoder.reset();
}

public void stopCoralPivot() {
coralPivotMotor.set(0);
}

public double getCoralPivotSpeed() {
  return coralPivotMotor.get();
}
public boolean isCoralExtLimit() {
if (isCoralPivotExtException) {
  return true;
} else {
  return CoralExtLimit.get();
}
}

public boolean isCoralRetLimit() {
if (isCoralPivotRetException) {
  return true;
} else {
  return CoralRetLimit.get();
}
}

public boolean isFullyExtended() {
  return (getCoralEncoder() <= Constants.CoralPivot.ENC_REVS_MAX);
}

public void setCoralPivotSpeed(double speed) {
  if (speed <= 0) {  //was >0 but changed since going negative when extending now
     //TODO make sure elevator speed > 0 when going up
    if (isCoralExtLimit() || isFullyExtended()) {
        // if fully extended limit is tripped or cartridge at the maximum desired extension and going out, stop 
        stopCoralPivot();
     }  else {
        // cartridge extending out but fully extended limit is not tripped, go at commanded speed
       coralPivotMotor.set(speed);
      }
 } 
 else {
      if (isCoralRetLimit()) {
        // cartridge retracting and retract limit is tripped, stop and zero encoder
        stopCoralPivot();
        resetCoralEncoder();
      } else {
        // cartridge retracting but fully retracted limit is not tripped, go at commanded speed
        coralPivotMotor.set(speed);
      }
     }
}



//Begin things that may not be relevant
//these are things that might be useful in the future if we use CANSparkMax PID
//we are not currently using it

//!!!! SPARKMAX PID STUFF - USE SPARKMAX PID, NOT WPILib PID 
//**** CHANGED BACK TO USING WPILib PID ****
//**** due to spurious encoder polarity changes when run multiple autos in a row ****
/*
public void setSetpoint(double encoderRevs) {
  tiltPIDController.setReference(encoderRevs, ControlType.kPosition);
}

public void setP(double kP) {
  tiltPIDController.setP(kP);
}

public void setI(double kI) {
  tiltPIDController.setI(kI);
}

public void setD(double kD) {
  tiltPIDController.setD(kD);
}

public void setFF(double kFF) {
  tiltPIDController.setFF(kFF);
}
*/


@Override
  public void periodic() {
    SmartDashboard.putBoolean("Coral Pivot Extend Limit: ", isCoralExtLimit());
    SmartDashboard.putBoolean("Coral Pivot Retract Limit", isCoralRetLimit());
    SmartDashboard.putNumber("Coral Pivot Encoder Revolutions ", getCoralEncoder());
    SmartDashboard.putBoolean("Coral Pivot is fully extended: ", isFullyExtended());
  }

}
