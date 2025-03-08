package frc.robot;

import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import frc.lib.util.COTSTalonFXSwerveConstants;
import frc.lib.util.SwerveModuleConstants;

public final class Constants {
    public static final double stickDeadband = 0.1;

    public static class Controller {
        // usb port on the laptop when driver using XBox controller
        public static final int USB_DRIVECONTROLLER = 0;//for driver
        public static final int USB_AUXCONTROLLER = 1; // for controller operator
    }

    public static final class  Targeting {
        public static final double DIST_TO_CENTER = 18; //forward distance center to bumper, inches
        public static final double DIST_CAMERA_TO_BUMPER_FWD = 4.25;  //inches
    }

    public static final class Swerve {
        public static final int pigeonID = 1;

            //AUTO SWITCHES
        public static final int DIO_AUTO_1 = 0;
        public static final int DIO_AUTO_2 = 1;
        public static final int DIO_AUTO_3 = 2;
        public static final int DIO_AUTO_4 = 3;

        public static final COTSTalonFXSwerveConstants chosenModule =  //TODO: This must be tuned to specific robot
        COTSTalonFXSwerveConstants.SDS.MK4.KrakenX60(COTSTalonFXSwerveConstants.SDS.MK4.driveRatios.L2);

        /* Drivetrain Constants */
        public static final double trackWidth = Units.inchesToMeters(23.5); //2024 testbed
        public static final double wheelBase = Units.inchesToMeters(23.5); //2024 testbed
        public static final double wheelCircumference = chosenModule.wheelCircumference;

        /* Swerve Kinematics 
         * No need to ever change this unless you are not doing a traditional rectangular/square 4 module swerve */
         public static final SwerveDriveKinematics swerveKinematics = new SwerveDriveKinematics(
            new Translation2d(wheelBase / 2.0, trackWidth / 2.0),
            new Translation2d(wheelBase / 2.0, -trackWidth / 2.0),
            new Translation2d(-wheelBase / 2.0, trackWidth / 2.0),
            new Translation2d(-wheelBase / 2.0, -trackWidth / 2.0));

        /* Module Gear Ratios */
        public static final double driveGearRatio = chosenModule.driveGearRatio;
        public static final double angleGearRatio = chosenModule.angleGearRatio;

        /* Motor Inverts */
        public static final InvertedValue angleMotorInvert = chosenModule.angleMotorInvert;
        public static final InvertedValue driveMotorInvert = chosenModule.driveMotorInvert;

        /* Angle Encoder Invert */
        public static final SensorDirectionValue cancoderInvert = chosenModule.cancoderInvert;

        /* Swerve Current Limiting */
        public static final int angleCurrentLimit = 25; //TODO check compared to last year
        public static final int angleCurrentThreshold = 40;
        public static final double angleCurrentThresholdTime = 0.1;
        public static final boolean angleEnableCurrentLimit = true;

        public static final int driveCurrentLimit = 35;
        public static final int driveCurrentThreshold = 60;
        public static final double driveCurrentThresholdTime = 0.1;
        public static final boolean driveEnableCurrentLimit = true;

        /* These values are used by the drive falcon to ramp in open loop and closed loop driving.
         * We found a small open loop ramp (0.25) helps with tread wear, tipping, etc */
        public static final double openLoopRamp = 0.25;
        public static final double closedLoopRamp = 0.0;

        /* Angle Motor PID Values */
        public static final double angleKP = chosenModule.angleKP;
        public static final double angleKI = chosenModule.angleKI;
        public static final double angleKD = chosenModule.angleKD;

        //TODO remove until we use closed loop driving

        /* Drive Motor PID Values */    

        public static final double driveKP = 2.5; //0.5, 1 //TODO: This must be tuned to specific robot, default is 0.1
        public static final double driveKI = 0; //2
        public static final double driveKD = 0.0;
        public static final double driveKF = 0.0; 

        /* Drive Motor Characterization Values From SYSID */
        public static final double driveKS = 0; //0.32; //TODO: This must be tuned to specific robot
        public static final double driveKV = 0; //1.51;
        public static final double driveKA = 0; //.27; 

        /* Swerve Profiling Values */
        /** Meters per Second*/
        public static final double maxSpeed = 4.5;
        /** Radians per Second */
        public static final double maxAngularVelocity = 0.00001; //10.0
        //throttle to slow down drive speed
        public static final double throttle = 0.1;

        /* Neutral Modes */
        public static final NeutralModeValue angleNeutralMode = NeutralModeValue.Coast;
        public static final NeutralModeValue driveNeutralMode = NeutralModeValue.Brake;

        /* Module Specific Constants */

        /* Front Left Module - Module 0 */
        public static final class Mod0 { //TODO: This must be tuned to specific robot
            public static final int driveMotorID = 7;
            public static final int angleMotorID = 6;
            public static final int canCoderID = 3;
            public static final Rotation2d angleOffset = Rotation2d.fromDegrees(81.1+180); 
            public static final SwerveModuleConstants constants = 
                new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
        }
        /* Front Right Module - Module 1 */
            public static final class Mod1 { //TODO: This must be tuned to specific robot
            public static final int driveMotorID = 3;
            public static final int angleMotorID = 2;
            public static final int canCoderID = 1;
            public static final Rotation2d angleOffset = Rotation2d.fromDegrees(-20.83+180); 
            public static final SwerveModuleConstants constants = 
                new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
        }    
        /* Back Left Module - Module 2 */
        public static final class Mod2 { //TODO: This must be tuned to specific robot
            public static final int driveMotorID = 5;
            public static final int angleMotorID = 4;
            public static final int canCoderID = 2;
            public static final Rotation2d angleOffset = Rotation2d.fromDegrees(8.1+180); 
            public static final SwerveModuleConstants constants = 
                new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
        }
        /* Back Right Module - Module 3 */
        public static final class Mod3 { //TODO: This must be tuned to specific robot
            public static final int driveMotorID = 1;
            public static final int angleMotorID = 10;
            public static final int canCoderID = 0;
            public static final Rotation2d angleOffset = Rotation2d.fromDegrees(-17.75+180); 
            public static final SwerveModuleConstants constants = 
                new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
        }
  
    }

    public static final class AutoConstants { //TODO: The below constants are used in the example auto, and must be tuned to specific robot
        public static final double kMaxSpeedMetersPerSecond = 2;
        public static final double kMaxAccelerationMetersPerSecondSquared = 2;
        public static final double kMaxAngularSpeedRadiansPerSecond = 3*Math.PI;
        public static final double kMaxAngularSpeedRadiansPerSecondSquared = 3*Math.PI;
    //X = forward, Y = to the left for swerve??
        public static final double kPXController = 12; //1 default
        public static final double kPYController = 12; //1
        public static final double kPThetaController = 20; //1 default
    
        /* Constraint for the motion profilied robot angle controller */
        public static final TrapezoidProfile.Constraints kThetaControllerConstraints =
            new TrapezoidProfile.Constraints(
                kMaxAngularSpeedRadiansPerSecond, kMaxAngularSpeedRadiansPerSecondSquared);
    }
	
	public static class MotorControllers {
    public static final int SMART_CURRENT_LIMIT = 40;
    //Elevator (placeholder)
    public static final int ID_ELEVATOR_LEFT = 46;
    public static final int ID_ELEVATOR_RIGHT = 47;

    //AlgaeHold (placeholder)
    public static final int ID_ALGAE_HOLD = 61;   // was 60

   //CoralHold (placeholder)
    // public static final int ID_CORAL_HOLD_MOTORSRX = 14; //IF USE TALON SRX
    public static final int ID_CORAL_HOLD_MOTOR = 49;//BRUSHED SM MOTOR

    //AlgaePivot (placeholder)
    public static final int ID_ALGAE_PIVOT = 60;//Change to 50 when test algaehold

     //CoralPivot (placeholder)
     public static final int ID_CORAL_PIVOT = 500;
  }

  public static class Elevator {
    public static final int DIO_ELEV_TOP = 4;
    public static final int DIO_ELEV_BOTTOM = 5;

    public static final double ELEV_UP_SPEED = 0.15;
    public static final double ELEV_DOWN_SPEED = -0.15;

    //placeholder conversion factors
    public static final double ELEV_REV_TO_IN = 0.32327;
    public static final double ELEV_IN_TO_REV = 1/(0.32327);

    public static final double L1_HEIGHT = 6;
    public static final double L2_HEIGHT = 12;
    public static final double L3_HEIGHT = 18;
    
    public static final double MIN_HEIGHT = 1.5;
    public static final double MAX_HEIGHT = 27;

    //placeholder PID values
    public static final double KP_ELEV = 0.2;
    public static final double KI_ELEV = 0;
    public static final double KD_ELEV = 0;

  }

public static class AlgaeHold {
  public static final double HOLD_SPEED = 0.1;  // is this zero
  public static final double RELEASE_SPEED = -0.1;
}
public static class CoralHold {
  public static final int DIO_COUNTER = 12;
  public static final double HOLD_SPEED = .1;
  public static final double RELEASE_SPEED = -.1;
  public static final double L4_RELEASE_SPEED = .1;
}

  public static class AlgaePivot {
    public static final int DIO_EXT_LIMIT = 8; //change to 8 when test AlgaePivot
    public static final int DIO_RET_LIMIT = 9; //change to 9 when test AlgaePivot
    public static final double ALGAEPIVOT_CLEAR_OF_ELEVATOR = 20;  //TODO find angle clear actual value
    public static final double ENC_REVS_MAX = -74;
    public static final double ENC_REVS_TEST1 = -30;
    public static final double ENC_REVS_TEST2 = -52;
    public static final double MAN_EXT_SPEED = -0.1;
    public static final double MAN_RET_SPEED = 0.1;
    public static final double KP = 0.029;
    public static final double KI = 0;
    public static final double KD = 0;
    public static final double KFF = 0;
  }
  
  public static class CoralPivot {
    public static final int DIO_EXT_LIMIT = 8; //change to 8 when test AlgaePivot
    public static final int DIO_RET_LIMIT = 9; //change to 9 when test AlgaePivot
    public static final double CORALPIVOT_CLEAR_OF_ELEVATOR = 20;  //TODO find angle clear actual value
    public static final double ENC_REVS_MAX = -74;
    public static final double ENC_REVS_TEST1 = -30;
    public static final double ENC_REVS_TEST2 = -52;
    public static final double MAN_EXT_SPEED = -0.1;
    public static final double MAN_RET_SPEED = 0.1;
    public static final double KP = 0.029;
    public static final double KI = 0;
    public static final double KD = 0;
    public static final double KFF = 0;
    public static final int DIO_ENC_A = 0; //TODO change
    public static final int DIO_ENC_B = 0; //TODO change
  }

    public static class XboxController {
        public static final int A = 1;
        public static final int B = 2;
        public static final int X = 3;
        public static final int Y = 4;
        public static final int LB = 5;
        public static final int RB = 6;
        public static final int VIEW = 7;
        public static final int MENU = 8;
        public static final int LM = 9;
        public static final int RM = 10;
    
        public static class AxesXbox {
          public static final int LX = 0;
          public static final int LY = 1;
          public static final int LTrig = 2;
          public static final int RTrig = 3;
          public static final int RX = 4;
          public static final int RY = 5;
        }
    
        public class POVXbox {
          public static final int UP_ANGLE = 0;
          public static final int RIGHT_ANGLE = 90;
          public static final int DOWN_ANGLE = 180;
          public static final int LEFT_ANGLE = 270;
        }
      }
    
}
