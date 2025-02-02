package frc.robot;

import com.pathplanner.lib.commands.PathPlannerAuto;
import com.fasterxml.jackson.databind.util.Named;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import frc.robot.autos.exampleAuto;
import frc.robot.commands.TeleopSwerve;
import frc.robot.commands.AlgaeHoldCommands.AlgaeGrab;
import frc.robot.commands.AlgaePivotCommands.ManualAlgaePivot;
import frc.robot.commands.AlgaePivotCommands.PIDAlgaePivot;
import frc.robot.commands.CoralHoldCommands.CoralGrab;
import frc.robot.commands.CoralHoldCommands.CoralGrabWithCounter;
import frc.robot.commands.CoralHoldCommands.CoralRelease;
import frc.robot.commands.CoralPivotCommands.ManualCoralPivot;
import frc.robot.commands.CoralPivotCommands.PIDCoralPivot;
import frc.robot.commands.ElevatorCommands.ManualUpDown;
import frc.robot.commands.ElevatorCommands.PIDToHeight;
import frc.robot.commands.Targeting.TargetAllParallel;
import frc.robot.commands.Targeting.TargetAllSeries;
import frc.robot.commands.Targeting.TargetAngle;
import frc.robot.commands.Targeting.TargetForwardDistance;
import frc.robot.commands.Targeting.TargetMegaTag2;
import frc.robot.commands.Targeting.TargetSideDistance;
import frc.robot.subsystems.AlgaeHold;
import frc.robot.subsystems.AlgaePivot;
import frc.robot.subsystems.CoralHold;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.coralPivot;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
    /* Controllers */
    //private final Joystick driver = new Joystick(0);
    XboxController driver = new XboxController(Constants.Controller.USB_DRIVECONTROLLER);
    XboxController auxController = new XboxController(Constants.Controller.USB_AUXCONTROLLER);

  //AUTO SWITCHES
/**
 * A DigitalInput object representing the first autonomous mode switch.
 * This switch is used to determine the autonomous mode to be executed.
 * connected to the DIO ports specified by Constants.
 */
    private static DigitalInput autoSwitch1 = new DigitalInput(Constants.Swerve.DIO_AUTO_1);
    private static DigitalInput autoSwitch2 = new DigitalInput(Constants.Swerve.DIO_AUTO_2);
    private static DigitalInput autoSwitch3 = new DigitalInput(Constants.Swerve.DIO_AUTO_3);
    private static DigitalInput autoSwitch4 = new DigitalInput(Constants.Swerve.DIO_AUTO_4);

   //create instance of each subsystem  
    private final Swerve s_Swerve = new Swerve();
    // other subsystems
    private final AlgaeHold  algaeHold = new AlgaeHold();
    private final AlgaePivot algaePivot = new AlgaePivot();
    private final Elevator elevator = new Elevator();
    private final CoralHold coralHold = new CoralHold();
    private final coralPivot coralPivot = new coralPivot();

  //Other Commmands - Any pid commands put pid at the beginning, then put the subsystem, then put the action :)
    //AlgaeHold
    private final AlgaeGrab algaeGrabPull = new AlgaeGrab(algaeHold, Constants.AlgaeHold.HOLD_SPEED);
    private final AlgaeGrab algaeGrabRelease = new AlgaeGrab(algaeHold, Constants.AlgaeHold.RELEASE_SPEED);

    //AlgaePivot
    private final ManualAlgaePivot algaePivotDown = new ManualAlgaePivot(algaePivot, Constants.AlgaePivot.MAN_EXT_SPEED);
    private final ManualAlgaePivot algaePivotUp = new ManualAlgaePivot(algaePivot, Constants.AlgaePivot.MAN_RET_SPEED);

    private final PIDAlgaePivot pidAlgaePivot1 = new PIDAlgaePivot(algaePivot, Constants.AlgaePivot.ENC_REVS_TEST1);
    private final PIDAlgaePivot pidAlgaePivot2 = new PIDAlgaePivot(algaePivot, Constants.AlgaePivot.ENC_REVS_TEST2);
    
    //Elevator
    private final ManualUpDown elevatorUp = new ManualUpDown(elevator, Constants.Elevator.ELEV_UP_SPEED);
    private final ManualUpDown elevatorDown = new ManualUpDown(elevator, Constants.Elevator.ELEV_DOWN_SPEED);

    private final PIDToHeight pidElevatorL1 = new PIDToHeight(elevator, Constants.Elevator.L1_HEIGHT);
    private final PIDToHeight pidElevatorL2 = new PIDToHeight(elevator, Constants.Elevator.L2_HEIGHT);
    private final PIDToHeight pidElevatorL3 = new PIDToHeight(elevator, Constants.Elevator.L3_HEIGHT);

    //CoralHold
    private final CoralGrab coralGrab = new CoralGrab(coralHold, Constants.CoralHold.HOLD_SPEED);
    private final CoralGrabWithCounter coralGrabWithCounter = new CoralGrabWithCounter(coralHold, Constants.CoralHold.HOLD_SPEED);
    private final CoralRelease coralRelease = new CoralRelease(coralHold, Constants.CoralHold.RELEASE_SPEED);
    private final CoralRelease coralReleaseL4 = new CoralRelease(coralHold, Constants.CoralHold.L4_RELEASE_SPEED);

    //CoralPivot
    private final ManualCoralPivot coralPivotDown = new ManualCoralPivot(coralPivot, Constants.CoralPivot.MAN_EXT_SPEED);
    private final ManualCoralPivot coralPivotUp = new ManualCoralPivot(coralPivot, Constants.CoralPivot.MAN_RET_SPEED);
    private final PIDCoralPivot pidCoralPivot1 = new PIDCoralPivot(coralPivot, Constants.CoralPivot.ENC_REVS_TEST1);
    private final PIDCoralPivot pidCoralPivot2 = new PIDCoralPivot(coralPivot, Constants.CoralPivot.ENC_REVS_TEST2);
 
    private final NamedCommands currentNamedCommands = new NamedCommands();

    /* Drive Controls */
    private final int translationAxis = XboxController.Axis.kLeftY.value;
    private final int strafeAxis = XboxController.Axis.kLeftX.value;
    private final int rotationAxis = XboxController.Axis.kRightX.value;

    /* Driver Buttons */
    private final JoystickButton zeroGyro = new JoystickButton(driver, XboxController.Button.kY.value);
    private final JoystickButton robotCentric = new JoystickButton(driver, XboxController.Button.kLeftBumper.value);

    //DRIVE COMMANDS
    //***Forward standoff - input into command as inches from bumper
    private final TargetAllParallel targetAllParallel = new TargetAllParallel(s_Swerve, 12, 0);
    private final TargetAllSeries targetAllSeries = new TargetAllSeries(s_Swerve, 12, 0);
    private final TargetAngle targetAngle =  new TargetAngle(s_Swerve, -driver.getRawAxis(translationAxis), -driver.getRawAxis(strafeAxis));
    private final TargetForwardDistance targetForwardDistance = new TargetForwardDistance(s_Swerve, -driver.getRawAxis(strafeAxis), -driver.getRawAxis(rotationAxis), 12);
    private final TargetSideDistance targetSideDistance = new TargetSideDistance(s_Swerve, -driver.getRawAxis(translationAxis), -driver.getRawAxis(rotationAxis), 0);
    private final TargetMegaTag2 target3DMegaTag2 = new TargetMegaTag2(s_Swerve);

     

    /** The container for the robot. Contains subsystems, OI devices, and commands. */
    public RobotContainer() {
        s_Swerve.setDefaultCommand(
            new TeleopSwerve(
                s_Swerve, 
                () -> -driver.getRawAxis(translationAxis), 
                () -> -driver.getRawAxis(strafeAxis), 
                () -> -driver.getRawAxis(rotationAxis), 
                () -> robotCentric.getAsBoolean()
            )
        );

        getAutonomousCommand();
        createPathPlannerCommands();
                // Configure the button bindings
                configureButtonBindings();
            }
        
        // MIGHT NEED TO ABANDON THIS METHOD AS WE DON'T DEFINE COMMANDS IN SUBSYSTEMS
        //  SHOULD EVALUATE EVENTS INSTEAD TO NAME COMMANDS INSTEAD OF IN SUBSYSTEMS
            private void createPathPlannerCommands() {
                
                // TODO Register SystemCommands as Named Commands for PathPlanner in Autonomous
                // the PathPlanner docs example has commands in the subsystem but we want to call commands instead
                /* 
                *   Named commands must be registered before the creation of any PathPlanner Autos or Paths. 
                *   It is recommended to do this in RobotContainer, after subsystem initialization, 
                *   but before the creation of any other commands.
                */
                        // NamedCommands.registerCommand("autoBalance", swerve.autoBalanceCommand());
                        // NamedCommands.registerCommand("exampleCommand", exampleSubsystem.exampleCommand());
                        // NamedCommands.registerCommand("SomeOtherCommand", new SomeOtherCommand());
                    
                    currentNamedCommands.registerCommand("algaeGrabPull", algaeGrabPull.schedule());   
                    // NamedCommands.registerCommand("algaeGrabPull", algaeGrabPull );
                // NamedCommands.registerCommand("algaeGrabRelease", algaeGrabRelease );
                // NamedCommands.registerCommand("lgaePivotDown", algaePivotDown );
                // NamedCommands.registerCommand("algaePivotUp", algaePivotUp );
                // NamedCommands.registerCommand("pidAlgaePivot1", pidAlgaePivot1 );
                // NamedCommands.registerCommand("pidAlgaePivot2", pidAlgaePivot2 );
                // NamedCommands.registerCommand("elevatorUp", elevatorUp );
                // NamedCommands.registerCommand("elevatorDown", elevatorDown );
                // NamedCommands.registerCommand("pidElevatorL1", pidElevatorL1 );
                // NamedCommands.registerCommand("pidElevatorL2", pidElevatorL2 );
                // NamedCommands.registerCommand("pidElevatorL3", pidElevatorL3 );
                // NamedCommands.registerCommand("coralGrab", coralGrab );
                // NamedCommands.registerCommand("coralGrabWithCounter", coralGrabWithCounter );
                // NamedCommands.registerCommand("coralRelease", coralRelease );
                // NamedCommands.registerCommand("coralReleaseL", coralReleaseL );
                // NamedCommands.registerCommand("coralPivotDown", coralPivotDown );
                // NamedCommands.registerCommand("coralPivotUp", coralPivotUp );
                // NamedCommands.registerCommand("pidCoralPivot1", pidCoralPivot1 );
                // NamedCommands.registerCommand("pidCoralPivot2", pidCoralPivot2 );
            }
        
        
            /**
     * Retrieves the autonomous command for the robot.
     * <p>
     * This method loads the autonomous command when it is called. However, it is
     * recommended to pre-load your paths or autonomous commands when the code starts,
     * and then return the pre-loaded command.
     *
     * @return the autonomous command to be executed
     */
    public Command getAutonomousCommand(String path) {
        // This method loads the auto when it is called, however, it is recommended
        // to first load your paths/autos when code starts, then return the
        // pre-loaded auto/path
        return new PathPlannerAuto(path);
      }


    /**
     * Use this method to define your button->command mappings. Buttons can be created by
     * instantiating a {@link GenericHID} or one of its subclasses ({@link
     * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
     * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
     */
    private void configureButtonBindings() {
        /* Driver Buttons */
        zeroGyro.onTrue(new InstantCommand(() -> s_Swerve.zeroHeading()));
        
 // XBOXCONTROLLER - DRIVER CONTROLLER
    JoystickButton x = new JoystickButton(driver, Constants.XboxController.X);
    JoystickButton a = new JoystickButton(driver, Constants.XboxController.A);
    JoystickButton b = new JoystickButton(driver, Constants.XboxController.B);
   //Y on driver controller is assigned to "zeroGyro" above
   // JoystickButton y = new JoystickButton(driver, Constants.XboxController.Y);
   //leftBumper on driver controller is assigned to "robotCentric" above
   // JoystickButton lb = new JoystickButton(driver, Constants.XboxController.LB);
    JoystickButton rb = new JoystickButton(driver, Constants.XboxController.RB);
    JoystickButton lm = new JoystickButton(driver, Constants.XboxController.LM);
    JoystickButton rm = new JoystickButton(driver, Constants.XboxController.RM);
    JoystickButton view = new JoystickButton(driver, Constants.XboxController.VIEW);
    JoystickButton menu = new JoystickButton(driver, Constants.XboxController.MENU);
    POVButton upPov = new POVButton(driver,Constants.XboxController.POVXbox.UP_ANGLE);
    POVButton downPov = new POVButton(driver,Constants.XboxController.POVXbox.DOWN_ANGLE); 
    POVButton leftPov = new POVButton(driver,Constants.XboxController.POVXbox.LEFT_ANGLE);
    POVButton rightPov = new POVButton(driver,Constants.XboxController.POVXbox.RIGHT_ANGLE);
    // XBOX CONTROLLER - AUX CONTROLLER
    JoystickButton x1 = new JoystickButton(auxController, Constants.XboxController.X);
    JoystickButton a1 = new JoystickButton(auxController, Constants.XboxController.A);
    JoystickButton b1 = new JoystickButton(auxController, Constants.XboxController.B);
    JoystickButton y1 = new JoystickButton(auxController, Constants.XboxController.Y);
    JoystickButton lb1 = new JoystickButton(auxController, Constants.XboxController.LB);
    JoystickButton rb1 = new JoystickButton(auxController, Constants.XboxController.RB);
    JoystickButton lm1 = new JoystickButton(auxController, Constants.XboxController.LM);
    JoystickButton rm1 = new JoystickButton(auxController, Constants.XboxController.RM);
    JoystickButton view1 = new JoystickButton(auxController, Constants.XboxController.VIEW);
    JoystickButton menu1 = new JoystickButton(auxController, Constants.XboxController.MENU);
    POVButton upPov1 = new POVButton(auxController,Constants.XboxController.POVXbox.UP_ANGLE);
    POVButton downPov1 = new POVButton(auxController,Constants.XboxController.POVXbox.DOWN_ANGLE);
    POVButton leftPov1 = new POVButton(auxController,Constants.XboxController.POVXbox.LEFT_ANGLE);
    POVButton rightPov1 = new POVButton(auxController,Constants.XboxController.POVXbox.RIGHT_ANGLE);

    //y button is already assigned to ZeroGyro
    //leftBumper button is already assigned to RobotCentric
    a.whileTrue(targetAllSeries);
    b.whileTrue(targetAllParallel);
    upPov.whileTrue(targetAngle);
    x.whileTrue(targetForwardDistance);
    rb.whileTrue(targetSideDistance);
    downPov.whileTrue(target3DMegaTag2);
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        // An ExampleCommand will run in autonomous
        return new exampleAuto(s_Swerve, false);
        
    }
}
