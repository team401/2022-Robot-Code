package frc.robot;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.XboxController.Button;
import frc.robot.Constants.ClimberConstants;
import frc.robot.Constants.InputDevices;
import frc.robot.commands.climber.ExtendTelescope;
import frc.robot.commands.climber.RetractTelescope;
import frc.robot.commands.climber.UpdateRotationArm;
import frc.robot.commands.drivetrain.OperatorControl;
import frc.robot.commands.drivetrain.RunAtPercent;
import frc.robot.subsystems.ClimbSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import pabeles.concurrency.ConcurrencyOps.NewInstance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

public class RobotContainer {

  //initialize the two joysticks (left and right) and our gamepad
  private final Joystick leftJoystick = new Joystick(InputDevices.leftJoystickPort);
  private final Joystick rightJoystick = new Joystick(InputDevices.rightJoystickPort);
  private final XboxController gamepad = new XboxController(InputDevices.gamepadPort);

  //list out all of the subsystems we need in our robot
  private final DriveSubsystem drive = new DriveSubsystem();
  private final ClimbSubsystem climb = new ClimbSubsystem();

  public RobotContainer() {

    //sets the default command of the drive to Operator Control (normal teleop driving) 
    //this set uses the gamepad as input for easy testing on field
    //for more info, go to the command
    /*drive.setDefaultCommand(
     new OperatorControl(
        drive, 
        () -> gamepad.getLeftY(), //Left up & down
        () -> gamepad.getLeftX(), //Left side-to-side
        () -> gamepad.getRightX(), //Right side-to-side
        true
      )
    );*/

    
    //uncomment this to use the competition control set-up (with two joysticks rather than the gamepad)
    drive.setDefaultCommand(
     new OperatorControl(
        drive, 
        () -> leftJoystick.getY(), //Left up & down
        () -> leftJoystick.getX(), //Left side-to-side
        () -> rightJoystick.getX(), //Right side-to-side
        true
      )
    );
    

    configureButtonBindings();

  }

  //where we put all of our button bindings
  private void configureButtonBindings() {

    /*
    Gamepad
      Y
    X   B
      A
    */

    //for testing
    //runs both the drive and rotation motors at a set speed to make sure they are all working
    //new JoystickButton(gamepad, Button.kX.value)
      //.whileHeld(new RunAtPercent(drive));

        //Climber Button Bindings ***TEMPORARY***
    /**
     * A = Intake Arm Position
     * B = Climb Arm Position
     * C = Default Arm Position
     * 
     * Left Xbox Trigger = Extend Telescope
     * Right Xbox Trigger = Retract Telescope
     */

    new JoystickButton(gamepad, Button.kA.value)
        .whenPressed(new UpdateRotationArm(climb, ClimberConstants.intakeArmPosition));

    new JoystickButton(gamepad, Button.kB.value)
        .whenPressed(new UpdateRotationArm(climb, ClimberConstants.climbArmPosition));

    new JoystickButton(gamepad, Button.kX.value)
        .whenPressed(new UpdateRotationArm(climb, ClimberConstants.defaultArmPosition));

    new JoystickButton(gamepad, Button.kLeftBumper.value)
        .whenPressed(new ExtendTelescope(climb));

    new JoystickButton(gamepad, Button.kRightBumper.value)
        .whenPressed(new RetractTelescope(climb));


    


  }

  //prepares the robot for autonomous and send sthe command we should use
  public Command getAutonomousCommand() {
   
    return null;

  }
}
