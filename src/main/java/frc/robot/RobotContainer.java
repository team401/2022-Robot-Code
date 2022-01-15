package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.Constants.InputDevices;
import frc.robot.commands.ExampleCommand;
import frc.robot.commands.drivetrain.OperatorControl;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.ExampleSubsystem;
import edu.wpi.first.wpilibj2.command.Command;

public class RobotContainer {

  //initialize the two joysticks (left and right) and our gamepad
  private final Joystick leftJoystick = new Joystick(InputDevices.leftJoystickPort);
  private final Joystick rightJoystick = new Joystick(InputDevices.rightJoystickPort);
  private final XboxController gamepad = new XboxController(InputDevices.gamepadPort);

  //list out all of the subsystems we need in our robot
  private final DriveSubsystem drive = new DriveSubsystem();

  public RobotContainer() {

    drive.setDefaultCommand(
      new OperatorControl(
        drive, 
        () -> leftJoystick.getY(GenericHID.Hand.kLeft), 
        () -> leftJoystick.getX(GenericHID.Hand.kLeft), 
        () -> rightJoystick.getX(GenericHID.Hand.kRight), 
        true
      )
    );

    configureButtonBindings();

  }

  //where we put all of our button bindings
  private void configureButtonBindings() {



  }

  //prepares the robot for autonomous and send sthe command we should use
  public Command getAutonomousCommand() {
   
    return null;

  }
}
