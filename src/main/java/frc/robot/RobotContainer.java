package frc.robot;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.XboxController.Button;
import frc.robot.Constants.InputDevices;
import frc.robot.commands.drivetrain.OperatorControl;
import frc.robot.commands.drivetrain.RunAtPercent;
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

  public RobotContainer() {

    //sets the default command of the drive to Operator Control (normal teleop driving) 
    //for more info, go to the command
    drive.setDefaultCommand(
     new OperatorControl(
        drive, 
        () -> gamepad.getLeftY(),//.getY(), //Does not work if we put the hand
        () -> leftJoystick.getX(), //Does not work if we put the hand
        () -> gamepad.getRightX(), //rightJoystick.getX(), //Does not work if we put the hand
        true
      )
    );

    configureButtonBindings();

  }

  //where we put all of our button bindings
  private void configureButtonBindings() {

    new JoystickButton(gamepad, Button.kB.value) 
      .whileHeld(new InstantCommand(drive::runTestatPercent))
      .whenReleased(new InstantCommand(drive::stopDriving));

    new JoystickButton(gamepad, Button.kA.value)
      .whileHeld(new InstantCommand(drive::runTestatPercentSpin))
      .whenReleased(new InstantCommand(drive::stopDriving));

      new JoystickButton(gamepad, Button.kX.value)
      .whileHeld(new RunAtPercent(drive));

  }

  //prepares the robot for autonomous and send sthe command we should use
  public Command getAutonomousCommand() {
   
    return null;

  }
}
