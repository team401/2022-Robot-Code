// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.drive;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import frc.robot.RobotState;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.drive.Drive;

public class ShootWhileMoving extends CommandBase {
  private final Drive drive;
  private final DoubleSupplier xPercent;
  private final DoubleSupplier yPercent;
  private final DoubleSupplier omegaPercent;
  private final boolean fieldRelative;
  private final XboxController gamepad;

  private final AxisProcessor xProcessor = new AxisProcessor(false);
  private final AxisProcessor yProcessor = new AxisProcessor(false);
  private final AxisProcessor omegaProcessor = new AxisProcessor(true);

  /** Creates a new DriveWithJoysticks. */
  public ShootWhileMoving(Drive drive, DoubleSupplier xPercent, DoubleSupplier yPercent, DoubleSupplier omegaPercent, boolean fieldRelative, XboxController gamepad) {
    this.drive = drive;
    this.xPercent = xPercent;
    this.yPercent = yPercent;
    this.omegaPercent = omegaPercent;
    this.fieldRelative = fieldRelative;
    this.gamepad = gamepad;

    addRequirements(drive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    xProcessor.reset(xPercent.getAsDouble());
    yProcessor.reset(yPercent.getAsDouble());
    omegaProcessor.reset(omegaPercent.getAsDouble());
    RobotState.getInstance().setLookAhead(true);
    gamepad.setRumble(RumbleType.kLeftRumble, 1);
    gamepad.setRumble(RumbleType.kRightRumble, 1);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double xMPerS = xProcessor.processJoystickInputs(xPercent.getAsDouble()) * DriveConstants.maxSpeedWhileShootingMPerS;
    double yMPerS = yProcessor.processJoystickInputs(yPercent.getAsDouble()) * DriveConstants.maxSpeedWhileShootingMPerS;
    double omegaRadPerS = omegaProcessor.processJoystickInputs(omegaPercent.getAsDouble()) * DriveConstants.maxAngularSpeedRadPerS;

    //Convert to field relative speeds
    ChassisSpeeds targetSpeeds = fieldRelative
      ? ChassisSpeeds.fromFieldRelativeSpeeds(xMPerS, yMPerS, omegaRadPerS, RobotState.getInstance().getLatestFieldToVehicle().getRotation()) 
      : new ChassisSpeeds(xMPerS, yMPerS, omegaRadPerS);

    drive.setGoalChassisSpeeds(targetSpeeds);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    RobotState.getInstance().setLookAhead(false);
    gamepad.setRumble(RumbleType.kLeftRumble, 0);
    gamepad.setRumble(RumbleType.kRightRumble, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }

  public static class AxisProcessor {
    private TrapezoidProfile.State state = new TrapezoidProfile.State();
    private static final double deadband = DriveConstants.driveJoystickDeadbandPercent;
    private final boolean square;

    public AxisProcessor(boolean square) {
      this.square = square;
    }

    public void reset(double value) {
      state = new TrapezoidProfile.State(value, 0.0);
    }

    //If joystick input exceeds deadbands, 
    public double processJoystickInputs(double value) {
      double scaledValue = 0.0;
      if (Math.abs(value) > deadband) {
        //Joystick input that starts after deadband as ratio of total possible joystick inputs
        scaledValue = (Math.abs(value) - deadband) / (1 - deadband);
        //scaled value is squared
        if (square) {
          scaledValue = Math.copySign(scaledValue * scaledValue, value);
        } else {
          scaledValue = Math.copySign(scaledValue, value);
        }
      }
      TrapezoidProfile profile = new TrapezoidProfile(
          new TrapezoidProfile.Constraints(99999,
              DriveConstants.driveMaxJerk),
          new TrapezoidProfile.State(scaledValue, 0.0), state);
      //calculate velocity and position 0.02 seconds in the future
      state = profile.calculate(0.02);
      return state.position;
    }
  }
}
