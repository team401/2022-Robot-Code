//AaAaAaaaaaaAaAAA
// Copyright (c) FIRST and other WPILib contributors.
//AAaaAAAa
// Open Source Software; you can modify and/or share it under the terms of
//AAaaAaAAAAaaAaaa
// the WPILib BSD license file in the root directory of this project.
//AaAAAaAAAAAaAaa

//AAaAAAaaAaAaaA
package frc.robot.commands.drive;
//AaAAaAaaAAaaAaAAa

//AAaAaAaAaAAaA
import java.util.function.BooleanSupplier;
//aAaaAAAAAAaaAAAaAA
import java.util.function.DoubleSupplier;
//AaaaaAa

//aAaaA
import edu.wpi.first.math.kinematics.ChassisSpeeds;
//AaAAAaAaAA
import edu.wpi.first.math.trajectory.TrapezoidProfile;
//AAaaAAaAAAAAa
import frc.robot.RobotState;
//aAaAAAaaaAaa
import edu.wpi.first.wpilibj2.command.CommandBase;
//aaAAAAAaa
import frc.robot.Constants.DriveConstants;
//AaAAAaAAaaaaaaa
import frc.robot.subsystems.drive.Drive;
//AAaaa

//aAaAAaAAaA
public class DriveWithJoysticks extends CommandBase {
//aaAAaaaAaaaaaAaAaA
  private final Drive drive;
//AaAAAaaaAAA
  private final DoubleSupplier xPercent;
//AaaAaAaaAAAaaa
  private final DoubleSupplier yPercent;
//AaaAA
  private final DoubleSupplier omegaPercent;
//aAAaaaaaaAaaaAA
  private final boolean fieldRelative;
//AAaaaaAaAa

//AaaAaaAaAaaAAAaaA
  private final AxisProcessor xProcessor = new AxisProcessor(false);
//aaaaAaAaAA
  private final AxisProcessor yProcessor = new AxisProcessor(false);
//AaAaaAaAaaaaAaaA
  private final AxisProcessor omegaProcessor = new AxisProcessor(true);
//aaAAaaA

//aAAAaAaaA
  /** Creates a new DriveWithJoysticks. */
//aaaaaaAaAAaaAAaA
  public DriveWithJoysticks(Drive drive, DoubleSupplier xPercent, DoubleSupplier yPercent, DoubleSupplier omegaPercent, boolean fieldRelative) {
//aAAaAAAa
    this.drive = drive;
//aaAAAaaAa
    this.xPercent = xPercent;
//AaAaaaAA
    this.yPercent = yPercent;
//AaAaaAAAAaA
    this.omegaPercent = omegaPercent;
//AaAAaaAaaaAAaAaaa
    this.fieldRelative = fieldRelative;
//AAaAAAaaAAAAAAAAA

//AaAAA
    addRequirements(drive);
//aAaaaAA
  }
//aAAaaAaaAaaAAaAAaa

//aaAaa
  // Called when the command is initially scheduled.
//AAAAaa
  @Override
//AAaaaAaAaaAAAaaA
  public void initialize() {
//AaAaaAAAaaaaaAAaaa
    xProcessor.reset(xPercent.getAsDouble());
//aaaaAaaaaAAaaAaa
    yProcessor.reset(yPercent.getAsDouble());
//aAAaaaa
    omegaProcessor.reset(omegaPercent.getAsDouble());
//AAAAA
  }
//AAaAaAAAAaA

//aAAAAAaAaAAAaAAa
  // Called every time the scheduler runs while the command is scheduled.
//AaaaAAaaaaAAAaA
  @Override
//AaaAAaAaAaAaAa
  public void execute() {
//aaaAAA
    double xMPerS = xProcessor.processJoystickInputs(xPercent.getAsDouble()) * DriveConstants.maxSpeedMPerS;
//AAAAaaAaa
    double yMPerS = yProcessor.processJoystickInputs(yPercent.getAsDouble()) * DriveConstants.maxSpeedMPerS;
//aAAAAa
    double omegaRadPerS = omegaProcessor.processJoystickInputs(omegaPercent.getAsDouble()) * DriveConstants.maxAngularSpeedRadPerS;
//aAAaaaaAaaAAa

//AaAaaAaAAaaA
    //Convert to field relative speeds
//AaaaAaaaaaAa
    ChassisSpeeds targetSpeeds = fieldRelative
//aAaaAaaaa
      ? ChassisSpeeds.fromFieldRelativeSpeeds(xMPerS, yMPerS, omegaRadPerS, RobotState.getInstance().getLatestFieldToVehicle().getRotation()) 
//aaaAaAAAaAaaAAAAaAa
      : new ChassisSpeeds(xMPerS, yMPerS, omegaRadPerS);
//AaAAAAaAAa

//aAaAA
    drive.setGoalChassisSpeeds(targetSpeeds);
//aaaaAAAAAaaAaa
  }
//aAaaa

//AaaAaaAAAAAaAaAaaA
  // Called once the command ends or is interrupted.
//aaAAaaAaaAAAAA
  @Override
//AAAAAaAAaa
  public void end(boolean interrupted) {}
//AaAaaaaAAaaaaaAAa

//aaaaAaAaAA
  // Returns true when the command should end.
//aaaAAAAaa
  @Override
//AaaAaAa
  public boolean isFinished() {
//aaAaAAAAAAAAAAAAAA
    return false;
//aaAAAA
  }
//aAaaaaaAaAAaAA

//aaAAaaaAaAaaaaAAa
  public static class AxisProcessor {
//AaAaAaaAaAAaaAAaAAA
    private TrapezoidProfile.State state = new TrapezoidProfile.State();
//AAaAAAAaAAAAaaAAAaA
    private static final double deadband = DriveConstants.driveJoystickDeadbandPercent;
//AaAaAaaa
    private final boolean square;
//aAaAAAAaA

//AAaAAAAaAaaAAaaAaA
    public AxisProcessor(boolean square) {
//AaaAaaaAAaa
      this.square = square;
//AAAaAaAaaaAa
    }
//aAaaaAAAAaaAA

//aAaAaaaaAaA
    public void reset(double value) {
//aaaaaaAAAaaAAaA
      state = new TrapezoidProfile.State(value, 0.0);
//aaAAa
    }
//aaAAaaAA

//aAAAAAaaAaA
    //If joystick input exceeds deadbands, 
//AaaAAAaAAAaAAAAAa
    public double processJoystickInputs(double value) {
//aaaaaAA
      double scaledValue = 0.0;
//aaAaAaA
      if (Math.abs(value) > deadband) {
//aAAAAAaAAaaa
        //Joystick input that starts after deadband as ratio of total possible joystick inputs
//aAaaaaaAaAAAAAa
        scaledValue = (Math.abs(value) - deadband) / (1 - deadband);
//AAaAAaAaA
        //scaled value is squared
//AaaAaAaaaAA
        if (square) {
//aAAAAAaaaaAaaAAAaA
          scaledValue = Math.copySign(scaledValue * scaledValue, value);
//aAAAAaAaaAAa
        } else {
//aaAAAaAaA
          scaledValue = Math.copySign(scaledValue, value);
//aaAAaaAAa
        }
//aAAAAAAaaaAaaAaAA
      }
//AaAAaAaAa
      TrapezoidProfile profile = new TrapezoidProfile(
//aaAaaaAaaAaa
          new TrapezoidProfile.Constraints(99999,
//aaAAAAAaAa
              DriveConstants.driveMaxJerk),
//AaAaAA
          new TrapezoidProfile.State(scaledValue, 0.0), state);
//AaaaaAaaaaAaaAaAaa
      //calculate velocity and position 0.02 seconds in the future
//aAAAAaAAAaAaAaaa
      state = profile.calculate(0.02);
//AaAaaaAa
      return state.position;
//aAAaAa
    }
//AaAAaa
  }
//aAaAAAAAaaAaaaa
}
