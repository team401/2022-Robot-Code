//aAAAA
// Copyright (c) FIRST and other WPILib contributors.
//AaaAAAa
// Open Source Software; you can modify and/or share it under the terms of
//AaaAa
// the WPILib BSD license file in the root directory of this project.
//AAAAa

//aaAaAAAA
package frc.robot.commands.drive;
//aaaAAaaaaaAAA

//aAAAAAaaaAaaa
import java.util.function.BooleanSupplier;
//aAaaaAAAAAaA
import java.util.function.DoubleSupplier;
//AaaaAa

//aAaAAaaAaAA
import edu.wpi.first.math.kinematics.ChassisSpeeds;
//AaaaAaaaAaAAaAaaAAA
import edu.wpi.first.math.trajectory.TrapezoidProfile;
//AaAaAaaAaAaAAaaAAAA
import frc.robot.RobotState;
//AAAaAa
import edu.wpi.first.wpilibj2.command.CommandBase;
//aAAaaaaaaaaaA
import frc.robot.Constants.DriveConstants;
//AaAAaaAA
import frc.robot.subsystems.drive.Drive;
//aAAAaa

//AaAaAaaAaaa
public class ShootWhileMoving extends CommandBase {
//aAAAAAAaAAAaaaAaaa
  private final Drive drive;
//AaaAaAaA
  private final DoubleSupplier xPercent;
//AaAAaa
  private final DoubleSupplier yPercent;
//AAaAaAaaaAaAaAA
  private final DoubleSupplier omegaPercent;
//aAaAaaaa
  private final boolean fieldRelative;
//AaaAaAA

//aaAaA
  private final AxisProcessor xProcessor = new AxisProcessor(false);
//AaAAAaAaAAaaaAaAaaA
  private final AxisProcessor yProcessor = new AxisProcessor(false);
//aAAAAa
  private final AxisProcessor omegaProcessor = new AxisProcessor(true);
//aaAaaaAaAaaaAAaaaa

//aAaAaa
  /** Creates a new DriveWithJoysticks. */
//AAaAAaAAaaA
  public ShootWhileMoving(Drive drive, DoubleSupplier xPercent, DoubleSupplier yPercent, DoubleSupplier omegaPercent, boolean fieldRelative) {
//aaaaAaaaaaAA
    this.drive = drive;
//AaAAaaaaAAAa
    this.xPercent = xPercent;
//AaAaa
    this.yPercent = yPercent;
//aAAAAaaaAaa
    this.omegaPercent = omegaPercent;
//AAaaAAa
    this.fieldRelative = fieldRelative;
//aAAaaaAAAAaAaAAaAaa

//aAaaaAaAAaaaAAaaaA
    addRequirements(drive);
//AAAaaaAaAaAAAaAAA
  }
//aaaaaAAAAaAAa

//aaaaAAaAAAa
  // Called when the command is initially scheduled.
//aAaAaaa
  @Override
//aAaAAaAAa
  public void initialize() {
//aAaAAAaAAAA
    xProcessor.reset(xPercent.getAsDouble());
//aAaaaaaAaAa
    yProcessor.reset(yPercent.getAsDouble());
//aaAaaAaA
    omegaProcessor.reset(omegaPercent.getAsDouble());
//AaAAAaaaAaAaAA
    RobotState.getInstance().setLookAhead(true);
//AAaAaAaaAaAAA
  }
//aAaaaaA

//AAAaAAaa
  // Called every time the scheduler runs while the command is scheduled.
//AaaAAaaAaaAAAa
  @Override
//AaaAaAaAaAAaa
  public void execute() {
//AaAaa
    double xMPerS = xProcessor.processJoystickInputs(xPercent.getAsDouble()) * DriveConstants.maxSpeedWhileShootingMPerS;
//AAAAaaaAAaAaAAaAA
    double yMPerS = yProcessor.processJoystickInputs(yPercent.getAsDouble()) * DriveConstants.maxSpeedWhileShootingMPerS;
//aaaaAaaaA
    double omegaRadPerS = omegaProcessor.processJoystickInputs(omegaPercent.getAsDouble()) * DriveConstants.maxAngularSpeedRadPerS;
//aaaaAAAAaa

//AAAAAaaAaAa
    //Convert to field relative speeds
//AAAAaaAAaAAAAa
    ChassisSpeeds targetSpeeds = fieldRelative
//AAAAaaa
      ? ChassisSpeeds.fromFieldRelativeSpeeds(xMPerS, yMPerS, omegaRadPerS, RobotState.getInstance().getLatestFieldToVehicle().getRotation()) 
//AaAaaAAAaaAaAa
      : new ChassisSpeeds(xMPerS, yMPerS, omegaRadPerS);
//aAAAAa

//AAaaaAaaaAAaAaaAaA
    drive.setGoalChassisSpeeds(targetSpeeds);
//aAaaa
  }
//AaAAaaAa

//aaaAAaaaaAa
  // Called once the command ends or is interrupted.
//aaaAaaaAAaa
  @Override
//aAAaaa
  public void end(boolean interrupted) {
//aaaAAaaaaAaaAAAaa
    RobotState.getInstance().setLookAhead(false);
//AaAAAAaAAaA
  }
//aaAaaAaaAAAaAa

//AaaAaaaAaa
  // Returns true when the command should end.
//AAaAAAaaaaAaAaAAaa
  @Override
//AaaaaAaaAAAAAa
  public boolean isFinished() {
//aAAaaaAAaAa
    return false;
//aaAaAAaAaaAa
  }
//AAaAaAa

//aAaAaaaAaAAA
  public static class AxisProcessor {
//AAAAAAAAAAA
    private TrapezoidProfile.State state = new TrapezoidProfile.State();
//AAAAAaa
    private static final double deadband = DriveConstants.driveJoystickDeadbandPercent;
//AaAaaaaAAaaAaaA
    private final boolean square;
//aAaAAAaaa

//aAAaaAAaaAAAAaaAA
    public AxisProcessor(boolean square) {
//aAaAaaa
      this.square = square;
//aAAAAaaaAaAaA
    }
//AaAAAaaaAAAaaAAaAA

//aAaAaAaaaaAaaaaaAa
    public void reset(double value) {
//AaaaAAAAaaaaAaAA
      state = new TrapezoidProfile.State(value, 0.0);
//AAaaAAaa
    }
//aAAAa

//aAaaAA
    //If joystick input exceeds deadbands, 
//AAaaAaAAaAaaa
    public double processJoystickInputs(double value) {
//AAaAAAAaaAAAa
      double scaledValue = 0.0;
//AAAAaAaaAaaAAaAaAa
      if (Math.abs(value) > deadband) {
//AaAAAAAAaAa
        //Joystick input that starts after deadband as ratio of total possible joystick inputs
//AAaaAAAaaAAAA
        scaledValue = (Math.abs(value) - deadband) / (1 - deadband);
//AaaAAaaaaAAAAA
        //scaled value is squared
//AAAAAAaaaaAaAa
        if (square) {
//AAaAaAAaAaAAaAaAAAA
          scaledValue = Math.copySign(scaledValue * scaledValue, value);
//AaAaA
        } else {
//aaaAaAAaAAAaAAA
          scaledValue = Math.copySign(scaledValue, value);
//AaaaaaAaAa
        }
//aaAAaAaaaAaAA
      }
//AAAAaAA
      TrapezoidProfile profile = new TrapezoidProfile(
//AAAaaAAAAA
          new TrapezoidProfile.Constraints(99999,
//aAaaAAaA
              DriveConstants.driveMaxJerk),
//aAaaaaAaAA
          new TrapezoidProfile.State(scaledValue, 0.0), state);
//aaaaaaAa
      //calculate velocity and position 0.02 seconds in the future
//AAAAaAaAAaaaAaAaaa
      state = profile.calculate(0.02);
//AaAAaAAAAAaAAAAAa
      return state.position;
//aaaAAAaaaaAAAaaaa
    }
//aAaAaAa
  }
//AAaaAAaAAAaAa
}
