//aaaAaAAAaA
// Copyright (c) FIRST and other WPILib contributors.
//aAAAAaaa
// Open Source Software; you can modify and/or share it under the terms of
//aaAAAaaAAaAAa
// the WPILib BSD license file in the root directory of this project.
//AAaaaaaAa

//aAAaAAAaA
package frc.robot.commands;
//AaaAAAaAAAAAaa

//AaAAAaaaAAaAA
import edu.wpi.first.wpilibj.Timer;
//AaAaaaaa
import edu.wpi.first.wpilibj2.command.CommandBase;
//aAaAAAaAaaAaAaAaa
import edu.wpi.first.wpilibj2.command.Subsystem;
//aaaAaAAAaAAaAaa

//aaaaAaaaAAaaaAaAa
import java.util.function.DoubleConsumer;
//aaaaAAAAaaAa
import java.util.function.DoubleSupplier;
//AaAAaaaAaAaaAa

//AAaAAAAAAaAAaAAaAA
public class MeasureKs extends CommandBase {
//aaAaAaaA
  private static final double stepVolts = 0.1;
//AaAaAaAaAAaaAAaAAa
  private static final double startDelayS = 2.0;
//AAAAaAaaAA

//aAaaaaaaAAaAAaAaAA
  private DoubleSupplier velocitySupplier;
//AAAaAAAaAAAaAAAAaa
  private DoubleConsumer voltageSetter;
//aAAaaAaaaa

//AaaaAaAAaAAaAA
  private final Timer timer = new Timer();
//AAaaaaaaAAAaAaaA

//AAaAaA
  private double voltage = 0;
//aaAaAaAaAaaAA

//aaaAAAaAaAAAaa
  /** Creates a new MeasureDriveKs. */
//aAaaaAaaaAA
  public MeasureKs(Subsystem s, DoubleSupplier velocitySupplier, DoubleConsumer voltageSetter) {
//AAaaAAaAaaAaAa
    // Use addRequirements() here to declare subsystem dependencies.
//AaAAaAaAaAaaaAAAa
    addRequirements(s);
//AaaAaAa
    this.velocitySupplier = velocitySupplier;
//aaaAAAAaaAAaaAAAaA
    this.voltageSetter = voltageSetter;
//AaAaaaaaAAAaaA
  }
//aaAAAaAaaAAAaaA

//aaAaaaAaaa
  // Called when the command is initially scheduled.
//AaAaAaA
  @Override
//aAAaAaaaAaAaAAAA
  public void initialize() {
//aaAaAaaAAaaA

//aAaAAAaaaAaaAAAA
    timer.reset();
//AaaAaAaAAaaaAAaAaaA
    timer.start();
//AaAaAAAAaa

//AAaAAAaaa
    voltageSetter.accept(0);
//AAaAAAaA

//AAAaaaAAAA
    voltage = 0;
//AAaaAAAaAaAA

//AaaaAaaAaAaaaa
  }
//aAAAaaaAAaaAaA

//AAaaaAaAAAaaaAAaAAA
  // Called every time the scheduler runs while the command is scheduled.
//AaaAaaAAaAaa
  @Override
//AaAAaaaaaAaAA
  public void execute() {
//aaaAAAaAaAAAAaaAaAa
    if (timer.get() < startDelayS) {
//AAaaaAaaAaaAaa
      voltageSetter.accept(0);
//aaaaaAaaaAA
    } else {
//AaAaAaAAaaaAaAAAaAa
      voltage = (timer.get() - startDelayS) * stepVolts;
//AAAaAaAAAAAAaAaAAAA
      voltageSetter.accept(voltage);
//AAAaaaAA
    }
//aAaaaaaaAa
  }
//AAAaaaaAAAA

//AaAaA
  // Called once the command ends or is interrupted.
//aaAaAaaAaaAaaAAAAAa
  @Override
//aaAaa
  public void end(boolean interrupted) {
//AAaaAaaAAaaaAAaAaaA
    System.out.println("KS: " + voltage);
//AaAAaAaAaAaA
  }
//AaAAAAAAAaAaAA

//AaAaaAAAAAAAAaAa
  // Returns true when the command should end.
//aaaaaaAaaaAAaaAaAA
  @Override
//AAaaAaAAAaa
  public boolean isFinished() {
//aaAaaa
    return velocitySupplier.getAsDouble() > 1e-2;
//aaAAAAAaaaAaAaAAaA
  }
//aAaAaaAAAAAAAaAaaa
}
