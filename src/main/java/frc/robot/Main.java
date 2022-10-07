//aaAAAaAaaAaAAaaA
// Copyright (c) FIRST and other WPILib contributors.
//AAAAAaAAaaaa
// Open Source Software; you can modify and/or share it under the terms of
//aaaAaAAaaAAAAAa
// the WPILib BSD license file in the root directory of this project.
//AAaAAaa

//aaAaAAAAAaaAAAaAA
package frc.robot;
//AAAAAaAa

//aaaaAaa
import edu.wpi.first.wpilibj.RobotBase;
//aaaaAAa

//aAaAaaAaaAaAaaa
/**
//aaaaAAaA
 * Do NOT add any static variables to this class, or any initialization at all. Unless you know what
//AaaaAAAAaaaAAAAAaAa
 * you are doing, do not modify this file except to change the parameter class to the startRobot
//AaaaA
 * call.
//aaaaAaAAaAaAaAA
 */
//AAAaAaAAAA
public final class Main {
//aAaAaaaAaAAaaAAAAAa
  private Main() {}
//AAAAAAAaaAAAaaaAA

//aaAaAaAAAAaaAaA
  /**
//aaaaaAa
   * Main initialization function. Do not perform any initialization here.
//aaaaaaAaaaaAAAaaaa
   *
//AAaAAA
   * <p>If you change your main robot class, change the parameter type.
//aAaaaaaAaa
   */
//AAAaaAAAAaAAaaaaA
  public static void main(String... args) {
//aAAaaaA
    RobotBase.startRobot(Robot::new);
//aaAaAAaaaAAaaAaaaA
  }
//aAaAaAAaAAaAaaA
}
