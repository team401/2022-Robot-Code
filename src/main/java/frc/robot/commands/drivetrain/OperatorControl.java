package frc.robot.commands.drivetrain;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.DriveSubsystem;

public class OperatorControl extends CommandBase {
    
    //Drive Input Command in Teleop

    private final DriveSubsystem drive;

    /**
     * Double Suppliers are initialized from the Joystick (so that they will continously update rather 
     * than just sending in one value)
     */
    private final DoubleSupplier forwardX;
    private final DoubleSupplier forwardY;
    private final DoubleSupplier rotation;

    //says if the given values are relative to the field or the front of the robot
    private final boolean isFieldRelative;

    //tolerance for deadbanding inputs
    private final double toleranceDeadband = 0.035;

    /**
     * Constructor that takes in the subsytem, the three double suppliers of the values we want to 
     * send to the drive, and whether or not those values are relative to the field
     */
    public OperatorControl(
        DriveSubsystem subsystem, 
        DoubleSupplier fwdX, 
        DoubleSupplier fwdY, 
        DoubleSupplier rot,
        boolean fieldRelative
        ) 
    {
        //initializes values of our subsystem and Double Suppliers
        drive = subsystem;
        forwardX = fwdX;
        forwardY = fwdY;
        rotation = rot;
        isFieldRelative = fieldRelative;

        /**
         * Adding requirements basically reports which subsystems are used in each command
         * In this case, it is the drive subsystem
         */

        addRequirements(subsystem);
    }

	//body of the command, runs every 20 ms until the end condition is met
    @Override
    public void execute() {

        /**
         * Units are in meters per second (for linear motion) and radians per second (for rotation)
         * Since the joysticks go from -1 to 1, we have to multiply the outputs by the max drive speed
         * (Otherwise our max speed will always be 1 m/s and 1 rad/s)
         * Math.copySign just makes sure to preserve the sign of a passed variable when we do functions on
         * it that would normally get rid of the sign (such as squaring)
         */

        double fwdX = forwardX.getAsDouble();
        fwdX = Math.copySign(fwdX, fwdX); //technically optional?
        fwdX = deadbandInputs(fwdX) * DriveConstants.maxDriveSpeed;

        double fwdY = forwardY.getAsDouble();
        fwdY = Math.copySign(fwdY, fwdY); //technically optional?
        fwdY = deadbandInputs(fwdY) * Units.feetToMeters(DriveConstants.maxDriveSpeed);

        double rot = rotation.getAsDouble();
        rot = Math.copySign(rot*rot, rot);
        rot = deadbandInputs(rot) * Units.degreesToRadians(DriveConstants.teleopTurnRateDegPerSec);

        drive.drive(
            fwdX,
            -fwdY,
            -rot,
            isFieldRelative
        );

    }

    /**Deadbands inputs to make sure accidental movements do not cause any motion 
     * Basically just checks to make sure the joystick value is actually over a certain tolerance
     * Usually, it is some value under 0.05
    */
    public double deadbandInputs(double input) {

        if (Math.abs(input) < toleranceDeadband) return 0.0;
        return input;

    }

}
