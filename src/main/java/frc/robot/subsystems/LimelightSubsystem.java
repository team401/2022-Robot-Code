package frc.robot.subsystems;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.LimelightConstants;

public class LimelightSubsystem extends SubsystemBase {

    /**
     * Holds the limelight info from the Limelight
     * The information is pulled from a network table, which is updated continuously through the match and
     * can be read from
     */

    private final NetworkTable table;

    /**
     * Values we read from the Limelight:
     * tV - tells if there is a target or not (0 - none, 1 - seen)
     * tX - offset in degrees from target center to center of Limelight's limelight (horizontally)
     * tY - offset in degrees from target center to center of Limelight's limelight (vertically)
     * tA - percentage of the view taken up by the target 
     */

     private double tv;
     private double tx;
     private double ty;
     private double ta;

     private double centeredTolerance = Units.degreesToRadians(5);

     //Constructor
     public LimelightSubsystem() {

        //gets the instance of the limelight from the network tables and gets the pipeline configuration of 0
        table = NetworkTableInstance.getDefault().getTable("limelight");
        NetworkTableInstance.getDefault().getTable("limelight").getEntry("pipeline").setNumber(0);

        //turns the leds off (1 is off, 0 is on)
        setLedMode(1);

     }

     @Override
     public void periodic() {

        //updates the values from reading the table, and if null, sets it to 0.0
        tv = table.getEntry("tv").getDouble(0.0);
        tx = table.getEntry("tx").getDouble(0.0);
        ty = table.getEntry("ty").getDouble(0.0);
        ta = table.getEntry("ta").getDouble(0.0);

        //Output to SmartDashboard
        SmartDashboard.putNumber("tY (Vertical Error)", ty);

     }

     //if tV is greater than 0, it has a target (since it is either 0 or 1)
     public boolean hasValidTarget() {

        return tv > 0;

     }
    
     //returns in radians since all of our math is done in radians usually
     public double getX() {

        return Units.degreesToRadians(tx);

     }

     //returns in radians
     public double getY() {

        return Units.degreesToRadians(ty);

     }

     //returns a double representing the area of the target in percent of frame
     public double getA() {

        return ta;

     }

     //turns the leds on (0) or off (1)
     public void setLedMode(int ledMode) {

        table.getEntry("ledMode").forceSetDouble(ledMode);

     }

     //returns whether the turret is locked on within the tolerance 
     public boolean withinTolerance() {

        return Math.abs(getX()) < centeredTolerance;

     }

     public double calculateDistance(double angle, double height) {

      return height / Math.sin( LimelightConstants.angle + Units.degreesToRadians(angle)) + LimelightConstants.distanceOffset;
     }

}
