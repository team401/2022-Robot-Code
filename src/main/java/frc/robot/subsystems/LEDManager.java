//AaAaaaaaaaAA
package frc.robot.subsystems;
//aAaAaaAaaAA

//aAAaAa
import edu.wpi.first.wpilibj.AddressableLED;
//aaaaaaAaaaa
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
//aAAaaAaAaAAAAaAaA
import edu.wpi.first.wpilibj.DriverStation;
//AaaAAAAAAaAaaAAaAa
import edu.wpi.first.wpilibj.Timer;
//AaAAAaaaAAaAaa
import edu.wpi.first.wpilibj.DriverStation.Alliance;
//AAAaaaaAaA
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
//aAaAaAAAaAAAAaAaAAA
import edu.wpi.first.wpilibj.util.Color;
//aAAAaAAaaAaaa
import edu.wpi.first.wpilibj2.command.SubsystemBase;
//AAAaaaaaaAaAaa
import frc.robot.RobotState;
//aAaAA
import frc.robot.subsystems.Shooter;
//AAaAaaAaAA
import frc.robot.subsystems.Tower;
//AaaAAAAAaAAAaaaaAaa
import frc.robot.subsystems.Vision;
//AAAAAa

//aAAAAAAaAAaA
public class LEDManager extends SubsystemBase {
//AaaaaaA

//AAaAaAAAAaaaAAA
    private static boolean error = false;
//AaaaA

//aAaAAaAAaa
    private AddressableLED led;
//aAaaaAaaAAaaA

//aAAAaAaaa
    private AddressableLEDBuffer buffer;
//aaaAaAAAaaAAaAaAAAa

//aAAAAaAaAaaa
    // left first
//AAAaaa
    private final int ledCountPerSide = 34;
//AAAAaaaAaAaaaAaAAA
    private final int ledArmCount = 22;
//AaaaA

//AaAAAaAaAaAAa
    private int rainbowFirstPixelHue = 0;
//aaAaAaAAA

//aaaaAAAAAAaAAA
    private boolean climbing = false;
//AAaaaAaA

//aaAAaaAaaAAaaa
    public LEDManager() {
//aAAAaaaAaaa

//aaaAAaAaAaAaAaAaAaA
        led = new AddressableLED(9);
//AaAaAAAAAaAAAaaA
        buffer = new AddressableLEDBuffer(ledCountPerSide*2);
//AaaaAaaAAAAa

//AAaaaAaaAAAaaa
        led.setLength(buffer.getLength());
//AaAAaA
        led.setData(buffer);
//AaaAAAAAaaaaAaaAa

//aaaAAAa
        led.start();
//aAAaAa

//aaAaAAAa
        climbLeds = new boolean[ledArmCount];
//AAAAAAaAaaaaAaa

//aAaaaaAAA
    }
//AaAaAAaaAaa

//aaAAAAaAAAAaAAAa
    @Override
//aaAAaaAAAaaAAaAAaa
    public void periodic() {
//aaaaaaAAAAaaA

//aAaAaAAaAaA
        // Blank buffers
//AaAaAaaAaaAAAAA
        for (int i = 0; i < buffer.getLength(); i++)
//AAaaaaa
            buffer.setRGB(i, 0, 0, 0);
//AAAAaaaaAaAaaAa

//AAAaaa
        /*if (DriverStation.isEnabled())
//AaAAaAaAAaaAAAAa
            if (!climbing)
//AAAAAAaa
                updateStrips();
//AaAAAaaaaaaa
            else
//aAaaAAAaaAaAA
                climb();
//AAAaaaAAAaaaaaaAAa
        else
//AaaAa
            rainbow();*/
//aaaAAaAaAaaaAAa

//AaAAaaaaAAaAaaAA
        if (climbing && DriverStation.isEnabled())
//AAaAA
            climb();
//AAaaa
        else
//aaaAaAAAA
            rainbow();
//AAaAAAaAaaaAaaAAAaa
        
//AaaaaaaAaAaaAaAAaa
        led.setData(buffer);
//AAAAaaaAaAAAAaaaa

//AAaAaaAaAAaaaaAa
    }
//AAaaAAaAAAAAaaaAaaA

//aaAAaAaaaaaaa
    private void updateStrips() {
//aaaaA

//AaaaAaAAAaa
        double deg = RobotState.getInstance().getLatestFieldToVehicle().getRotation().getDegrees();
//aaaaAaaAaaaAaAAaA

//aaaaAAAAAa
        if (deg > 67.5 || deg < -112.5) // Left front
//AaAAAA
            updateStrip(buffer, ledArmCount+ledCountPerSide);
//aAAAaaAAAAa
		if (deg > 22.5 || deg < -157.5) // Left mid front
//aAAAAAAaaaAaAaaA
            updateStrip(buffer, ledArmCount+ledCountPerSide+3);
//AaAAaaaAaAaAAAa
		if (deg > -22.5 && deg < 157.5) // Left mid back
//AAaaAaAaaaAAaA
            updateStrip(buffer, ledArmCount+ledCountPerSide+6);
//AAAAaAAa
		if (deg > -67.5 && deg < 112.5) // Left back
//aaAaAaAaaaaAAAaAAa
            updateStrip(buffer, ledArmCount+ledCountPerSide+9);
//aAAAa
        
//AAAAaa
		if (deg > 112.5 || deg < -67.5) // Right front
//aAAaaAAAaaAAaaAa
            updateStrip(buffer, ledArmCount);
//AAAaaAaAaAAaaAA
		if (deg > 157.5 || deg < -22.5) // Right mid front
//aaAaaAaAaa
            updateStrip(buffer, ledArmCount+3);
//AAaAaAAAaaAaaAaAA
		if (deg > -157.5 && deg < 22.5) // Right mid back
//aaAAAAAaAAa
            updateStrip(buffer, ledArmCount+6);
//aaaAAAAAaaaAaaA
		if (deg > -112.5 && deg < 67.5) // Right back
//aaaaAAAaaAAAaaaaA
            updateStrip(buffer, ledArmCount+9);
//AAAaa

//aaaAAa
        if (Math.abs(Vision.getTX()) < 5 && Shooter.atGoalStatic()) {
//AAaAaaAaaAAAaaAaaaA
            if (deg < 0) {
//aaaAaaAaAaaaaAA
                for (int i = 0; i < ledArmCount; i++)
//aAAAA
                    buffer.setRGB(i, 50, 50, 50);
//AaaAaAAaaAAAa
            }
//aAAaaaAaAAAaaAAaA
            else {
//AAaaaaaaaaAAaAAaaaa
                for (int i = ledCountPerSide; i < ledArmCount+ledCountPerSide; i++)
//aAAAA
                    buffer.setRGB(i, 50, 50, 50);
//aaaAAaaaaAaa
            }
//AaaAaAAAaA
        }
//aAaaAA

//aAaaAaaa
    }
//AaaaaaAAaAAAaAAaaa

//AAAaAaaaAaaa
    private void updateStrip(AddressableLEDBuffer buffer, int offset) {
//AAAAAaaaaAaaAAaaAaA

//aAaAAAAaaAAAAAaaAa
        /**
//aaAAAaaaa
         * locked and revved - white
//aaaaaA
         * two correct balls - green
//aAAaaAAaaaaAaAaa
         * one correct balls - yellow
//AAAaAAaaaAAaaAaaaA
         * has incorrect ball - red
//aAAAaAaAaa
         * error - random
//AaaAAaAAAaaAA
         */
//aaAAAaaa

//aaaaaaAaaAAAAaAaAa
        //int topBall = BallType.toByte(Tower.getTopBall());
//aAAaAaaa
        //int bottomBall = BallType.toByte(Tower.getBottomBall());
//AaAaaaaAaA
        boolean readyToShoot = Math.abs(Vision.getTX()) < 5 && Shooter.atGoalStatic();
//AAAAAaAAAAAa
        int alliance = DriverStation.getAlliance() == Alliance.Blue ? 1 : 2;
//AaAAaAaAAaA

//AAAaAAaAAa
        /*Color color = Color.kBlack;
//aaAaAAaAaaaaA

//aaaAaAAaa
        if (readyToShoot) 
//aAaaAAAaA
            color = Color.kWhite;
//aaaaaaAAAAAAA
        else if (topBall == bottomBall && topBall == alliance)  
//AaaAA
            color = Color.kGreen;
//AAAaaAAAAAA
        else if (topBall == alliance) 
//AAaaAAAaaaAa
            color = Color.kYellow;
//AAAaaaa
        else if ((topBall != 0 && topBall != alliance) || (bottomBall != 0 && bottomBall != alliance))
//aaaaAaaAAaaaaA
            color = Color.kRed;
//aAAAAAaaAaaaAa
        else if (error)
//aaaaAaAaAaAAAaaaaA
            color = new Color((int)(Math.random()*256), (int)(Math.random()*256), (int)(Math.random()*256));
//aaaAaaAaaAaa

//AaAaaaaaAaaAaA
        for (int i = offset; i < offset+3; i++)
//AaaAaAaaAAaaAa
            buffer.setLED(i, color);*/
//AAaAAaAaaaa

//AAAaA
        if (readyToShoot) {
//aaAAAAaAAa
            for (int i = offset; i < offset+3; i++)
//aAaAaAaAaAaaAAAAAa
                buffer.setRGB(i, 50, 50, 50);
//AaaaAAaaaA
        }
//AAAaAaaAaAAaAAaaAAA

//aAAAAaAAAAaA
    }
//aAAaaAAAAaAaaAaaA

//aAAAaAAaAaAa
    private void rainbow() {
//AaaaAaAa
        for (int i = 0; i < buffer.getLength(); i++) {
//AaaaaA
            int hue = (rainbowFirstPixelHue + 90 + (i * 180 / ledArmCount)) % 180;
//AaaaAAAaa
            buffer.setHSV(i, hue, 255, 128);
//AAaaaaaAaAAaAAaaaaa
        }
//AaAAAaAAAAAaaa
        rainbowFirstPixelHue += 3;
//aaaAa
        rainbowFirstPixelHue %= 180;
//aaaAaAaaaAaAAaaaAAA

//aaaAaaaAaAAA
        for (int i = 0; i < buffer.getLength(); i++) {
//aAAAaAaaAAaAaaaa
            Color color = buffer.getLED(i);
//AAAAAAAAaaaAAaA
            buffer.setRGB(i, (int)(color.red*50), (int)(color.green*50), (int)(color.blue*50));
//AaaAAaAA
        }
//AAaaAaAAaAAaAa
    }
//aaAaAaaa

//aaAaaAaAAAa
    private final int climbCycleCount = 2;
//aAAaaaAaAa
    private final int maxDelay = 30;
//AaaaaAaaaaAAaaAa
    private final int minDelay = 10;
//AaAAaaAAA
    private final int maxLength = 6;
//AAAaA
    private final int minLength = 1;
//aaaAAAaAAAaAAaaA

//AAAaaAaaAa
    private final boolean[] climbLeds;
//AaaAaaAA

//AaAaAAAaaaaaAAaa
    private int cycleSpawn = minDelay;
//AaAaaAaAAAa
    private int cycleUpdate = climbCycleCount;
//aAaAaaaaaAaaAA

//AAAAAaAAAaaAAaA
    private void climb() {
//AaAAAAaAaAaAaaAAA

//AAAaAAAAAaAAA
        // Spawn
//aAaAaaAAA
        cycleSpawn--;
//aaAAaAaAAaaaAaA
        if (cycleSpawn == 0) {
//aAaaAa
            int length = (int)(Math.random()*(maxLength-minLength)+minLength);
//AAAaaAAaAA
            cycleSpawn = (int)(Math.random()*(maxDelay-minDelay)+minDelay) + (int)(length*2);
//aaaaAAaaAaaaAAAaaAa
            for (int i = 0; i < length && i < climbLeds.length; i++)
//aaAaAa
                climbLeds[i] = true;
//AAaaAaaAaAAa
        }
//AaAaaAA

//AaAaAaAAAAaaaAAaa
        // Update
//aAAAAaAAa
        cycleUpdate--;
//aaaAAaAAAAa
        if (cycleUpdate == 0) {
//aaAaAAAAAAaAAAAaAAA
            cycleUpdate = climbCycleCount;
//AaaaAaAaaAaAAaAaA
            for (int i = climbLeds.length-2; i > 0; i--)
//AAaAaAAaAaaa
                climbLeds[i] = climbLeds[i-1];
//aAaaaAAaaA
            climbLeds[0] = false;
//aAaaAaAAaAA
        }
//aAaAAA

//aaAAAAa
        // Buffer
//aAAAa
        Color color = DriverStation.getAlliance() == Alliance.Red ? new Color(50, 0, 0) : new Color(0, 0, 50);
//AaAaa
        for (int i = 0; i < ledArmCount; i++) {
//aAAAaaaaAaaA
            buffer.setLED(i, climbLeds[i] ? color : Color.kBlack);
//AaaAAaaaAA
            buffer.setLED(i+ledCountPerSide, climbLeds[i] ? color : Color.kBlack);
//AaAAaaaa
        }
//aAaaaaAA

//aaAAAa
    }
//AAaaaAa

//aaAAAAaAaaaAAa
    public static void setError(boolean e) {
//aaAaAAaaaAA
        error = e;
//aaaaaAAAAaaAAAaaaAA
    }
//aAAAaaaAAA

//AaaaaAAaAAaaAaaaAA
    public void setClimb(boolean v) {
//AaAAAaaAaAAaAA
        climbing = v;
//aaAAaAAaA
    }
//aAaAAAAaaA
    
//AAAAAAAaaAAaAaA
}