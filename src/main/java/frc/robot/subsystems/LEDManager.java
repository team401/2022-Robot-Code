package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotState;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Tower;
import frc.robot.subsystems.Vision;

public class LEDManager extends SubsystemBase {

    private static boolean error = false;

    private AddressableLED led;

    private AddressableLEDBuffer buffer;

    // left first
    private final int ledCountPerSide = 34;
    private final int ledArmCount = 22;

    private int rainbowFirstPixelHue = 0;

    private boolean climbing = false;

    public LEDManager() {

        led = new AddressableLED(9);
        buffer = new AddressableLEDBuffer(ledCountPerSide*2);

        led.setLength(buffer.getLength());
        led.setData(buffer);

        led.start();

        climbLeds = new boolean[ledArmCount];

    }

    @Override
    public void periodic() {

        // Blank buffers
        for (int i = 0; i < buffer.getLength(); i++)
            buffer.setRGB(i, 0, 0, 0);

        if (climbing) {
            climb();
        }
        else {
            if (DriverStation.isTeleop() || DriverStation.isDisabled())
                rainbow();
            else
                charge();
            
            if (DriverStation.isEnabled())
                updateStrips();
        }
        
        led.setData(buffer);

    }

    private void updateStrips() {

        double deg = RobotState.getInstance().getLatestFieldToVehicle().getRotation().getDegrees();

        if (deg > 67.5 || deg < -112.5) // Left front
            updateStrip(buffer, ledArmCount+ledCountPerSide);
		if (deg > 22.5 || deg < -157.5) // Left mid front
            updateStrip(buffer, ledArmCount+ledCountPerSide+3);
		if (deg > -22.5 && deg < 157.5) // Left mid back
            updateStrip(buffer, ledArmCount+ledCountPerSide+6);
		if (deg > -67.5 && deg < 112.5) // Left back
            updateStrip(buffer, ledArmCount+ledCountPerSide+9);
        
		if (deg > 112.5 || deg < -67.5) // Right front
            updateStrip(buffer, ledArmCount);
		if (deg > 157.5 || deg < -22.5) // Right mid front
            updateStrip(buffer, ledArmCount+3);
		if (deg > -157.5 && deg < 22.5) // Right mid back
            updateStrip(buffer, ledArmCount+6);
		if (deg > -112.5 && deg < 67.5) // Right back
            updateStrip(buffer, ledArmCount+9);

        if (Math.abs(Vision.getTX()) < 5 && Shooter.atGoalStatic()) {
            if (deg < 0) {
                for (int i = 0; i < ledArmCount; i++)
                    buffer.setRGB(i, 50, 50, 50);
            }
            else {
                for (int i = ledCountPerSide; i < ledArmCount+ledCountPerSide; i++)
                    buffer.setRGB(i, 50, 50, 50);
            }
        }

    }

    private void updateStrip(AddressableLEDBuffer buffer, int offset) {

        boolean readyToShoot = Math.abs(Vision.getTX()) < 5 && Shooter.atGoalStatic();

        if (readyToShoot) {
            for (int i = offset; i < offset+3; i++)
                buffer.setRGB(i, 50, 50, 50);
        }

    }

    private void rainbow() {
        for (int i = 0; i < buffer.getLength(); i++) {
            int hue = (rainbowFirstPixelHue + 90 + (i * 180 / ledArmCount)) % 180;
            buffer.setHSV(i, hue, 255, 128);
        }
        
        if (DriverStation.isEnabled())
            rainbowFirstPixelHue += 3;
        rainbowFirstPixelHue %= 180;

        for (int i = 0; i < buffer.getLength(); i++) {
            Color color = buffer.getLED(i);
            buffer.setRGB(i, (int)(color.red*50), (int)(color.green*50), (int)(color.blue*50));
        }
    }

    private void charge() {
        int ledNum = (int)((15-DriverStation.getMatchTime()) / 15 * (ledArmCount+1))+1;
        int r = DriverStation.getAlliance() == DriverStation.Alliance.Red ? 255 : 0;
        int b = DriverStation.getAlliance() == DriverStation.Alliance.Blue ? 255 : 0;
        for (int i = 0; i < ledNum && i < ledArmCount; i++) {
            buffer.setRGB(i, r, 0, b);
        }
        if (ledNum == ledArmCount+1)
        {
            for (int i = ledArmCount; i < ledCountPerSide; i++) {
                buffer.setRGB(i, r, 0, b);
            }
        }
    }

    private final int climbCycleCount = 2;
    private final int maxDelay = 30;
    private final int minDelay = 10;
    private final int maxLength = 6;
    private final int minLength = 1;

    private final boolean[] climbLeds;

    private int cycleSpawn = minDelay;
    private int cycleUpdate = climbCycleCount;

    private void climb() {

        // Spawn
        cycleSpawn--;
        if (cycleSpawn == 0) {
            int length = (int)(Math.random()*(maxLength-minLength)+minLength);
            cycleSpawn = (int)(Math.random()*(maxDelay-minDelay)+minDelay) + (int)(length*2);
            for (int i = 0; i < length && i < climbLeds.length; i++)
                climbLeds[i] = true;
        }

        // Update
        cycleUpdate--;
        if (cycleUpdate == 0) {
            cycleUpdate = climbCycleCount;
            for (int i = climbLeds.length-2; i > 0; i--)
                climbLeds[i] = climbLeds[i-1];
            climbLeds[0] = false;
        }

        // Buffer
        Color color = DriverStation.getAlliance() == Alliance.Red ? new Color(50, 0, 0) : new Color(0, 0, 50);
        for (int i = 0; i < ledArmCount; i++) {
            buffer.setLED(i, climbLeds[i] ? color : Color.kBlack);
            buffer.setLED(i+ledCountPerSide, climbLeds[i] ? color : Color.kBlack);
        }

    }

    public static void setError(boolean e) {
        error = e;
    }

    public void setClimb(boolean v) {
        climbing = v;
    }
    
}