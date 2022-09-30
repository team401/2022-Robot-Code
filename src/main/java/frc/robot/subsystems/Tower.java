package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import frc.robot.Constants.CANDevices;
import frc.robot.Constants.DIOChannels;
import frc.robot.util.PicoColorSensor;
import frc.robot.util.PicoColorSensor.RawColor;

import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.RobotState;

public class Tower extends SubsystemBase {

    private final CANSparkMax conveyorMotor;
    private final CANSparkMax indexMotor;

    private final DigitalInput topBanner;
    private final DigitalInput bottomBanner;

    // Pico Color Sensor initialization needs to be up here
    private final PicoColorSensor colorSensor = new PicoColorSensor();

    public boolean topSensor;
    public boolean bottomSensor;

    public double conveyorCurrent;
    public double indexCurrent;

    public RawColor detectedColor;

    private int ballTop = 0;
    private int ballBottom = 0;
    
    private boolean prevTopBannerState = false;
    private int prevColorDetected = 0;
    
    private double prevConveyorPercent = 0;

    public Tower() {

        conveyorMotor = new CANSparkMax(CANDevices.conveyorMotorID, MotorType.kBrushed);
        indexMotor = new CANSparkMax(CANDevices.indexMotorID, MotorType.kBrushed);
        topBanner = new DigitalInput(DIOChannels.topBannerPort);
        bottomBanner = new DigitalInput(DIOChannels.bottomBannerPort);

        conveyorMotor.setIdleMode(IdleMode.kBrake);
        indexMotor.setIdleMode(IdleMode.kBrake);

        conveyorMotor.enableVoltageCompensation(12);
        indexMotor.enableVoltageCompensation(12);

        conveyorMotor.setSmartCurrentLimit(30);
        indexMotor.setSmartCurrentLimit(20);

        conveyorMotor.setInverted(false);
        indexMotor.setInverted(true);

        indexMotor.burnFlash();
        conveyorMotor.burnFlash();

    }

    @Override
    public void periodic() {

        topSensor = !topBanner.get();
        bottomSensor = !bottomBanner.get();
        conveyorCurrent = conveyorMotor.getOutputCurrent();
        indexCurrent = indexMotor.getOutputCurrent();
        detectedColor = colorSensor.getRawColor0();

        boolean topBannerState = getTopSensor();
        int color = getDetectedColor();

        // Intake/Reverse Intake
        if (color != 0 && prevColorDetected == 0)
        {
            if (prevConveyorPercent > 0) // Intake
            {
                ballTop = ballBottom;
                ballBottom = color;
            }
            else // Reverse Intake, flush values just in case
            {
                ballBottom = 0;
                ballTop = 0;
            }
        }
        // Shooting
        if (!topBannerState && prevTopBannerState && prevConveyorPercent > 0)
        {
            ballTop = ballBottom;
            ballBottom = 0;
        }

        prevTopBannerState = topBannerState;
        prevColorDetected = color;

        RobotState.getInstance().setCurrentBall(ballTop);

    }

    public void setConveyorPercent(double percent) {
        conveyorMotor.set(percent);
    }

    public void setIndexWheelsPercent(double percent) {
        indexMotor.set(percent);
    }

    public boolean getTopSensor() {
        return topSensor;
    }

    public boolean getBottomSensor() {
        return bottomSensor;
    }

    // TODO: If this ever gets reliable remember to uncomment changes in PrepareToShoot.java
    // 0 = none, 1 = red, 2 = blue
    public int getDetectedColor() {
        RawColor color = detectedColor;
        if (DriverStation.getAlliance() == DriverStation.Alliance.Red)
            return color.red > 40 ? 1 : color.green > 35 ? 2 : 0;
        else
            return color.green > 35 ? 2 : color.red > 40 ? 1 : 0;
    }
    
}
