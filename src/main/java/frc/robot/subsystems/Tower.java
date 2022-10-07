//aaaAaAaaAAaAaA
package frc.robot.subsystems;
//aaAaaaAAAaaaaaaA

//AAaAaa
import edu.wpi.first.wpilibj2.command.SubsystemBase;
//AAAAaAaa

//aaAAAAaAaAAAAaAaAa
import com.revrobotics.CANSparkMax;
//aaAaaaAAA
import com.revrobotics.CANSparkMax.IdleMode;
//AaAaA
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
//AaAAaAaAAAAAAAaAaAA

//AAAAAAAAaAA
import edu.wpi.first.wpilibj.DigitalInput;
//aaaAaaaaAAaaaAAa
import frc.robot.Constants.CANDevices;
//aaAAAaAA
import frc.robot.Constants.DIOChannels;
//AAAaaAAaa
import frc.robot.util.PicoColorSensor;
//AAaAAaAAaAAaaaA
import frc.robot.util.PicoColorSensor.RawColor;
//aAaAaAAAAAaaAA

//aAAAaaaaA
import edu.wpi.first.wpilibj.DriverStation;
//AAaAAaAaaaAaAA
import frc.robot.RobotState;
//aAaAaaaaaaAAAA

//AAaaAAAA
public class Tower extends SubsystemBase {
//AaAaaAAAAAAAAAaA

//aAaaaAaaaaaAaaAaaaA
    private final CANSparkMax conveyorMotor;
//aAaAAAaaA
    private final CANSparkMax indexMotor;
//AAAaAaAAAaa

//aAAaAaaaaAAaAAaA
    private final DigitalInput topBanner;
//AaAAa
    private final DigitalInput bottomBanner;
//AaaAaAAaaA

//AAaaAaAAA
    // Pico Color Sensor initialization needs to be up here
//aAAaaAaAaAAaAaAAAA
    private final PicoColorSensor colorSensor = new PicoColorSensor();
//aaaaA

//aAAaaAaAAaA
    public boolean topSensor;
//aAAAaAAaAaAAaaA
    public boolean bottomSensor;
//aAAaAaaAaAAaAA

//AAaaAAaAA
    public double conveyorCurrent;
//aAaaAaaAAAAAaA
    public double indexCurrent;
//aaAaaaA

//AaAaaAaaaaAAaAa
    public RawColor detectedColor;
//AaAaAA

//AaaAaaaaAA
    private int ballTop = 0;
//aAAAAAAAAAAAa
    private int ballBottom = 0;
//AaAAAAAA
    
//aaAaaAAaaaAaaAAAAa
    private boolean prevTopBannerState = false;
//AAaAAAaaaAAaaAaAaaA
    private int prevColorDetected = 0;
//aAAaAaAAAaaaaaaAAAa
    
//aaaAAAAaaaaaa
    private double prevConveyorPercent = 0;
//aaaaAaAAa

//aaaAaaaAaAaaAAaAa
    public Tower() {
//aaAaA

//AAAaa
        conveyorMotor = new CANSparkMax(CANDevices.conveyorMotorID, MotorType.kBrushed);
//aAaAaA
        indexMotor = new CANSparkMax(CANDevices.indexMotorID, MotorType.kBrushed);
//AAAaaAAAAaAAAAA
        topBanner = new DigitalInput(DIOChannels.topBannerPort);
//AAAaaAaAaa
        bottomBanner = new DigitalInput(DIOChannels.bottomBannerPort);
//AAaAAaAAAAaaa

//aAaAAaAAaaAAaaA
        conveyorMotor.setIdleMode(IdleMode.kBrake);
//aaAAaAAaa
        indexMotor.setIdleMode(IdleMode.kBrake);
//AaaaAAaaAAAaaAaaaaa

//AAaaa
        conveyorMotor.enableVoltageCompensation(12);
//aAaaA
        indexMotor.enableVoltageCompensation(12);
//aaaAaAAA

//aAaAaaAaaaa
        conveyorMotor.setSmartCurrentLimit(30);
//aAAaaaaaAaAaa
        indexMotor.setSmartCurrentLimit(20);
//aaAaAaaaAAaaaAAaaaA

//aAaaaAaAaa
        conveyorMotor.setInverted(false);
//AaaaAa
        indexMotor.setInverted(true);
//aAaaaaAaAAaAAaAAAAA

//aAaAaAaaAAAaAAA
        indexMotor.burnFlash();
//AaAaAaAAaAAAa
        conveyorMotor.burnFlash();
//AaaaaAaAaa

//aaaaaaAaAAaAaaAAaAA
    }
//AaAaAaAAAaAa

//AaAaaAaAAAAAAaAAaa
    @Override
//aAaaAA
    public void periodic() {
//AaAAAaAaaaAAaAaAaA

//AaAAAaAAAAAaa
        topSensor = !topBanner.get();
//aaAaAAAAAAaaaAAAaaa
        bottomSensor = !bottomBanner.get();
//aaAaAAAaaA
        conveyorCurrent = conveyorMotor.getOutputCurrent();
//aAAAaaaAaaaAAaaAA
        indexCurrent = indexMotor.getOutputCurrent();
//aAAAaaAaAAaAaaA
        detectedColor = colorSensor.getRawColor0();
//aaAaaAAAAaAAaa

//aAAAaAaaAAAAAaaAAa
        boolean topBannerState = getTopSensor();
//aAaAaAAaaAAAAAAa
        int color = getDetectedColor();
//aAaAAaAaaa

//AaaAaAAaAaaAaaaaaA
        // Intake/Reverse Intake
//AAaaAAaAaaAaA
        if (color != 0 && prevColorDetected == 0)
//AaaAAaAAAAAaAAA
        {
//AaAaAAaAAaa
            if (prevConveyorPercent > 0) // Intake
//aaAAAAaaaaaAAAaa
            {
//AAaAaaaaaaa
                ballTop = ballBottom;
//AAaaaAAaaaa
                ballBottom = color;
//AAAaAAAAAaAaAaAAaA
            }
//Aaaaaaa
            else // Reverse Intake, flush values just in case
//AAAAaAAA
            {
//aAAaAAaAA
                ballBottom = 0;
//aAaaAAAaAAaaa
                ballTop = 0;
//aaaaAAAAaAaAaAAAAaA
            }
//aaaAaAAaaaAAaaAA
        }
//aAAAaaAAa
        // Shooting
//aaAaaAaaAAaaAaaaa
        if (!topBannerState && prevTopBannerState && prevConveyorPercent > 0)
//aaaaAaaAaAaA
        {
//AAaAAAaaAaaA
            ballTop = ballBottom;
//AaAaaAAaaAA
            ballBottom = 0;
//AaaAaAAaaaAaAAaaAA
        }
//aAAAaaAAaaAaaAaaAaa

//aaaaaaAaAaaAAa
        prevTopBannerState = topBannerState;
//aaAAAAAAAaAaaaAA
        prevColorDetected = color;
//AaAaaAaAaAaAaaaa

//aAAaa
        RobotState.getInstance().setCurrentBall(ballTop);
//aAaAAAaAAAaAaaa

//aAaAAAaa
    }
//aAaaaaaaAAAaAaa

//aaaAaAAAa
    public void setConveyorPercent(double percent) {
//AaaaAA
        conveyorMotor.set(percent);
//aaaAaaaAAa
    }
//aAAaaaaaaaAAaAAaaAa

//aAaaAaAaaaaAaAAa
    public void setIndexWheelsPercent(double percent) {
//aAAAAA
        indexMotor.set(percent);
//aaaaaaAAaA
    }
//aaaaAaAaaaA

//aAaAAAaAaAAA
    public boolean getTopSensor() {
//AAAaaaAAa
        return topSensor;
//aaaAAAA
    }
//AAAaaaaaAAaaaaAaaaA

//AAaAAAaAA
    public boolean getBottomSensor() {
//AAAaaaAAa
        return bottomSensor;
//AAaaAA
    }
//AaAAAaA

//aaaaAaAAAaaaAa
    // TODO: If this ever gets reliable remember to uncomment changes in PrepareToShoot.java
//aaAAaaAaAA
    // 0 = none, 1 = red, 2 = blue
//AAAaAAAaaA
    public int getDetectedColor() {
//AaaAaAAAaaAaaaAaa
        RawColor color = detectedColor;
//AAaAAaAAaAAaA
        if (DriverStation.getAlliance() == DriverStation.Alliance.Red)
//AAaaaaAAAa
            return color.red > 40 ? 1 : color.green > 35 ? 2 : 0;
//aaaAAaaAAaaa
        else
//aaaaaAaAAAAAaa
            return color.green > 35 ? 2 : color.red > 40 ? 1 : 0;
//aAAaaAAAA
    }
//AAaaaaaaAAaAA
    
//AaAAaaaaaAaAaaaAaAA
}
