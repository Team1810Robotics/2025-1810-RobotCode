package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.LedConstants;

import com.ctre.phoenix.led.*;

public class LedSubsystem extends SubsystemBase {
    private final CANdle m_candle = new CANdle(35, "rio");
    private Animation m_toAnimate = null;

    public enum AnimationTypes {
        ColorFlow, Fire, Larson, Rainbow, RgbFade, SingleFade, Strobe, Twinkle, TwinkleOff, SetAll
    }

    public LedSubsystem() {
        configureCANdle();
    }

    private void configureCANdle() {
        CANdleConfiguration configAll = new CANdleConfiguration();
        configAll.statusLedOffWhenActive = true;
        configAll.disableWhenLOS = false;
        configAll.stripType = CANdle.LEDStripType.GRB;
        configAll.brightnessScalar = 0.1;
        configAll.vBatOutputMode = CANdle.VBatOutputMode.Modulated;
        m_candle.configAllSettings(configAll, 100);
    }

    public void manageLed() {
        m_candle.setLEDs(255, 0, 0, 0, 0, LedConstants.CANdle_COUNT); // Candle leds - 8
        m_candle.setLEDs(0, 150, 150, 0, LedConstants.CANdle_COUNT + 1, LedConstants.ARM_L_COUNT); // Left Arm leds - ?
        m_candle.setLEDs(255, 0, 0, 0, LedConstants.ARM_L_COUNT + 1, LedConstants.ARM_R_COUNT); // Right Arm leds - ?
        m_candle.setLEDs(255, 0, 0, 0, LedConstants.ARM_R_COUNT + 1, LedConstants.ARM_STATUS); // Arm Status leds - ?
    }

    @Override
    public void periodic() {
        if (m_toAnimate == null) {
            m_candle.setLEDs(255, 0, 0); // Default solid red
        } else {
            manageLed();
        }
    }
}