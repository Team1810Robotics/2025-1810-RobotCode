package frc.robot.util;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.revrobotics.spark.config.SparkMaxConfig;

import frc.robot.util.constants.RobotConstants.ArmConstants;

public class Config {
    private static final TalonFXConfiguration PITCH_CONFIG = new TalonFXConfiguration();
    private static final SparkMaxConfig ROLL_CONFIG = new SparkMaxConfig();

    private static final TalonFXConfiguration EXTENDER_CONFIG = new TalonFXConfiguration();

    private static final SparkMaxConfig ARM_CONFIG_1 = new SparkMaxConfig();
    private static final SparkMaxConfig ARM_CONFIG_2 = new SparkMaxConfig();
    
    private static final SparkMaxConfig INTAKE_CONFIG = new SparkMaxConfig();

    private static final double GLOBAL_CURRENT_LIMIT_BASE = 45;


    public static TalonFXConfiguration getPitchConfig() {
        PITCH_CONFIG.CurrentLimits.StatorCurrentLimit = GLOBAL_CURRENT_LIMIT_BASE;
        PITCH_CONFIG.CurrentLimits.StatorCurrentLimitEnable = true;
        PITCH_CONFIG.Feedback.SensorToMechanismRatio = 4; // TODO: Get this value

        return PITCH_CONFIG;
    }

    public static TalonFXConfiguration getExtenderConfig() {
        EXTENDER_CONFIG.CurrentLimits.StatorCurrentLimit = GLOBAL_CURRENT_LIMIT_BASE;
        EXTENDER_CONFIG.CurrentLimits.StatorCurrentLimitEnable = true;
        EXTENDER_CONFIG.Feedback.SensorToMechanismRatio = 4; 

        return EXTENDER_CONFIG;
    }

    public static SparkMaxConfig getRollConfig() {
        ROLL_CONFIG.smartCurrentLimit(40);
        ROLL_CONFIG.idleMode(SparkMaxConfig.IdleMode.kBrake);
        ROLL_CONFIG.inverted(false);

        return ROLL_CONFIG;
    }

    public static SparkMaxConfig getArmConfig1() {
        ARM_CONFIG_1.smartCurrentLimit((int) GLOBAL_CURRENT_LIMIT_BASE);
        ARM_CONFIG_1.idleMode(SparkMaxConfig.IdleMode.kBrake);

        return ARM_CONFIG_1;
    }

    public static SparkMaxConfig getArmConfig2() {
        ARM_CONFIG_2.smartCurrentLimit((int) GLOBAL_CURRENT_LIMIT_BASE);
        ARM_CONFIG_2.idleMode(SparkMaxConfig.IdleMode.kBrake);
        
        ARM_CONFIG_2.follow(ArmConstants.MOTOR_ID_1);

        return ARM_CONFIG_2;
    }

    public static SparkMaxConfig getIntakeConfig() {
        INTAKE_CONFIG.smartCurrentLimit((int) GLOBAL_CURRENT_LIMIT_BASE);
        INTAKE_CONFIG.idleMode(SparkMaxConfig.IdleMode.kCoast);
        INTAKE_CONFIG.inverted(false);

        return INTAKE_CONFIG;
    }
}
