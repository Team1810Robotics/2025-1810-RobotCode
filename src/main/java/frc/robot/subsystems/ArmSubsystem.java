package frc.robot.subsystems;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ArmConstants;
import com.revrobotics.spark.SparkBase.ResetMode;


public class ArmSubsystem extends SubsystemBase {
   
    private SparkMax armMotor1;
    private SparkMax armMotor2;
    private DutyCycleEncoder armEncoder;

    private SparkMaxConfig config;
   
    private final ArmFeedforward feedforward;
    private final PIDController armPIDController;

    public ArmSubsystem() {
        armMotor1 = new SparkMax(ArmConstants.MOTOR_ID_1, MotorType.kBrushless);
        armMotor2 = new SparkMax(ArmConstants.MOTOR_ID_2, MotorType.kBrushless);
        armEncoder = new DutyCycleEncoder(ArmConstants.ENCODER_ID);

        config = new SparkMaxConfig();

        config.follow(ArmConstants.MOTOR_ID_1);

        armMotor2.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        
        armPIDController = new PIDController(0 , 0 , 0 ); //kP //kI //kD
        feedforward = new ArmFeedforward(0 , 0 , 0 ); //ks //kg //kv

        SmartDashboard.putData(armPIDController);
        Shuffleboard.getTab("Arm").addNumber("Arm Deg",() -> armEncoder.get());
    }

    public double getMeasurement() {
        double position = armEncoder.get() -0.382;
        double degrees = position * 360;
       
        return degrees - 0.382; //TODO: change zero to real offset
    }


    public void useOutput(double setpoint) {
        double feedforwardOutput = feedforward.calculate(setpoint, 0);
        double output = armPIDController.calculate(getMeasurement(), setpoint);

        armMotor1.set(-output + feedforwardOutput);
    }

    public double armPower(){
        return armMotor1.getOutputCurrent();
    }


    public void setSpeed(double speed){
        armMotor1.set(speed);
    }

    public void stop(){
        armMotor1.stopMotor();
        armMotor2.stopMotor();
    }
}






