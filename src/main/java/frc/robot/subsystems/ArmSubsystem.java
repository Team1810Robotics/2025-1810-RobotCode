package frc.robot.subsystems;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
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
        armEncoder = new DutyCycleEncoder(0);

        config = new SparkMaxConfig();

        config.follow(ArmConstants.MOTOR_ID_1);

        armMotor2.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        
        armPIDController = new PIDController(0 , 0 , 0 ); //kP //kI //kD
        feedforward = new ArmFeedforward(0 , 0 , 0 ); //ks //kg //kv
    }


    public double getMeasurement() {
        double position = armEncoder.get();
        double degrees = position * 360;
       
        return degrees - 0; //TODO: change zero to real offset
    }


    public void useOutput(double output, double setpoint) {
        // double armEncoderVelocity = //TODO: Recalc velocity
        //     armEncoder.getVelocity().getValueAsDouble() * 360;
        double feedforwardOutput = feedforward.calculate(setpoint, 0);
        armMotor1.set(output + feedforwardOutput);
        armMotor2.set(output + feedforwardOutput);
    }


    public void setSpeed(double speed){
        armMotor1.set(speed);
    }



    public void stop(){
        armMotor1.stopMotor();
        armMotor2.stopMotor();
    }


}






