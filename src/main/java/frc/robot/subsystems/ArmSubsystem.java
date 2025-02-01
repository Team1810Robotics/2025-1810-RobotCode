package frc.robot.subsystems;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


public class ArmSubsystem extends SubsystemBase {
   
    private SparkMax armMotor;
    private DutyCycleEncoder armEncoder;
   
    private final ArmFeedforward feedforward;
    private final PIDController armPIDController;

    public ArmSubsystem() {
        //TODO: Figure out this port
        armMotor = new SparkMax(0, MotorType.kBrushless);
        armEncoder = new DutyCycleEncoder(0);
                        
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
        armMotor.set(output + feedforwardOutput);
    }


    public void setSpeed(double speed){
        armMotor.set(speed);
    }



    public void stop(){
        armMotor.stopMotor();
    }


}






