package frc.robot.subsystems.superstructure;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.Configs;
import frc.robot.util.constants.RobotConstants.ArmConstants;

import com.revrobotics.spark.SparkBase.ResetMode;

public class ArmSubsystem extends SubsystemBase {
   
    private SparkMax armMotor1;
    private SparkMax armMotor2;
    private DutyCycleEncoder armEncoder;

    private final PIDController armPIDController;

    public double currentSetpoint;

    public ArmSubsystem() {
        armMotor1 = new SparkMax(ArmConstants.MOTOR_ID_1, MotorType.kBrushless);
        armMotor2 = new SparkMax(ArmConstants.MOTOR_ID_2, MotorType.kBrushless);
        armEncoder = new DutyCycleEncoder(ArmConstants.ENCODER_ID);


        armMotor1.configure(Configs.getArmConfig1(), ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
        armMotor2.configure(Configs.getArmConfig2(), ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        
        armPIDController = new PIDController(ArmConstants.kP, ArmConstants.kI, ArmConstants.kD);
        armPIDController.setTolerance(ArmConstants.TOLERANCE);

        SmartDashboard.putData("armPID", armPIDController);
        Shuffleboard.getTab("Arm").addNumber("Arm Deg",() -> getMeasurement());
        Shuffleboard.getTab("Arm").addNumber("Arm Rad Raw",() -> armEncoder.get());
        Shuffleboard.getTab("Arm").addNumber("Arm Rad Adj",() -> armEncoder.get() - ArmConstants.ENCODER_OFFSET);

        Shuffleboard.getTab("Arm").addNumber("Arm PID Out", () -> armPIDController.calculate(getMeasurement(), 0));
        Shuffleboard.getTab("Encoder").addBoolean("Arm Encoder", () -> armEncoder.isConnected());

        Shuffleboard.getTab("Arm").addNumber("Arm Setpoint", () -> currentSetpoint);
    }

    /**
     * Get the current measurement of the arm in degrees.
     *
     * <p>
     * This method returns the current angle of the arm in degrees, calculated
     * by subtracting the encoder offset from the raw encoder value and then
     * converting the result to degrees.
     *
     * @return the current measurement of the arm in degrees
     */
    public double getMeasurement() {
        double position = armEncoder.get() - ArmConstants.ENCODER_OFFSET; 
        double degrees = Units.rotationsToDegrees(position);
       
        return degrees; 
    }


    public boolean isEncoderConnected(){
        return armEncoder.isConnected();
    }

    public double armPower(){
        return armMotor1.getOutputCurrent();
    }

    public void setSpeed(double speed){
        armMotor1.set(speed);
    }

    public boolean atSetpoint() {
        return armPIDController.atSetpoint();
    }

    /**
     * Runs the arm motor with PID
     * 
     * @param setpoint Setpoint for arm
     * @return Command to run the arm
     */
    public Command run(double setpoint) {
        currentSetpoint = setpoint;
        if (armEncoder.isConnected()){
            double output = armPIDController.calculate(getMeasurement(), setpoint);
            return Commands.run(() -> armMotor1.set(-output), this).finallyDo(() -> stop());
        } else {
            DataLogManager.log("Arm Encoder Disconnected");
            stop();
            return new InstantCommand();
        }
    }

    public void stop(){
        armMotor1.stopMotor();
        armMotor2.stopMotor();
    }
}