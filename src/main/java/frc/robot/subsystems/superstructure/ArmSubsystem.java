package frc.robot.subsystems.superstructure;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.lib.Elastic;
import frc.robot.Constants.ArmConstants;
import com.revrobotics.spark.SparkBase.ResetMode;

public class ArmSubsystem extends SubsystemBase {
   
    private SparkMax armMotor1;
    private SparkMax armMotor2;
    private DutyCycleEncoder armEncoder;

    private SparkMaxConfig config1;
    private SparkMaxConfig config2;

    private Elastic.Notification notification;
   
    private final PIDController armPIDController;

    public double currentSetpoint;

    public ArmSubsystem() {
        armMotor1 = new SparkMax(ArmConstants.MOTOR_ID_1, MotorType.kBrushless);
        armMotor2 = new SparkMax(ArmConstants.MOTOR_ID_2, MotorType.kBrushless);
        armEncoder = new DutyCycleEncoder(ArmConstants.ENCODER_ID);

        config1 = new SparkMaxConfig();
        config2 = new SparkMaxConfig();

        config2.follow(ArmConstants.MOTOR_ID_1);

        config1.idleMode(IdleMode.kBrake);
        config2.idleMode(IdleMode.kBrake);

        armMotor1.configure(config1, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
        armMotor2.configure(config2, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        
        armPIDController = new PIDController(ArmConstants.kP, ArmConstants.kI, ArmConstants.kD);

        SmartDashboard.putData("armPID", armPIDController);
        Shuffleboard.getTab("Arm").addNumber("Arm Deg",() -> getMeasurement());
        Shuffleboard.getTab("Arm").addNumber("Arm Rad Raw",() -> armEncoder.get());
        Shuffleboard.getTab("Arm").addNumber("Arm Rad Adj",() -> armEncoder.get() - ArmConstants.ENCODER_OFFSET);

        Shuffleboard.getTab("Arm").addNumber("Arm PID Out", () -> armPIDController.calculate(getMeasurement(), 0));
        Shuffleboard.getTab("Encoder").addBoolean("Arm Encoder", () -> armEncoder.isConnected());

        Shuffleboard.getTab("Arm").addNumber("Arm Setpoint", () -> currentSetpoint);

        notification = new Elastic.Notification(Elastic.Notification.NotificationLevel.ERROR, "!!! Arm Error !!!", "Encoder Disconected");
    }

    public double getMeasurement() {
        double position = armEncoder.get() - ArmConstants.ENCODER_OFFSET; 
        double degrees = Units.rotationsToDegrees(position);
       
        return degrees; 
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
            System.out.println("Arm Encoder Disconnected");
            stop();
            Elastic.sendNotification(notification.withAutomaticHeight());
            return new InstantCommand();
        }
    }

    public boolean isEncoderConnected(){
        return armEncoder.isConnected();
    }

    public void testNotif(){
        Elastic.sendNotification(notification);
    }

    public double armPower(){
        return armMotor1.getOutputCurrent();
    }

    public void setSpeed(double speed){
        armMotor1.set(speed);
    }

    public boolean atSetpoint() {
        return Math.abs(getMeasurement() - armPIDController.getSetpoint()) < ArmConstants.TOLERANCE;
    }

    public void stop(){
        armMotor1.stopMotor();
        armMotor2.stopMotor();
    }
}