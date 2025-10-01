package frc.robot.subsystems;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.lib.Elastic;
import frc.robot.util.Configs;
import frc.robot.util.ShuffleboardTabs;
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

    private ShuffleboardTab tab = ShuffleboardTabs.ARM;

    public ArmSubsystem() {
        armMotor1 = new SparkMax(ArmConstants.MOTOR_ID_1, MotorType.kBrushless);
        armMotor2 = new SparkMax(ArmConstants.MOTOR_ID_2, MotorType.kBrushless);
        armEncoder = new DutyCycleEncoder(ArmConstants.ENCODER_ID);

        config1 = Configs.getArmConfig1();
        config2 = Configs.getArmConfig2();

        armMotor1.configure(config1, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
        armMotor2.configure(config2, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        
        armPIDController = new PIDController(ArmConstants.kP, ArmConstants.kI, ArmConstants.kD);

        tab.addNumber("Degree",() -> getMeasurement());
        tab.addNumber("Raw", () -> armEncoder.get());
        tab.addNumber("Raw Adjusted", () -> armEncoder.get() - ArmConstants.ENCODER_OFFSET);
        tab.addNumber("PID Out", () -> armPIDController.calculate(getMeasurement(), currentSetpoint));
        tab.add("PID", armPIDController);

    }

    public double getMeasurement() {
        double position = armEncoder.get() - ArmConstants.ENCODER_OFFSET; 
        double degrees = Units.rotationsToDegrees(position);
       
        return degrees; 
    }

    public void run(double setpoint) {
        currentSetpoint = setpoint;
        if (armEncoder.isConnected()){
            double output = armPIDController.calculate(getMeasurement(), setpoint);
            armMotor1.set(-output);
        } else {
            System.out.println("Arm Encoder Disconnected");
            stop();
            Elastic.sendNotification(notification.withAutomaticHeight());
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

    public void stop(){
        armMotor1.stopMotor();
        armMotor2.stopMotor();
    }
}