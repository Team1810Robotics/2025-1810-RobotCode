package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ExtenderConstants;
import frc.robot.Constants.ExtenderConstants.ExtenderHeights;

public class ExtenderSubsystem extends SubsystemBase {

    private SparkMax extenderMotor;
    private DutyCycleEncoder encoder;

    private PIDController extenderpidController;

    private double cumulativeRotations = 0;
    private double previousRotation = 0;
    private int fullRotations = 0;  


    public ExtenderSubsystem() {
        extenderMotor = new SparkMax(ExtenderConstants.MOTOR_ID, MotorType.kBrushless);
        encoder = new DutyCycleEncoder(ExtenderConstants.ENCODER_ID);

        extenderpidController = new PIDController(ExtenderConstants.kP, ExtenderConstants.kI, ExtenderConstants.kD); //I have no clue if this will work

        Shuffleboard.getTab("Extender").addNumber("Extender Encoder", () -> getEncoder());
        Shuffleboard.getTab("Extender").addNumber("Extender Distance", () -> getDistance());
    }

    /**
     * Get the total number of rotations the extender has gone through.
     *
     * <p>This method detects when the encoder wraps around and adds one to the cumulative
     * count. This is necessary because the DutyCycleEncoder wraps around to 0 after a certain
     * point, so simply adding up the positions would not give an accurate total.
     *
     * @return the total number of rotations the extender has gone through
     */
    
    public void totalRotations() {
        double currentRotation = getEncoder();
        
        // Check for wraparound
        if (currentRotation - previousRotation > 0.5) {
            fullRotations--;  // Wrapped backwards (1 -> 0)
        } else if (currentRotation - previousRotation < -0.5) {
            fullRotations++;  // Wrapped forwards (0 -> 1)
        }
        
        cumulativeRotations = fullRotations + currentRotation;
        previousRotation = currentRotation;
    }

    public double getEncoder() {
        return encoder.get() - .09;
    }

    public void run(double speed) {
        extenderMotor.set(speed);
    }

    public double getDistance() {
        return cumulativeRotations * ExtenderConstants.INCHES_PER_ROTATION;
    }

    public void extend(double height) {
        extenderMotor.set(extenderpidController.calculate(getDistance(), height));
    }

    // /**
    //  * Given a current distance and a target distance, returns the output scalar that should be
    //  * given to the motor in order to reach the target distance. This scalar is calculated using
    //  * a quadratic equation to slow down the motor as it approaches its target.
    //  * 
    //  * <a href="https://www.desmos.com/calculator/tldkhkjmiz">See the equation visualized here</a>
    //  *
    //  * @param currentDistance the current distance
    //  * @param targetDistance the target distance
    //  * @return the output scalar
    //  */
    // public double outputScalar(double currentDistance, double targetDistance) {
    //     double percentDistance = (currentDistance - targetDistance) / targetDistance;

    //     if (percentDistance < 0 || percentDistance > 1) {
    //         CommandScheduler.getInstance().schedule((Commands.print("Invalid percent distance of: " + percentDistance)));
    //         return 0;
    //     }

    //     return (-4 * Math.pow(percentDistance, 2)) + (4 * percentDistance);
    // }

    public double getTargetHeight(ExtenderHeights height) {
        switch (height) {
            case BASE:
                return ExtenderConstants.BASE_HEIGHT;
            case L2:
                return ExtenderConstants.L2_HEIGHT;
            case L3:
                return ExtenderConstants.L3_HEIGHT;
            case L4:
                return ExtenderConstants.L4_HEIGHT;
        }
        return 0;
        
    }
    
    public void stop() {
        extenderMotor.stopMotor();
    }
    
    @Override
    public void periodic() {
        totalRotations();
    }

    

}