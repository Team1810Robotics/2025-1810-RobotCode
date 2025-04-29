package frc.robot;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.subsystems.superstructure.IntakeSubsystem;

public class RobotState {
    public enum RobotStates {
        NONE,
        CORAL,
        ALGAE
    }
    
    public static RobotStates currentRobotState = RobotStates.NONE;

    private static IntakeSubsystem intakeSubsystem = RobotContainer.getIntakeSubsystem();

    public static BooleanSupplier stateIsCoral = () -> currentRobotState == RobotStates.CORAL;
    public static BooleanSupplier stateIsAlgae = () -> currentRobotState == RobotStates.ALGAE;
    public static BooleanSupplier stateIsNone = () -> currentRobotState == RobotStates.NONE;


    public static RobotStates getRobotState() {
        return currentRobotState;
    }

    public static void updateState(RobotStates state) {
        currentRobotState = state;
    }

    public static void updateState() {
        boolean isCoral = intakeSubsystem.isCoralPresent();
        boolean isAlgae = intakeSubsystem.isAlgaePresent();

        if (isCoral && isAlgae) {
            DriverStation.reportWarning("Bot thinks both Coral and Algae present, setting RobotState to None", null);
            currentRobotState = RobotStates.NONE;
        }

        if (isCoral) {
            currentRobotState = RobotStates.CORAL;
        } else if (isAlgae) {
            currentRobotState = RobotStates.ALGAE;
        } else {
            currentRobotState = RobotStates.NONE;
        }
    }


}
