package frc.robot;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.subsystems.superstructure.IntakeSubsystem;

public class RobotState {
    public enum RobotStates {
        BASE,
        CORAL,
        ALGAE,
        OVERRIDE
    }
    
    public static RobotStates currentRobotState = RobotStates.BASE;

    private static IntakeSubsystem intakeSubsystem = RobotContainer.getIntakeSubsystem();

    public static BooleanSupplier stateIsCoral = () -> currentRobotState == RobotStates.CORAL;
    public static BooleanSupplier stateIsAlgae = () -> currentRobotState == RobotStates.ALGAE;
    public static BooleanSupplier stateIsBase = () -> currentRobotState == RobotStates.BASE;
    public static BooleanSupplier stateIsOverride = () -> currentRobotState == RobotStates.OVERRIDE;

    public static BooleanSupplier shouldEject = () -> intakeSubsystem.isCoralPresent() && intakeSubsystem.isAlgaePresent();


    public static RobotStates getRobotState() {
        return currentRobotState;
    }

    public static void updateState(RobotStates state) {
        if (stateIsOverride.getAsBoolean()) {
            DriverStation.reportWarning("Attempted to reset state after override", null);
            return;
        }

        currentRobotState = state;
    }

    public static void updateState() {
        if (stateIsOverride.getAsBoolean()) return;
        
        boolean isCoral = intakeSubsystem.isCoralPresent();
        boolean isAlgae = intakeSubsystem.isAlgaePresent();

        if (shouldEject.getAsBoolean()) {
            DriverStation.reportWarning("Bot thinks both Coral and Algae present, setting RobotState to None", null);
            currentRobotState = RobotStates.BASE;
        }

        if (isCoral) {
            currentRobotState = RobotStates.CORAL;
        } else if (isAlgae) {
            currentRobotState = RobotStates.ALGAE;
        } else {
            currentRobotState = RobotStates.BASE;
        }
    }
}
