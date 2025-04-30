package frc.robot;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.subsystems.superstructure.IntakeSubsystem;
import frc.robot.subsystems.superstructure.Superstructure;

public class RobotState {
    public static enum States {
        NONE,
        CORAL,
        ALGAE
    }

    public static States currentRobotState = States.NONE;

    private static IntakeSubsystem intakeSubsystem = Superstructure.getInstance().getIntakeSubsystem();

    public static BooleanSupplier stateIsCoral = () -> currentRobotState == States.CORAL;
    public static BooleanSupplier stateIsAlgae = () -> currentRobotState == States.ALGAE;
    public static BooleanSupplier stateIsNone = () -> currentRobotState == States.NONE;

    public static BooleanSupplier shouldEject = () -> intakeSubsystem.isCoralPresent() == intakeSubsystem.isAlgaePresent() == true;

    public static void updateState(States state) {
        currentRobotState = state;
    }

    public static void updateState() {
        boolean isCoral = intakeSubsystem.isCoralPresent();
        boolean isAlgae = intakeSubsystem.isAlgaePresent();

        if (shouldEject.getAsBoolean()) {
            DriverStation.reportWarning("Bot thinks both Coral and Algae present, setting RobotState to None", null);
            currentRobotState = States.NONE;
        }

        if (isCoral) {
            currentRobotState = States.CORAL;
        } else if (isAlgae) {
            currentRobotState = States.ALGAE;
        } else {
            currentRobotState = States.NONE;
        }
    }

    public static States getRobotState() {
        return currentRobotState;
    }


}
