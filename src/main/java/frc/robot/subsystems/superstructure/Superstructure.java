package frc.robot.subsystems.superstructure;

import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.superstructure.wrist.PitchSubsystem;
import frc.robot.subsystems.superstructure.wrist.RollSubsystem;

public class Superstructure {

    private PitchSubsystem pitchSubsystem;
    private RollSubsystem rollSubsystem;
    private ExtenderSubsystem extenderSubsystem;
    private ArmSubsystem armSubsystem;
    private IntakeSubsystem intakeSubsystem;

    private static SuperstructureState currentSuperstructureState = SuperstructureState.BASE;

    public Superstructure(PitchSubsystem pitchSubsystem, RollSubsystem rollSubsystem,
            ExtenderSubsystem extenderSubsystem, ArmSubsystem armSubsystem, IntakeSubsystem intakeSubsystem) {
        this.pitchSubsystem = pitchSubsystem;
        this.rollSubsystem = rollSubsystem;
        this.extenderSubsystem = extenderSubsystem;
        this.armSubsystem = armSubsystem;
        this.intakeSubsystem = intakeSubsystem;
    }

    public static SuperstructureState getCurrentSuperstructureState() {
        return currentSuperstructureState;
    }

    public boolean superstructureAtSetpoint() {
        return pitchSubsystem.atSetpoint() && rollSubsystem.atSetpoint() && extenderSubsystem.atSetpoint()
                && armSubsystem.atSetpoint();
    }

    /**
     * Applies the given superstructure mode to the subsystems.
     * 
     * @param state The superstructure mode to apply.
     * @return A command that runs the subsystems in the given mode.
     */
    public Command applyTargetStateParallel(SuperstructureState state) {
        currentSuperstructureState = state;
        DataLogManager.log("Superstructue state " + state.toString() + " parallely applied");
        
        return Commands.parallel(
            extenderSubsystem.run(state.extenderSetpoint),
            armSubsystem.run(state.armSetpoint),
            pitchSubsystem.run(state.pitchSetpoint), 
            rollSubsystem.run(state.rollSetpoint),
            intakeSubsystem.run(state.intakeMode)
        );

    }

    public Command applyTargetStateDynamic(SuperstructureState state) {
        currentSuperstructureState = state;
        DataLogManager.log("Superstructue state " + state.toString() + " dynamically applied");
        
        return (
            Commands.parallel(rollSubsystem.run(state.rollSetpoint), pitchSubsystem.run(state.pitchSetpoint)).alongWith(
                Commands.waitUntil(() -> pitchSubsystem.atSetpoint() && rollSubsystem.atSetpoint()).andThen(
                    armSubsystem.run(state.armSetpoint).alongWith(
                        Commands.waitUntil(() -> armSubsystem.atSetpoint()).andThen(
                            extenderSubsystem.run(state.extenderSetpoint).alongWith(
                                intakeSubsystem.run(state.intakeMode)
                            )
                        )
                    )
                )
            )
        );
    }
}
