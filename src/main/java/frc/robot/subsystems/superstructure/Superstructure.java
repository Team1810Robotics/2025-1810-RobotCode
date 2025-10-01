package frc.robot.subsystems.superstructure;

import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.superstructure.wrist.PitchSubsystem;
import frc.robot.subsystems.superstructure.wrist.RollSubsystem;
import frc.robot.util.constants.RobotConstants.IntakeConstants.IntakeMode;

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
     * Applies the given superstructure mode to the subsystems using alongWith() like the original code.
     * This creates a race group instead of a parallel group, avoiding subsystem conflicts.
     * 
     * @param state The superstructure mode to apply.
     * @return A command that runs the subsystems in the given mode.
     */
    public Command applyTargetState(SuperstructureState state) {
        // currentSuperstructureState = state;
        DataLogManager.log("Superstructur%e state " + state.toString() + " applied");
        
        
        return armSubsystem.run(state.armSetpoint).alongWith(
            rollSubsystem.run(state.rollSetpoint),
            pitchSubsystem.run(state.pitchSetpoint),
            extenderSubsystem.run(state.extenderSetpoint),
            intakeSubsystem.run(state.intakeMode)
        );
    }

    /**
     * Alternative method for positions that need intake to run independently
     * (like your original outtake method)
     */
    public Command applyTargetStateWithSeparateIntake(SuperstructureState state, IntakeMode intakeMode) {
        currentSuperstructureState = state;
        DataLogManager.log("Superstructure state " + state.toString() + " applied with separate intake mode: " + intakeMode);
        
        // For cases where you want to override the intake mode (like outtaking while maintaining position)
        return armSubsystem.run(state.armSetpoint).alongWith(
            rollSubsystem.run(state.rollSetpoint),
            pitchSubsystem.run(state.pitchSetpoint),
            extenderSubsystem.run(state.extenderSetpoint),
            intakeSubsystem.run(intakeMode)
        );
    }

    /**
     * Create an outtake command like your original - maintains current position while outtaking
     */
    public Command outtakeAtCurrentPosition() {
        // Get current setpoints (you may need to expose these from your subsystems)
        return intakeSubsystem.run(IntakeMode.OUT).alongWith(
            pitchSubsystem.run(pitchSubsystem.currentSetpoint),
            rollSubsystem.run(rollSubsystem.currentSetpoint),
            extenderSubsystem.run(extenderSubsystem.getCurrentSetpoint()),
            armSubsystem.run(armSubsystem.currentSetpoint)
        );
    }
}