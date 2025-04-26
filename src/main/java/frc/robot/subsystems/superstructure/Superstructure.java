package frc.robot.subsystems.superstructure;

import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.constants.RobotConstants.SuperstructueConstants.SuperstructureState;
import frc.robot.subsystems.superstructure.wrist.PitchSubsystem;
import frc.robot.subsystems.superstructure.wrist.RollSubsystem;

public class Superstructure {

    private static Superstructure instance;

    private final ExtenderSubsystem extenderSubsystem = new ExtenderSubsystem();
    private final ArmSubsystem armSubsystem = new ArmSubsystem();
    private final PitchSubsystem pitchSubsystem = new PitchSubsystem();
    private final RollSubsystem rollSubsystem = new RollSubsystem();
    private final IntakeSubsystem intakeSubsystem = new IntakeSubsystem();

    private static SuperstructureState currentSuperstructureState = SuperstructureState.BASE;

    private Superstructure() {}

    /**
     * Applies the given superstructure mode to the subsystems.
     * 
     * @param state The superstructure mode to apply.
     * @return A command that runs the subsystems in the given mode.
     */
    public Command applyTargetState(SuperstructureState state) {
        currentSuperstructureState = state;
        DataLogManager.log("Superstructue state " + state.toString() + " applied");
        
        return Commands.parallel(
            extenderSubsystem.run(state.extenderSetpoint),
            armSubsystem.run(state.armSetpoint),
            pitchSubsystem.run(state.pitchSetpoint), 
            rollSubsystem.run(state.rollSetpoint),
            intakeSubsystem.run(state.intakeMode)
        );
        // return (
        //     Commands.parallel(rollSubsystem.run(state.rollSetpoint), pitchSubsystem.run(state.pitchSetpoint)).alongWith(
        //         Commands.waitUntil(() -> pitchSubsystem.atSetpoint() && rollSubsystem.atSetpoint()).andThen(
        //             armSubsystem.run(state.armSetpoint).alongWith(
        //                 Commands.waitUntil(() -> armSubsystem.atSetpoint()).andThen(
        //                     extenderSubsystem.run(state.extenderSetpoint).alongWith(
        //                         intakeSubsystem.run(state.intakeMode)
        //                     )
        //                 )
        //             )
        //         )
        //     )
        // );
    }

    public static SuperstructureState getCurrentSuperstructureState() {
        return currentSuperstructureState;
    }

    /**
     * Returns the singleton instance of the Superstructure.
     * 
     * @return The Superstructure instance.
     */
    public static synchronized Superstructure getInstance() {
        if (instance == null) {
            instance = new Superstructure();
        }
        return instance;
    }

    public ExtenderSubsystem getExtenderSubsystem() {
        return extenderSubsystem;
    }

    public ArmSubsystem getArmSubsystem() {
        return armSubsystem;
    }

    public PitchSubsystem getPitchSubsystem() {
        return pitchSubsystem;
    }

    public RollSubsystem getRollSubsystem() {
        return rollSubsystem;
    }

    public IntakeSubsystem getIntakeSubsystem() {
        return intakeSubsystem;
    }
}
