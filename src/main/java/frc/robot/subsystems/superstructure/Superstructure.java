package frc.robot.subsystems.superstructure;

import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.ExtenderConstants;
import frc.robot.Constants.IntakeConstants.IntakeMode;
import frc.robot.Constants.WristConstants.PitchConstants;
import frc.robot.Constants.WristConstants.RollConstants;
import frc.robot.subsystems.superstructure.wrist.PitchSubsystem;
import frc.robot.subsystems.superstructure.wrist.RollSubsystem;

public class Superstructure {
    public static enum SuperstructureMode {
        BASE(ExtenderConstants.BASE_HEIGHT, ArmConstants.BASE_POSITION, PitchConstants.BASE_POSITION, RollConstants.BASE_POSITION, IntakeMode.STOP),
        L1(ExtenderConstants.L1_HEIGHT, ArmConstants.L1_POSITION, PitchConstants.L1_POSITION, RollConstants.L1_POSITION, IntakeMode.STOP),
        L2(ExtenderConstants.L2_HEIGHT, ArmConstants.L2_POSITION, PitchConstants.L2_POSITION, RollConstants.L2_POSITION, IntakeMode.STOP),
        L3(ExtenderConstants.L3_HEIGHT, ArmConstants.L3_POSITION, PitchConstants.L3_POSITION, RollConstants.L3_POSITION, IntakeMode.STOP),
        L4(ExtenderConstants.L4_HEIGHT, ArmConstants.L4_POSITION, PitchConstants.L4_POSITION, RollConstants.L4_POSITION, IntakeMode.STOP),
        ALGAE1(ExtenderConstants.ALGAE1_HEIGHT, ArmConstants.ALGAE1_POSITION, PitchConstants.ALGAE1_POSITION, RollConstants.ALGAE1_POSITION, IntakeMode.KICK),
        ALGAE2(ExtenderConstants.ALGAE2_HEIGHT, ArmConstants.ALGAE2_POSITION, PitchConstants.ALGAE2_POSITION, RollConstants.ALGAE2_POSITION, IntakeMode.KICK),
        CORAL_STATION(ExtenderConstants.CORAL_STATION_HEIGHT, ArmConstants.CORAL_STATION_POSITION, PitchConstants.CORAL_STATION_POSITION, RollConstants.CORAL_STATION_POSITION, IntakeMode.IN),
        GROUND_PICKUP(ExtenderConstants.GROUND_PICKUP_HEIGHT, ArmConstants.GROUND_PICKUP_POSITION, PitchConstants.GROUND_PICKUP_POSITION, RollConstants.GROUND_PICKUP_POSITION, IntakeMode.IN);

        public final double extenderSetpoint;
        public final double armSetpoint;
        public final double pitchSetpoint;
        public final double rollSetpoint;
        public final IntakeMode intakeMode;

        private SuperstructureMode(double extenderSetpoint, double armSetpoint, double pitchSetpoint, double rollSetpoint, IntakeMode intakeMode) {
            this.extenderSetpoint = extenderSetpoint;
            this.armSetpoint = armSetpoint;
            this.pitchSetpoint = pitchSetpoint;
            this.rollSetpoint = rollSetpoint;
            this.intakeMode = intakeMode;
        }
    }

    private ExtenderSubsystem extenderSubsystem;
    private ArmSubsystem armSubsystem;
    private PitchSubsystem pitchSubsystem;
    private RollSubsystem rollSubsystem;
    private IntakeSubsystem intakeSubsystem;

    public Superstructure(ExtenderSubsystem extenderSubsystem, ArmSubsystem armSubsystem, PitchSubsystem pitchSubsystem, RollSubsystem rollSubsystem, IntakeSubsystem intakeSubsystem) {
        this.extenderSubsystem = extenderSubsystem;
        this.armSubsystem = armSubsystem;
        this.pitchSubsystem = pitchSubsystem;
        this.rollSubsystem = rollSubsystem;
        this.intakeSubsystem = intakeSubsystem;
    }


    public Command applyRequest(SuperstructureMode mode) {
        return Commands.parallel(
            extenderSubsystem.run(mode.extenderSetpoint),
            armSubsystem.run(mode.armSetpoint),
            pitchSubsystem.run(mode.pitchSetpoint),
            rollSubsystem.run(mode.rollSetpoint),
            intakeSubsystem.run(mode.intakeMode),
            Commands.run(() -> DataLogManager.log("Superstructue mode: " + mode.toString() + " applied"))
        );
    }


}
