package frc.robot.subsystems.superstructure;

import frc.robot.util.constants.RobotConstants.ArmConstants;
import frc.robot.util.constants.RobotConstants.ExtenderConstants;
import frc.robot.util.constants.RobotConstants.IntakeConstants.IntakeMode;
import frc.robot.util.constants.RobotConstants.WristConstants.PitchConstants;
import frc.robot.util.constants.RobotConstants.WristConstants.RollConstants;

public enum SuperstructureState {
    BASE(ExtenderConstants.BASE_HEIGHT, ArmConstants.BASE_POSITION, PitchConstants.BASE_POSITION, RollConstants.BASE_POSITION, IntakeMode.STOP),
    L1(ExtenderConstants.L1_HEIGHT, ArmConstants.L1_POSITION, PitchConstants.L1_POSITION, RollConstants.L1_POSITION, IntakeMode.STOP),
    L2(ExtenderConstants.L2_HEIGHT, ArmConstants.L2_POSITION, PitchConstants.L2_POSITION, RollConstants.L2_POSITION, IntakeMode.STOP),
    L3(ExtenderConstants.L3_HEIGHT, ArmConstants.L3_POSITION, PitchConstants.L3_POSITION, RollConstants.L3_POSITION, IntakeMode.STOP),
    L4(ExtenderConstants.L4_HEIGHT, ArmConstants.L4_POSITION, PitchConstants.L4_POSITION, RollConstants.L4_POSITION, IntakeMode.STOP),
    LOW_ALGAE_PICKUP(ExtenderConstants.LOW_ALGAE_PICKUP_HEIGHT, ArmConstants.LOW_ALGAE_PICKUP_POSITION, PitchConstants.LOW_ALGAE_PICKUP_POSITION, RollConstants.LOW_ALGAE_PICKUP_POSITION, IntakeMode.IN),
    HIGH_ALGAE_PICKUP(ExtenderConstants.HIGH_ALGAE_PICKUP_HEIGHT, ArmConstants.HIGH_ALGAE_PICKUP_POSITION, PitchConstants.HIGH_ALGAE_PICKUP_POSITION, RollConstants.HIGH_ALGAE_PICKUP_POSITION, IntakeMode.IN),
    LOW_ALGAE_CLEAR(ExtenderConstants.LOW_ALGAE_CLEAR_HEIGHT, ArmConstants.LOW_ALGAE_CLEAR_POSITION, PitchConstants.LOW_ALGAE_CLEAR_POSITION, RollConstants.LOW_ALGAE_CLEAR_POSITION, IntakeMode.KICK),
    HIGH_ALGAE_CLEAR(ExtenderConstants.HIGH_ALGAE_CLEAR_HEIGHT, ArmConstants.HIGH_ALGAE_CLEAR_POSITION, PitchConstants.HIGH_ALGAE_CLEAR_POSITION, RollConstants.HIGH_ALGAE_CLEAR_POSITION, IntakeMode.KICK),
    CORAL_STATION(ExtenderConstants.CORAL_STATION_HEIGHT, ArmConstants.CORAL_STATION_POSITION, PitchConstants.CORAL_STATION_POSITION, RollConstants.CORAL_STATION_POSITION, IntakeMode.IN),
    GROUND_PICKUP(ExtenderConstants.GROUND_PICKUP_HEIGHT, ArmConstants.GROUND_PICKUP_POSITION, PitchConstants.GROUND_PICKUP_POSITION, RollConstants.GROUND_PICKUP_POSITION, IntakeMode.IN),
    PROCESSOR(ExtenderConstants.PROCESSOR_HEIGHT, ArmConstants.PROCESSOR_POSITION, PitchConstants.PROCESSOR_POSITION, RollConstants.PROCESSOR_POSITION, IntakeMode.STOP),
    NET(ExtenderConstants.NET_HEIGHT, ArmConstants.NET_POSITION, PitchConstants.NET_POSITION, RollConstants.NET_POSITION, IntakeMode.STOP);

    public final double extenderSetpoint;
    public final double armSetpoint;
    public final double pitchSetpoint;
    public final double rollSetpoint;
    public final IntakeMode intakeMode;

    private SuperstructureState(double extenderSetpoint, double armSetpoint, double pitchSetpoint, double rollSetpoint, IntakeMode intakeMode) {
        this.extenderSetpoint = extenderSetpoint;
        this.armSetpoint = armSetpoint;
        this.pitchSetpoint = pitchSetpoint;
        this.rollSetpoint = rollSetpoint;
        this.intakeMode = intakeMode;
    }
}
