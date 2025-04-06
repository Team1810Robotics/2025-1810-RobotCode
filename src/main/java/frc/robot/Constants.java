package frc.robot;

import static edu.wpi.first.units.Units.Meters;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.units.measure.Distance;
import frc.robot.Constants.IntakeConstants.IntakeMode;
import frc.robot.Constants.WristConstants.PitchConstants;
import frc.robot.Constants.WristConstants.RollConstants;

public class Constants {
    public class SuperstructueConstants {
        public static enum SuperstructureState {
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

            private SuperstructureState(double extenderSetpoint, double armSetpoint, double pitchSetpoint, double rollSetpoint, IntakeMode intakeMode) {
                this.extenderSetpoint = extenderSetpoint;
                this.armSetpoint = armSetpoint;
                this.pitchSetpoint = pitchSetpoint;
                this.rollSetpoint = rollSetpoint;
                this.intakeMode = intakeMode;
            }
        }
    }
    public class WristConstants {
        public class RollConstants {
            public static final int MOTOR_ID = 13;

            public static final int ENCODER_ID = 2;
            public static final double ENCODER_OFFSET = .1;

            public static double kPI = 0.055;
            public static double kII = 0.0;
            public static double kDI = 0.00005;

            public static double kPD = 0.0055;
            public static double kID = 0.0;
            public static double kDD = 0.0;

            public static final double TOLERANCE = 2.0;

            public static final double CORAL_STATION_POSITION = 140;
            public static final double GROUND_PICKUP_POSITION = CORAL_STATION_POSITION;

            public static final double L1_POSITION = CORAL_STATION_POSITION;
            public static final double L2_POSITION = 230;
            public static final double L3_POSITION = L2_POSITION;
            public static final double L4_POSITION = L2_POSITION;

            public static final double ALGAE1_POSITION = CORAL_STATION_POSITION;
            public static final double ALGAE2_POSITION = CORAL_STATION_POSITION;

            public static final double BASE_POSITION = CORAL_STATION_POSITION;
        }

        public class PitchConstants {
            public static final int MOTOR_ID = 15;

            public static final int ENCODER_ID = 3;
            public static final double ENCODER_OFFSET = 0;

            public static double kP = 0.0075;
            public static double kI = 0.0;
            public static double kD = 0.0;

            public static final double TOLERANCE = 2.0;

            public static final double CORAL_STATION_POSITION = 133;
            public static final double GROUND_PICKUP_POSITION = 134;

            public static final double L1_POSITION = 120;
            public static final double L2_POSITION = 124;
            public static final double L3_POSITION = 136;
            public static final double L4_POSITION = 130;
            public static final double BASE_POSITION = 116;

            public static final double ALGAE1_POSITION = 110;
            public static final double ALGAE2_POSITION = 137;

        }
    }

    public class IntakeConstants {
        public static int INTAKE_MOTOR = 14;

        public enum IntakeMode {
            IN,
            OUT,
            STOP,
            KICK
        }
    }

    public class ArmConstants {
        public static final int MOTOR_ID_1 = 10;
        public static final int MOTOR_ID_2 = 11;

        public static final int ENCODER_ID = 0;
        public static final double ENCODER_OFFSET = .465;

        public static final double kP = 0.045;
        public static final double kI = 0.0;
        public static final double kD = 0.000001;

        public static final double TOLERANCE = 2.0;

        public static final double CORAL_STATION_POSITION = 100; 

        public static final double GROUND_PICKUP_POSITION = 185;
        public static final double L1_POSITION = 145;
        public static final double L2_POSITION = 105;
        public static final double L3_POSITION = 97.5;
        public static final double L4_POSITION = 92;

        public static final double ALGAE1_POSITION = L1_POSITION;
        public static final double ALGAE2_POSITION = 102;

        public static final double BASE_POSITION = 94;
    }

    public class ExtenderConstants {
        public static final int MOTOR_ID = 12;
        public static final int ENCODER_ID = 1;
        public static final int LIMIT_SWITCH_ID = 4;

        public static double ENCODER_OFFSET = .75;

        public static double kP = 2.5;
        public static double kI = 0.0;
        public static double kD = 0.0;

        public static final double BASE_HEIGHT = 0.5;

        public static final double CORAL_STATION_HEIGHT = BASE_HEIGHT;
        public static final double GROUND_PICKUP_HEIGHT = BASE_HEIGHT;

        public static final double L1_HEIGHT = BASE_HEIGHT;
        public static final double L2_HEIGHT = BASE_HEIGHT;
        public static final double L3_HEIGHT = 6.22 - .25;
        public static final double L4_HEIGHT = 14;

        public static final double GROUND_PICKUP = 0.15;

        public static final double ALGAE1_HEIGHT = L1_HEIGHT;
        public static final double ALGAE2_HEIGHT = 2.27 + .5;

        public static final double INCHES_PER_ROTATION = .5;
        public static final double MAX_EXTENSION_INCHES = 44.0;

    }

    public class VisionConstants {
        public static final String TARGET_CAMERA_LEFT = "RIGHT_TARGET_CAM";
        public static final String TARGET_CAMERA_RIGHT = "LEFT_TARGET_CAM";

        public static final Transform3d CAMERA_TO_ROBOT_RIGHT = new Transform3d(new Translation3d(0.127, 0.17145, 0.3175),
            new Rotation3d(0, 0, 0));

        public static final Transform3d CAMERA_TO_ROBOT_LEFT = new Transform3d(new Translation3d(0.127, -0.17145, 0.3175),
            new Rotation3d(0, 0, 0));

        public static final Distance MAX_DISTANCE = Meters.of(3);

        // Vision Rotation PID vars
        public static double VR_kP = 0.07;
        public static double VR_kI = 0.0;
        public static double VR_kD = 0.0;

        // Vision Drive PID vars
        public static double VY_kP = 0.8;
        public static double VY_kI = 0.0;
        public static double VY_kD = 0.0;

        // Vision Drive PID var
        public static double VX_kP = 0.025;
        public static double VX_kI = 0.0;
        public static double VX_kD = 0.0;

    }

    public class LedConstants {
        public static final int CANdle_COUNT = 8;
        public static final int ARM_L_COUNT = 50;
        public static final int ARM_R_COUNT = 100;
        public static final int ARM_STATUS = 108;
    }
}
