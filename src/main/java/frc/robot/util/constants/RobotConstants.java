package frc.robot.util.constants;

import static edu.wpi.first.units.Units.Centimeters;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Radians;

import java.util.Set;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;


public class RobotConstants {
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

            public static final double LOW_ALGAE_PICKUP_POSITION = CORAL_STATION_POSITION;
            public static final double HIGH_ALGAE_PICKUP_POSITION = CORAL_STATION_POSITION;

            public static final double LOW_ALGAE_CLEAR_POSITION = CORAL_STATION_POSITION;
            public static final double HIGH_ALGAE_CLEAR_POSITION = CORAL_STATION_POSITION;  

            public static final double PROCESSOR_POSITION = CORAL_STATION_POSITION;
            public static final double NET_POSITION = CORAL_STATION_POSITION;

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

            public static final double LOW_ALGAE_PICKUP_POSITION = 110;
            public static final double HIGH_ALGAE_PICKUP_POSITION = 137;

            public static final double LOW_ALGAE_CLEAR_POSITION = 110;
            public static final double HIGH_ALGAE_CLEAR_POSITION = 137;

            public static final double PROCESSOR_POSITION = 120;
            public static final double NET_POSITION = 120;

            public static final double BASE_POSITION = 116;

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

        public static final double LOW_ALGAE_PICKUP_POSITION = L1_POSITION;
        public static final double HIGH_ALGAE_PICKUP_POSITION = 102;

        public static final double LOW_ALGAE_CLEAR_POSITION = L1_POSITION;
        public static final double HIGH_ALGAE_CLEAR_POSITION = 102;

        public static final double PROCESSOR_POSITION = 100;
        public static final double NET_POSITION = 100;

        public static final double BASE_POSITION = 94;
    }

    public class ExtenderConstants {
        public static final int MOTOR_ID = 12;
        public static final int ENCODER_ID = 1;
        public static final int LIMIT_SWITCH_ID = 4;

        public static double kP = 2.5;
        public static double kI = 0.0;
        public static double kD = 0.0;

        public static final double TOLERANCE = 0.25;

        public static final double INCHES_PER_ROTATION = .5;
        public static final double MAX_EXTENSION = 18;
        public static final double MIN_EXTENSION = 0.5;

        public static final double BASE_HEIGHT = 0.5;

        public static final double CORAL_STATION_HEIGHT = BASE_HEIGHT;
        public static final double GROUND_PICKUP_HEIGHT = BASE_HEIGHT;

        public static final double L1_HEIGHT = BASE_HEIGHT;
        public static final double L2_HEIGHT = BASE_HEIGHT;
        public static final double L3_HEIGHT = 6.22 - .25;
        public static final double L4_HEIGHT = 14;

        public static final double GROUND_PICKUP = 0.15;

        public static final double LOW_ALGAE_PICKUP_HEIGHT = L1_HEIGHT;
        public static final double HIGH_ALGAE_PICKUP_HEIGHT = 2.27;

        public static final double LOW_ALGAE_CLEAR_HEIGHT = L1_HEIGHT;
        public static final double HIGH_ALGAE_CLEAR_HEIGHT = 2.77;

        public static final double PROCESSOR_HEIGHT = 0.5;
        public static final double NET_HEIGHT = 0.5;

    }

    public class VisionConstants {
        public static final String LEFT_CAMERA = "LEFT_CAMERA";
        public static final String RIGHT_CAMERA = "RIGHT_CAMERA";

        public static final AprilTagFieldLayout layout = AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeWelded);

        public static final Transform3d CAMERA_TO_ROBOT_RIGHT = new Transform3d(new Translation3d(0.127, 0.17145, 0.3175),
            new Rotation3d(0, 0, 0));

        public static final Transform3d CAMERA_TO_ROBOT_LEFT = new Transform3d(new Translation3d(0.127, -0.17145, 0.3175),
            new Rotation3d(0, 0, 0));

        public static final double MAX_DISTANCE_METERS = 3;

        public static final double X_STANDARD_DEVIATION = 0.1;
        public static final double Y_STANDARD_DEVIATION = 0.1;
        public static final double THETA_STANDARD_DEVIATION = 0.1;

        // Vision Rotation PID vars
        public static final double VR_kI = 0.0;
        public static final double VR_kP = 0.07;
        public static final double VR_kD = 0.0;

        // Vision Drive PID vars
        public static final double VY_kP = 0.8;
        public static final double VY_kI = 0.0;
        public static final double VY_kD = 0.0;

        // Vision Drive PID var
        public static final double VX_kP = 0.025;
        public static final double VX_kI = 0.0;
        public static final double VX_kD = 0.0;

        public static final double POSITION_TOLERANCE_METERS = Centimeters.of(5).in(Meters);
        public static final double ROTATION_TOLERANCE_RADIANS = Degrees.of(2).in(Radians); 

        public static final Set<Integer> HIGH_ALGAE_TAGS = Set.of(20, 18, 22, 9, 7, 11);
        public static final Set<Integer> LOW_ALGAE_TAGS = Set.of(22,19, 17, 10, 6, 8);

    }

    public class LedConstants {
        public static final int CANdle_COUNT = 8;
        public static final int ARM_L_COUNT = 50;
        public static final int ARM_R_COUNT = 100;
        public static final int ARM_STATUS = 108;
    }
}
