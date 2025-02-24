package frc.robot;

public class Constants {



    public class WristConstants {
        public class RollConstants {
            public static final int MOTOR_ID = 13;

            public static final int ENCODER_ID = 2;
            public static final double ENCODER_OFFSET = .21; 

            public static double kP = 0.002;
            public static double kI = 0.0;
            public static double kD = 0.0;

            //TODO: Figure out setpoints

            public static final double INTAKE_POSITION = 160;
            public static final double SCORE_POSITION = 250;
        }
        
        public class PitchConstants {
            public static final int MOTOR_ID = 15;

            public static final int ENCODER_ID = 3;
            public static final double ENCODER_OFFSET = .16;
        
            public static double kP = 0.007;
            public static double kI = 0.0;
            public static double kD = 0.0;

            public static final double INTAKE_POSITION = 135;
            public static final double SCORE_POSITION = 30;
            public static final double BASE_POSITION = 80;
        }
    }

    public class IntakeConstants {
        public static int INTAKE_MOTOR = 14;
        
        public enum Mode {
            CORAL,
            ALGAE,
            OUT,
            IDLE
        }
    }

    public class ArmConstants {
        public static final int MOTOR_ID_1 = 10;
        public static final int MOTOR_ID_2 = 11;

        public static final int ENCODER_ID = 0;

        public static final double kP = 0.035;
        public static final double kI = 0.0;
        public static final double kD = 0.000005;

        public static final double ENCODER_OFFSET = .29;

        public static final double INTAKE_POSITION = 0;


        public static final double L1_POSITION = 0;
        public static final double L2_POSITION = 0;
        public static final double L3_POSITION = 0;
        public static final double L4_POSITION = 0;

        public static final double BASE_POSITION = 0;
    }

    public class ExtenderConstants {
        public static final int MOTOR_ID = 12;
        public static final int ENCODER_ID = 1;

        public static final double ENCODER_OFFSET = .09;

        public static double kP = 0.0;
        public static double kI = 0.0;
        public static double kD = 0.0;

        public static final double L2_HEIGHT = 0.0;
        public static final double L3_HEIGHT = 0.0;
        public static final double L4_HEIGHT = 0.0;
        public static final double BASE_HEIGHT = 0.0;

        public static final double INCHES_PER_ROTATION = .125;
        public static final double MAX_EXTENSION_INCHES = 44.0;

        public enum ExtenderHeights {
            BASE,
            L2,
            L3,
            L4
        }
    }

    public class VisionConstants {
        public static final String TARGET_CAMERA = "Arducam_OV9281_USB_Camera (1)";
        
        //Vision Rotation PID vars
        public static double VR_Kp = 0.07;
        public static double VR_Ki = 0.0;
        public static double VR_Kd = 0.0;

        //Vision Drive PID vars
        public static double VY_Kp = 1;
        public static double VY_Ki = 0.0;
        public static double VY_Kd = 0.0;

        //Vision Drive PID var
        public static double VX_Kp = 0.5;
        public static double VX_Ki = 0.0;
        public static double VX_Kd = 0.0;
    }

    public class LedConstants {
        public static final int CANdle_COUNT = 8;
        public static final int ARM_L_COUNT = 50;
        public static final int ARM_R_COUNT = 100;
        public static final int ARM_STATUS = 108;
    }
}
