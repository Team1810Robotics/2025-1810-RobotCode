package frc.robot;

public class Constants {



    public class WristConstants {
        public class RollConstants {
            public static final int MOTOR_ID = 13;

            public static final int ENCODER_ID = 2;
            public static final double ENCODER_OFFSET = .3; 

            public static double kP = 0.0175;
            public static double kI = 0.0;
            public static double kD = 0.0;

            //TODO: Figure out setpoints more

            //public static final double INTAKE_POSITION = 65;
            public static final double SCORE_POSITION = 163;
            
            public static final double INTAKE_POSITION = 68;
            public static final double GROUND_PICKUP = 66.95;
            public static final double L1_POSITION = 68.0;
            public static final double L2_POSITION = 158.0;
            public static final double L3_POSITION = 158.0;
            public static final double L4_POSITION = 158.0;
            public static final double BASE_POSITION = 68; //Check This
        }
        
        public class PitchConstants {
            public static final int MOTOR_ID = 15;

            public static final int ENCODER_ID = 3;
            public static final double ENCODER_OFFSET = .319 - (5.0/360.0);
        
            public static double kP = 0.007;
            public static double kI = 0.0;
            public static double kD = 0.0;

            public static final double INTAKE_POSITION = 135; // 45
            public static final double GROUND_PICKUP = 145;
            public static final double L1_POSITION = 135;
            public static final double L2_POSITION = 135;
            public static final double L3_POSITION = 135;
            public static final double L4_POSITION = 135;
            public static final double BASE_POSITION = 125; //Check This


            public static final double UPRIGHT = 90;
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

        public static final double kP = 0.045;
        public static final double kI = 0.0;
        public static final double kD = 0.000001;

        public static final double ENCODER_OFFSET = .3034;

        public static final double INTAKE_POSITION = 105; //75 


        public static final double GROUND_PICKUP = 183;
        public static final double L1_POSITION = 145;
        public static final double L2_POSITION = 105;
        public static final double L3_POSITION = 99;
        public static final double L4_POSITION = 94.6;

        public static final double BASE_POSITION = 115;
    }

    public class ExtenderConstants {
        public static final int MOTOR_ID = 12;
        public static final int ENCODER_ID = 1;

        public static final double ENCODER_OFFSET = .23;

        public static double kP = 1;
        public static double kI = 0.0;
        public static double kD = 0.0;


        public static final double GROUND_PICKUP = 0.0;
        public static final double L1_HEIGHT = 0.5;
        public static final double L2_HEIGHT = 0.1;
        public static final double L3_HEIGHT = 6;
        public static final double L4_HEIGHT = 13;
        public static final double BASE_HEIGHT = 0;

        public static final double INCHES_PER_ROTATION = .5;
        public static final double MAX_EXTENSION_INCHES = 44.0;

    }

    public class VisionConstants {
        public static final String TARGET_CAMERA_RIGHT = "Arducam_OV9281_USB_Camera (1)";
        public static final String TARGET_CAMERA_LEFT = "Arducam_OV9281_USB_Camera";
        
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
