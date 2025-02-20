package frc.robot;

public class Constants {
    public class IntakeConstants {
        public static int INTAKE_MOTOR = 25;    
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
}
