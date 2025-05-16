package frc.robot.commands;

import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecondPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecondPerSecond;

import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.util.constants.RobotConstants.VisionConstants;

public class DriveToPose extends Command {
    private final CommandSwerveDrivetrain drivetrain;
    private final SwerveRequest.RobotCentric drive;
    private final Pose2d targetPose;
    
    private final PIDController xController;
    private final PIDController yController;
    private final ProfiledPIDController thetaController;
    
    
    /**
     * Creates a new command to drive to a specific pose.
     * 
     * @param drivetrain The drive subsystem to use
     * @param targetPose The target pose to drive to
     */
    public DriveToPose(CommandSwerveDrivetrain drivetrain, SwerveRequest.RobotCentric drive, Pose2d targetPose) {
        this.drivetrain = drivetrain;
        this.targetPose = targetPose;
        this.drive = drive;
        
        xController = new PIDController(VisionConstants.VX_kP, VisionConstants.VX_kI, VisionConstants.VX_kD);
        yController = new PIDController(VisionConstants.VY_kP, VisionConstants.VY_kI, VisionConstants.VY_kD);
        
        thetaController = new ProfiledPIDController(
            VisionConstants.VR_kP,
            VisionConstants.VR_kI,
            VisionConstants.VR_kD,
            new TrapezoidProfile.Constraints(RotationsPerSecond.of(.5).in(RadiansPerSecond), 
                                            RotationsPerSecondPerSecond.of(0.5).in(RadiansPerSecondPerSecond)
                                            )
        );
        
        thetaController.enableContinuousInput(-Math.PI, Math.PI);
        
        xController.setTolerance(VisionConstants.POSITION_TOLERANCE_METERS);
        yController.setTolerance(VisionConstants.POSITION_TOLERANCE_METERS);
        thetaController.setTolerance(VisionConstants.ROTATION_TOLERANCE_RADIANS);
        
        addRequirements(drivetrain);
    }
    
    @Override
    public void initialize() {
        xController.reset();
        yController.reset();
        thetaController.reset(drivetrain.getState().Pose.getRotation().getRadians());
        
    }
    
    @Override
    public void execute() {
        Pose2d currentPose = drivetrain.getState().Pose;
        
        double xSpeed = xController.calculate(currentPose.getX(), targetPose.getX());
        double ySpeed = yController.calculate(currentPose.getY(), targetPose.getY());
        double thetaSpeed = thetaController.calculate(
            currentPose.getRotation().getRadians(), 
            targetPose.getRotation().getRadians()
        );

    
        drivetrain.setControl(
            drive.
                withVelocityX(xSpeed)
                .withVelocityY(ySpeed)
                .withRotationalRate(thetaSpeed)
        );
    }
    
    @Override
    public boolean isFinished() {
        // Command is finished when we're at the target position and rotation within tolerance
        return xController.atSetpoint() && 
               yController.atSetpoint() && 
               thetaController.atSetpoint();
    }
    
    @Override
    public void end(boolean interrupted) {
        // Stop the robot when the command ends
        drivetrain.setControl(
            drive.withVelocityX(0).withVelocityY(0).withRotationalRate(0)
        );
    }
}