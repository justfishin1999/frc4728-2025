package frc.robot.commands;

import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.LimelightHelpers;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.Command;

public class AutoAlign_Right extends Command {

    private final CommandSwerveDrivetrain drivetrain;
    private final SwerveRequest.RobotCentric limeDrive;
    private double alignmentSpeed, m_xspeed;
    private int m_pipeline;

    // Constructor accepts limeDrive as a parameter from RobotContainer
    public AutoAlign_Right(CommandSwerveDrivetrain drivetrain, SwerveRequest.RobotCentric limeDrive, double alignmentSpeed, int pipeline) {
        this.drivetrain = drivetrain;
        this.limeDrive = limeDrive;  // Use the limeDrive from RobotContainer
        this.alignmentSpeed = alignmentSpeed;
        this.m_pipeline = pipeline;
        addRequirements(drivetrain);  // Ensure the drivetrain is required for this command
    }

    @Override
    public void initialize() {
        System.out.println("Caught auto alignment command");
        NetworkTableInstance.getDefault().getTable("limelight").getEntry("pipeline").setDouble(m_pipeline);
    }

    @Override
    public void execute() {
        m_xspeed = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tx").getDouble(0.0)*alignmentSpeed;

        drivetrain.applyRequest(() -> limeDrive.withVelocityY(-m_xspeed));
        //drivetrain.setControl(limeDrive.withVelocityY(m_xspeed));
    }

    @Override
    public boolean isFinished() {
        // The command finishes when tx is sufficiently close to zero (aligned)
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        // Stop robot motion after alignment
}
}
