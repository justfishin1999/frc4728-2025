package frc.robot.commands;

import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.LimelightHelpers;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;

import edu.wpi.first.wpilibj2.command.Command;

public class AutoAlign_Right extends Command {

    private final CommandSwerveDrivetrain drivetrain;
    private final SwerveRequest.RobotCentric limeDrive;
    private final double alignmentSpeed;

    // Constructor accepts limeDrive as a parameter from RobotContainer
    public AutoAlign_Right(CommandSwerveDrivetrain drivetrain, SwerveRequest.RobotCentric limeDrive, double alignmentSpeed) {
        this.drivetrain = drivetrain;
        this.limeDrive = limeDrive;  // Use the limeDrive from RobotContainer
        this.alignmentSpeed = alignmentSpeed;
        addRequirements(drivetrain);  // Ensure the drivetrain is required for this command
    }

    @Override
    public void initialize() {
        System.out.println("Caught auto alignment command");
        // Optionally, you can initialize the pipeline or perform other tasks here if needed.
    }

    @Override
    public void execute() {
        // Get the horizontal offset (tx) from Limelight
        double tx = LimelightHelpers.getTX("limelight");
        double ty = LimelightHelpers.getTY("limelight");
        System.out.println("Mid auto align command");

        //PPHolonomicDriveController.overrideXFeedback(()->-tx*alignmentSpeed);
        PPHolonomicDriveController.overrideXYFeedback(()->-tx*alignmentSpeed,()->-ty*alignmentSpeed);
    }

    @Override
    public boolean isFinished() {
        // The command finishes when tx is sufficiently close to zero (aligned)
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        // Stop robot motion after alignment
        PPHolonomicDriveController.clearXYFeedbackOverride();
}
}
