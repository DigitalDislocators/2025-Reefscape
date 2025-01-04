package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.SwerveDriveConstants;
import frc.robot.subsystems.PoseEstimator;
import frc.robot.subsystems.drive.SwerveDrive;

public class ArcadeDriveCmd extends Command {

    /**
     * Command to allow for driver input in teleop
     * Can't be inlined efficiently if we want to edit the inputs in any way (deadband, square, etc.)
     */

    private final SwerveDrive swerve;
    private final PoseEstimator poseEstimator;

    /**
     * Joysticks return DoubleSuppliers when the get methods are called
     * This is so that joystick getter methods can be passed in as a parameter but will continuously update, 
     * versus using a double which would only update when the constructor is called
     */
    private final DoubleSupplier drive;
    private final DoubleSupplier strafe;
    private final DoubleSupplier rot;
    
    private final boolean squareInputs;

    /**
     * Constructs a new ArcadeDriveCmd.
     * 
     * <p>ArcadeDriveCmd is used to control the swerve drive base with arcade drive.
     * 
     * @param drive The commanded forward/backward lateral motion.
     * @param strafe The commanded left/right lateral motion.
     * @param rot The commanded rotational motion.
     * @param squareInputs Should be true if the commanded inputs are to be squared.
     * @param swerve The required SwerveDrive.
     */
    public ArcadeDriveCmd(
        DoubleSupplier drive, 
        DoubleSupplier strafe, 
        DoubleSupplier rot,
        boolean squareInputs,
        SwerveDrive swerve,
        PoseEstimator poseEstimator
    ) {
        this.swerve = swerve;
        this.poseEstimator = poseEstimator;

        this.drive = drive;
        this.strafe = strafe;
        this.rot = rot;

        this.squareInputs = squareInputs;

        addRequirements(swerve);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {}

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        double drive = this.drive.getAsDouble();
        double strafe = this.strafe.getAsDouble();
        double rot = this.rot.getAsDouble();

        if(squareInputs) {
            // square inputs while preserving commanded lateral direction
            double theta = Math.atan2(drive, strafe);
            double r = Math.pow(Math.hypot(drive, strafe), 2.0);

            drive = r * Math.sin(theta);
            strafe = r * Math.cos(theta);

            rot = Math.copySign(Math.pow(rot, 2.0), rot);
        }

        swerve.driveFieldRelative(
            -drive * SwerveDriveConstants.maxAttainableSpeedMetersPerSec,
            -strafe * SwerveDriveConstants.maxAttainableRotationRadPerSec,
            -rot * SwerveDriveConstants.maxAttainableRotationRadPerSec,
            poseEstimator.get().getRotation()
        );
    }
    
    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {}
    
    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return true;
    }
}