package frc.robot.subsystems;

import java.util.function.Supplier;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class PoseEstimator extends SubsystemBase {

    public static class Constants {
        public static final double odomTranslationStdDevMeters = 0.05;
        public static final double odomRotationStdDevRad = Units.degreesToRadians(0.25);

        public static final double visionTranslationStdDevMeters = 0.35;
        public static final double visionRotationStdDevRad = Units.degreesToRadians(30.0);
    }

    private final SwerveDrivePoseEstimator poseEstimator;

    private final Supplier<Rotation2d> gyroHeadingSupplier;
    private final Supplier<SwerveModulePosition[]> modulePositionsSupplier;

    public PoseEstimator(
        SwerveDriveKinematics swerveKinematics,
        Supplier<Rotation2d> gyroHeadingSupplier,
        Supplier<SwerveModulePosition[]> modulePositionsSupplier) {

        poseEstimator = new SwerveDrivePoseEstimator(
            swerveKinematics,
            gyroHeadingSupplier.get(),
            modulePositionsSupplier.get(),
            new Pose2d(),
            VecBuilder.fill(
                Constants.odomTranslationStdDevMeters,
                Constants.odomTranslationStdDevMeters,
                Constants.odomRotationStdDevRad),
            VecBuilder.fill(
                Constants.visionTranslationStdDevMeters,
                Constants.visionTranslationStdDevMeters,
                Constants.visionRotationStdDevRad));

        this.gyroHeadingSupplier = gyroHeadingSupplier;
        this.modulePositionsSupplier = modulePositionsSupplier;
    }

    @Override
    public void periodic() {
        poseEstimator.update(gyroHeadingSupplier.get(), modulePositionsSupplier.get());
    }

    public Pose2d get() {
        return poseEstimator.getEstimatedPosition();
    }

    public void addVisionMeasurement(Pose2d visionPose, double captureTimestampSecs) {
        poseEstimator.addVisionMeasurement(visionPose, captureTimestampSecs);
    }

    public void updatePose(Pose2d pose) {
        poseEstimator.resetPosition(gyroHeadingSupplier.get(), modulePositionsSupplier.get(), pose);
    }
    
}
