package frc.robot.subsystems.drive;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public abstract class SwerveModule extends SubsystemBase {

    public abstract double getDriveDistanceMeters();

    public abstract double getDriveSpeedMetersPerSec();

    public abstract Rotation2d getSteerAngle();

    public abstract double getSteerSpeedRadPerSec();

    public abstract Rotation2d getCanCoderAngle();

    public abstract SwerveModulePosition getModulePosition();
    
    public abstract SwerveModuleState getModuleState();

    public abstract void resetSteerEncAngle();

    public abstract void setState(SwerveModuleState desiredState);

    public abstract void applyCharacterizationVoltage(double volts);
}
