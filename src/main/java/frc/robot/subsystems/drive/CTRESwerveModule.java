package frc.robot.subsystems.drive;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.MagnetSensorConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.robot.Constants.SwerveModuleConstants;

public class CTRESwerveModule extends SwerveModule {

    private final TalonFX driveMtr;
    private final TalonFX steerMtr;

    private final CANcoder canCoder;

    private final Rotation2d moduleOffset;

    
    private final VelocityVoltage driveRequest = new VelocityVoltage(0).withSlot(0);
    private final PositionVoltage steerRequest = new PositionVoltage(0).withSlot(0);

    // FIXME This class is currently untested. I am not super familiar with Phoenix6 and have no way to test this currently.
    public CTRESwerveModule(int canCoderID, int driveMtrID, int steerMtrID, Rotation2d moduleOffset) {
        this.moduleOffset = moduleOffset;

        driveMtr = new TalonFX(driveMtrID);
        steerMtr = new TalonFX(steerMtrID);

        TalonFXConfiguration driveMtrConfig = new TalonFXConfiguration();
        TalonFXConfiguration steerMtrConfig = new TalonFXConfiguration();

        driveMtrConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        steerMtrConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;

        driveMtrConfig.CurrentLimits.SupplyCurrentLimit = SwerveModuleConstants.driveCurrentLimitAmps;
        driveMtrConfig.CurrentLimits.SupplyCurrentLimitEnable = true;

        steerMtrConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

        // FIXME check if correct
        driveMtrConfig.Feedback.SensorToMechanismRatio = 1 / SwerveModuleConstants.driveMetersPerEncRev;
        steerMtrConfig.Feedback.SensorToMechanismRatio = 1 / SwerveModuleConstants.steerRadiansPerEncRev;

        driveMtrConfig.Slot0.kP = SwerveModuleConstants.driveKp;
        steerMtrConfig.Slot0.kP = SwerveModuleConstants.steerKp;

        driveMtrConfig.Slot0.kD = SwerveModuleConstants.driveKd;
        steerMtrConfig.Slot0.kD = SwerveModuleConstants.steerKd;

        steerMtrConfig.ClosedLoopGeneral.ContinuousWrap = true;

        driveMtr.getConfigurator().apply(driveMtrConfig);
        steerMtr.getConfigurator().apply(steerMtrConfig);

        canCoder = new CANcoder(canCoderID);

        canCoder.getConfigurator().apply(new CANcoderConfiguration()
            .withMagnetSensor(new MagnetSensorConfigs()
                .withSensorDirection(SensorDirectionValue.CounterClockwise_Positive)));

        resetSteerEncAngle();
    }

    public double getDriveDistanceMeters() {
        return driveMtr.getPosition().getValueAsDouble();
    }

    public double getDriveSpeedMetersPerSec() {
        return driveMtr.getVelocity().getValueAsDouble();
    }

    public Rotation2d getSteerAngle() {
        Rotation2d rot = Rotation2d.fromRadians(steerMtr.getPosition().getValueAsDouble());
        if (rot.getRadians() < 0.0) {
            rot = rot.plus(new Rotation2d(2.0 * Math.PI));
        }
        return rot;
    }

    public double getSteerSpeedRadPerSec() {
        return steerMtr.getVelocity().getValueAsDouble();
    }

    public Rotation2d getCanCoderAngle() {
        return Rotation2d.fromRotations(canCoder.getAbsolutePosition().getValueAsDouble());
    }

    public SwerveModulePosition getModulePosition() {
        return new SwerveModulePosition(getDriveDistanceMeters(), getSteerAngle());
    }  
    
    public SwerveModuleState getModuleState() {
        return new SwerveModuleState(getDriveSpeedMetersPerSec(), getSteerAngle());
    }

    public void resetSteerEncAngle() {
        steerMtr.setPosition(getCanCoderAngle().getRadians() - moduleOffset.getRadians());
    }

    public void setState(SwerveModuleState desiredState) {
        // optimize speed and angle to minimize change in heading
        // (e.g. module turns 1 degree and reverses drive direction to get from 90 degrees to -89 degrees)
        desiredState.optimize(getSteerAngle());

        // scale velocity based on turn error to help prevent skew
        double angleErrorRad = desiredState.angle.getRadians() - getSteerAngle().getRadians();
        desiredState.speedMetersPerSecond *= Math.cos(angleErrorRad);

        driveMtr.setControl(driveRequest.withVelocity(desiredState.speedMetersPerSecond).withFeedForward(SwerveModuleConstants.driveFF.calculate(desiredState.speedMetersPerSecond)));
        steerMtr.setControl(steerRequest.withPosition(desiredState.angle.getRadians()));
    }

    public void applyCharacterizationVoltage(double volts) {
        driveMtr.setVoltage(volts);
        steerMtr.setControl(steerRequest.withPosition(0.0));
    }
}
