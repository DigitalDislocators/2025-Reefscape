// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.config.ModuleConfig;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
    public static class RobotConstants {
        // TODO Set to the current the weight of the robot, including the battery and bumpers.
        public static final double massKg = 45.0; // ~100 lbs

        // TODO Set the frame dimensions of the robot.
        public static final double robotWidthMeters = Units.inchesToMeters(21.75);
        public static final double robotLengthMEters = Units.inchesToMeters(21.75);

        // Moment of inertia of a uniform-mass slab with the axis of rotation centered and perpendicular to the slab
        // This should be a reasonable approximation of the robot's MOI
        public static final double momentOfInertiaKgMetersSq = massKg * (Math.pow(robotWidthMeters, 2) + Math.pow(robotLengthMEters, 2)) / 12;
    }


	public static class OperatorConstants {
		public static final int kDriverControllerPort = 0;
        public static final int operatorGamepadport = 1;

		public static final double joystickDeadband = 0.15;
        public static final double tiggerPressedThreshold = 0.25;
	}

	public static class CANDevices {
		public static final int pigeonID = 14;

		public static final int flModuleCANCoderID = 2;
		public static final int flModuleDriveMtrID = 6;
		public static final int flModuleSteerMtrID = 10;

		public static final int frModuleCANCoderID = 3;
		public static final int frModuleDriveMtrID = 7;
		public static final int frModuleSteerMtrID = 11;

		public static final int blModuleCANCoderID = 4;
		public static final int blModuleDriveMtrID = 8;
		public static final int blModuleSteerMtrID = 12;

		public static final int brModuleCANCoderID = 5;
		public static final int brModuleDriveMtrID = 9;
		public static final int brModuleSteerMtrID = 13;

        // left Mtr is leader and right Mtr is Follower
        public static final int leftElevatorMtrID = 15;
        public static final int rightElevatorMtrID = 16;
	}

	public static class SwerveModuleConstants {
        // TODO Tune the below PID and FF values using the SysID routines.
        public static final double driveKp = 0.13;
        public static final double driveKd = 0.0;

        public static final double steerKp = 0.37431;
        public static final double steerKd = 0.27186;

        public static final double driveKsVolts = 0.667;
        public static final double driveKvVoltSecsPerMeter = 2.44;
        public static final double driveKaVoldSecsPerMeterSq = 0.0;

        public static final SimpleMotorFeedforward driveFF = new SimpleMotorFeedforward(driveKsVolts, driveKvVoltSecsPerMeter, driveKaVoldSecsPerMeterSq);

        // TODO You may want to change this value depending on your breakers and the current usage of the rest of your robot.
        public static final int driveCurrentLimitAmps = 40;

        // TODO This number may have to be adjusted depending on what wheels you use.
        public static final double wheelRadiusMeters = Units.inchesToMeters(2.0);

        // TODO Set this value to the coefficient of friction of your wheels.
        public static final double wheelCoefficientOfFriction = 1.5;

        public static final double driveGearReduction = (16.0 / 50.0) * (27.0 / 17.0) * (15.0 / 45.0);

        public static final double driveMetersPerEncRev = driveGearReduction * 2.0 * wheelRadiusMeters * Math.PI;

        public static final double driveMetersPerSecPerEncRPM = driveMetersPerEncRev / 60.0;

        public static final double steerGearReduction = (15.0 / 32.0) * (10.0 / 60.0);

        public static final double steerRadiansPerEncRev = steerGearReduction * 2.0 * Math.PI;

        public static final double steerRadiansPerSecPerEncRPM = steerRadiansPerEncRev / 60.0;
        
        
        public static final double driveFreeSpeedMetersPerSec = Units.feetToMeters(20.1);

        public static final double driveFreeSpeedRadPerSec = driveFreeSpeedMetersPerSec / wheelRadiusMeters;

        public static final double driveNominalOperatingVoltage = 12.0;
        public static final double driveStallTorqueNewtonMeters = 3.6 / driveGearReduction; // Motor's stall torque times gear ratio
        public static final double driveStallCurrentAmps = 211.0;
        public static final double driveFreeCurrentAmps = 3.6;

        public static final ModuleConfig moduleConfig = new ModuleConfig(
            wheelRadiusMeters, driveFreeSpeedMetersPerSec, wheelCoefficientOfFriction, 
            new DCMotor(driveNominalOperatingVoltage, driveStallTorqueNewtonMeters, driveStallCurrentAmps, driveFreeCurrentAmps, driveFreeSpeedRadPerSec, 1),
            driveCurrentLimitAmps, 4);
    }

	public static class SwerveDriveConstants {
        public static final Rotation2d flModuleOffset = Rotation2d.fromDegrees(32.08);
        public static final Rotation2d frModuleOffset = Rotation2d.fromDegrees(164.17);
        public static final Rotation2d blModuleOffset = Rotation2d.fromDegrees(71.68);
        public static final Rotation2d brModuleOffset = Rotation2d.fromDegrees(116.9);

		// TODO Set these dimensions for the distance between the center of each wheel.
        // Note that these values are different from the robot's overall dimenstions.
		public static final double chassisLengthMeters = Units.inchesToMeters(21.75);
        public static final double chassisWidthMeters = Units.inchesToMeters(21.75);

        public static final double chassisRadiusMeters = Math.hypot(chassisLengthMeters, chassisWidthMeters);

        public static final SwerveDriveKinematics kinematics = new SwerveDriveKinematics(
            new Translation2d(chassisWidthMeters / 2.0, chassisLengthMeters / 2.0),  // front left
            new Translation2d(chassisWidthMeters / 2.0, -chassisLengthMeters / 2.0), // front right
            new Translation2d(-chassisWidthMeters / 2.0, chassisLengthMeters / 2.0), // back left
            new Translation2d(-chassisWidthMeters / 2.0, -chassisLengthMeters / 2.0) // back right
        );

        public static final double maxAttainableSpeedMetersPerSec = Units.feetToMeters(20.1);
        public static final double maxAttainableRotationRadPerSec = 13.4;

        public static final double skewCompensationRatioOmegaPerTheta = 0.1;

        // TODO Tune the below PID values using the SysID routines.
        public static final double autoTranslationKp = 5.0;
        public static final double autoTranslationKd = 0.0;

        public static final double autoRotationKp = 5.0;
        public static final double autoRotationKd = 0.0;
    }

    public static class ElevatorConstants {
        public static final int maxElevatorCurrentAmps = 50;

        public static final double gearRatio = 5.0;
        
        public static final double kP = 0.03; // 0.035
        public static final double kD = 0.0; // 0.00037

        public static final double sprocketDiameter = 0.9;

        public static final double inchesPerEncRev = ((2 * sprocketDiameter) / gearRatio);
        public static final double inchesPerSecPerRPM = ((2 * sprocketDiameter) / gearRatio * 60.0);

        public static final double freeSpeedInchesPerSec = 6784.0 / gearRatio * sprocketDiameter;

        public static final double maxVelInchesPerSec = 50.0; // 400.0

        public static final double maxAccelInchesPerSecSq = 250.0; // 575.0

        public static final double maxManualInchesPerSec = 100.0;

        public static final double  maxManualInchesPerSecSq = 100.0;

        public static final double coralOnePresetInches = -10.0;
 
        public static final double coralTwoPresetInches = -20.0;

        public static final double coralThreePresetInches = -30.0;

        public static final double homePresetInches = 0.0;

        public static final double lowerLimitInches = -30f;

        public static final double upperLimitInches = 0f; // 47.47

        public static final double toleranceInches = 0.1;

     }
}
