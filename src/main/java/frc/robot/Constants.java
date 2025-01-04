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
        public static final double robotWidthMeters = Units.inchesToMeters(26.0);
        public static final double robotLengthMEters = Units.inchesToMeters(26.0);

        // Moment of inertia of a uniform-mass slab with the axis of rotation centered and perpendicular to the slab
        // This should be a reasonable approximation of the robot's MOI
        public static final double momentOfInertiaKgMetersSq = massKg * (Math.pow(robotWidthMeters, 2) + Math.pow(robotLengthMEters, 2)) / 12;
    }


	public static class OperatorConstants {
		public static final int kDriverControllerPort = 0;

		public static final double joystickDeadband = 0.1;
	}

	public static class CANDevices {
		public static final int pigeonID = 14;

		public static final int flModuleCANCoderID = 10;
		public static final int flModuleDriveMtrID = 9;
		public static final int flModuleSteerMtrID = 8;

		public static final int frModuleCANCoderID = 11;
		public static final int frModuleDriveMtrID = 3;
		public static final int frModuleSteerMtrID = 2;

		public static final int blModuleCANCoderID = 12;
		public static final int blModuleDriveMtrID = 7;
		public static final int blModuleSteerMtrID = 6;

		public static final int brModuleCANCoderID = 13;
		public static final int brModuleDriveMtrID = 5;
		public static final int brModuleSteerMtrID = 4;
	}

	public static class SwerveModuleConstants {
        // TODO Tune the below PID and FF values using the SysID routines.
        public static final double driveKp = 0.1;
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

        // TODO Select the corresponding gear reduction for the configuration of your modules. This value is for MK4i L3 modules.
        // This value can be found on the SDS website.
        public static final double driveGearReduction = (14.0 / 50.0) * (28.0 / 16.0) * (15.0 / 45.0);

        public static final double driveMetersPerEncRev = driveGearReduction * 2.0 * wheelRadiusMeters * Math.PI;

        public static final double driveMetersPerSecPerEncRPM = driveMetersPerEncRev / 60.0;

        // TODO Select the correstponding gear reduction for the configuration of your modules. This value is for MK4i modules.
        // This value can be found on the SDS website.
        public static final double steerGearReduction = 7.0 / 150.0;

        public static final double steerRadiansPerEncRev = steerGearReduction * 2.0 * Math.PI;

        public static final double steerRadiansPerSecPerEncRPM = steerRadiansPerEncRev / 60.0;
        
        // TODO Choose the free speed for the configuration of your modules. This value is for MK4i L3 modules with NEO Vortexes.
        // This value can be found on the SDS website.
        public static final double driveFreeSpeedMetersPerSec = Units.feetToMeters(19.3);

        public static final double driveFreeSpeedRadPerSec = driveFreeSpeedMetersPerSec / wheelRadiusMeters;

        // TODO Set these values for the specifications of your drive motors. These are the specs of a NEO Vortex.
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
        public static final Rotation2d flModuleOffset = Rotation2d.fromDegrees(123.13);
        public static final Rotation2d frModuleOffset = Rotation2d.fromDegrees(184.13);
        public static final Rotation2d blModuleOffset = Rotation2d.fromDegrees(66.18);
        public static final Rotation2d brModuleOffset = Rotation2d.fromDegrees(78.39);

		// TODO Set these dimensions for the distance between the center of each wheel.
        // Note that these values are different from the robot's overall dimenstions.
		public static final double chassisLengthMeters = Units.inchesToMeters(24.0);
        public static final double chassisWidthMeters = Units.inchesToMeters(24.0);

        public static final double chassisRadiusMeters = Math.hypot(chassisLengthMeters, chassisWidthMeters);

        public static final SwerveDriveKinematics kinematics = new SwerveDriveKinematics(
            new Translation2d(chassisWidthMeters / 2.0, chassisLengthMeters / 2.0),  // front left
            new Translation2d(chassisWidthMeters / 2.0, -chassisLengthMeters / 2.0), // front right
            new Translation2d(-chassisWidthMeters / 2.0, chassisLengthMeters / 2.0), // back left
            new Translation2d(-chassisWidthMeters / 2.0, -chassisLengthMeters / 2.0) // back right
        );

        public static final double maxAttainableSpeedMetersPerSec = 5.8;
        public static final double maxAttainableRotationRadPerSec = 13.4;

        public static final double skewCompensationRatioOmegaPerTheta = 0.1;

        // TODO Tune the below PID values using the SysID routines.
        public static final double autoTranslationKp = 5.0;
        public static final double autoTranslationKd = 0.0;

        public static final double autoRotationKp = 5.0;
        public static final double autoRotationKd = 0.0;
    }
}
