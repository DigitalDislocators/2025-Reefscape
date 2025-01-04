// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.Constants.RobotConstants;
import frc.robot.Constants.SwerveDriveConstants;
import frc.robot.Constants.SwerveModuleConstants;
import frc.robot.commands.ArcadeDriveCmd;
import frc.robot.commands.ExampleCommand;
import frc.robot.commands.SysIDRoutines;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.PoseEstimator;
import frc.robot.subsystems.drive.SwerveDrive;

import java.util.function.BooleanSupplier;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
	// The robot's subsystems and commands are defined here...
	private final ExampleSubsystem exampleSubsystem = new ExampleSubsystem();

	private final SwerveDrive swerveDrive = new SwerveDrive();

	private final PoseEstimator poseEstimator = new PoseEstimator(
		SwerveDriveConstants.kinematics,
		() -> swerveDrive.getHeading(),
		() -> swerveDrive.getModulePositions());

	private final CommandXboxController driverController = new CommandXboxController(OperatorConstants.kDriverControllerPort);

	// Initializes and populates the auto chooser with all the PathPlanner autos in the project.
	// The deafulat auto is "Do Nothing" and runs Commands.none(), which does nothing.
	private final SendableChooser<Command> autoChooser;

	/** The container for the robot. Contains subsystems, OI devices, and commands. */
	public RobotContainer() {
		// Configure the trigger bindings
		configureBindings();

		AutoBuilder.configure(
			poseEstimator::get,
			poseEstimator::updatePose,
			swerveDrive::getRobotRelativeSpeeds, 
			(chassisSpeeds, feedforward) -> swerveDrive.driveRobotRelative(chassisSpeeds),
			new PPHolonomicDriveController(
				new PIDConstants(SwerveDriveConstants.autoTranslationKp, SwerveDriveConstants.autoTranslationKd),
				new PIDConstants(SwerveDriveConstants.autoRotationKp, SwerveDriveConstants.autoRotationKd)),
			new RobotConfig(RobotConstants.massKg, RobotConstants.momentOfInertiaKgMetersSq, 
				SwerveModuleConstants.moduleConfig, SwerveDriveConstants.kinematics.getModules()),
			// FIXME see if field is mirrored
			(BooleanSupplier) () -> false,
			swerveDrive);

		autoChooser = AutoBuilder.buildAutoChooser("Do Nothing");

		autoChooser.addOption("SysID Quasistatic Forward", SysIDRoutines.quasistaticForward(swerveDrive));
		autoChooser.addOption("SysID Quasistatic Reverse", SysIDRoutines.quasistaticReverse(swerveDrive));
		autoChooser.addOption("SysID Dynamic Forward", SysIDRoutines.dynamicForward(swerveDrive));
		autoChooser.addOption("SysID Dynamic Reverse", SysIDRoutines.dynamicReverse(swerveDrive));

		SmartDashboard.putData(autoChooser);

		// TODO Register the commands used in the PathPlanner auto builder like the below example.
		NamedCommands.registerCommand("exampleCommand", new ExampleCommand(exampleSubsystem));
	}

	/**
	 * Use this method to define your trigger->command mappings. Triggers can be created via the
	 * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
	 * predicate, or via the named factories in {@link
	 * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
	 * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
	 * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
	 * joysticks}.
	 */
	private void configureBindings() {
		swerveDrive.setDefaultCommand(
			new ArcadeDriveCmd(
					() -> MathUtil.applyDeadband(driverController.getLeftY(), OperatorConstants.joystickDeadband),
					() -> MathUtil.applyDeadband(driverController.getLeftX(), OperatorConstants.joystickDeadband),
					() -> MathUtil.applyDeadband(driverController.getRightX(), OperatorConstants.joystickDeadband),
					true,
					swerveDrive,
					poseEstimator));
	}

	/**
	 * Use this to pass the autonomous command to the main {@link Robot} class.
	 *
	 * @return the command to run in autonomous
	 */
	public Command getAutonomousCommand() {
		// An example command will be run in autonomous
		return autoChooser.getSelected();
	}

	public void updateDashboard() {
		SmartDashboard.putNumber("FL CANcoder", swerveDrive.getCanCoderAngles()[0].getDegrees());
		SmartDashboard.putNumber("FR CANcoder", swerveDrive.getCanCoderAngles()[1].getDegrees());
		SmartDashboard.putNumber("BL CANcoder", swerveDrive.getCanCoderAngles()[2].getDegrees());
		SmartDashboard.putNumber("BR CANcoder", swerveDrive.getCanCoderAngles()[3].getDegrees());
	}
}
