package frc.robot.subsystems; 
 
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkFlexConfig;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CANDevices;
import frc.robot.Constants.ElevatorConstants;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkClosedLoopController;


public class CoralSys extends SubsystemBase {

    //private final SparkFlex ;

      //  private final RelativeEncoder ;

   // private final ProfiledPIDController ;

   // TODO change all of the MotorType

   public CoralSys() {
    // eftElevatorMtr = new SparkFlex(CANDevices.leftElevatorMtrID, MotorType.kBrushless);
    // SparkFlexConfig leftElevatorMtSparkFlexConfig = new SparkFlexConfig();
    
    // rightElevatorMtr = new SparkFlex(CANDevices.rightElevatorMtrID, MotorType.kBrushless);
    // SparkFlexConfig rightElevatorMtSparkFlexConfig = new SparkFlexConfig();
    
    // leftElevatorMtSparkFlexConfig.inverted(false);
    // leftElevatorMtSparkFlexConfig.idleMode(com.revrobotics.spark.config.SparkBaseConfig.IdleMode.kBrake);
    // leftElevatorMtSparkFlexConfig.encoder.positionConversionFactor(-1.0 * ElevatorConstants.inchesPerEncRev);
    // leftElevatorMtSparkFlexConfig.encoder.velocityConversionFactor(-1.0 * ElevatorConstants.inchesPerSecPerRPM);
    
    // rightElevatorMtSparkFlexConfig.inverted(true);
    // rightElevatorMtSparkFlexConfig.idleMode(com.revrobotics.spark.config.SparkBaseConfig.IdleMode.kBrake);
    // rightElevatorMtSparkFlexConfig.encoder.positionConversionFactor(-1.0 * ElevatorConstants.inchesPerEncRev);
    // rightElevatorMtSparkFlexConfig.encoder.velocityConversionFactor(-1.0 * ElevatorConstants.inchesPerSecPerRPM);
    
    // leftElevatorMtSparkFlexConfig.voltageCompensation(10); 
    // rightElevatorMtSparkFlexConfig.voltageCompensation(10);
    
    // leftElevatorMtSparkFlexConfig.smartCurrentLimit(ElevatorConstants.maxElevatorCurrentAmps);
    // rightElevatorMtSparkFlexConfig.smartCurrentLimit(ElevatorConstants.maxElevatorCurrentAmps);
    
    
    // leftElevatorMtSparkFlexConfig.softLimit.forwardSoftLimitEnabled(true);
    // rightElevatorMtSparkFlexConfig.softLimit.reverseSoftLimitEnabled(true);
    // // leftElevatorMtConfig.softLimit.forwardSoftLimitEnable(true);
    // // rightElevatorMtConfig.softLimit.reverseSoftLimitEnable(true);
    
    // leftElevatorMtSparkFlexConfig.softLimit.forwardSoftLimit(ElevatorConstants.upperLimitInches);
    // rightElevatorMtSparkFlexConfig.softLimit.forwardSoftLimit(ElevatorConstants.lowerLimitInches);
    // // leftElevatorMtConfig.softLimit.forwardSoftLimit(ElevatorConstants.upperLimitInches);
    // // rightElevatorMtConfig.softLimit.forwardSoftLimit(ElevatorConstants.lowerLimitInches);
    
    // leftElevatorEnc = leftElevatorMtr.getEncoder();
    // righElevatorEnc = rightElevatorMtr.getEncoder();
    
    // leftElevatorEnc.setPosition(ElevatorConstants.homePresetInches);
    // righElevatorEnc.setPosition(ElevatorConstants.homePresetInches);
    
    // elevatorController = new ProfiledPIDController(
    //     ElevatorConstants.kP, 0.0, ElevatorConstants.kD, 
    //     new Constraints(ElevatorConstants.maxVelInchesPerSec, ElevatorConstants.maxAccelInchesPerSecSq));
    }
  }
