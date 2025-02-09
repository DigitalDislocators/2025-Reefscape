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


public class AlgaePivotSys extends SubsystemBase {

   // private final SparkFlex ;
    //private final SparkFlex ;

      //  private final RelativeEncoder ;

//  private final ProfiledPIDController ;

// TODO change all of the MotorType
public AlgaePivotSys() {

  // leftElevatorMtr = new SparkFlex(CANDevices.leftElevatorMtrID, MotorType.kBrushless);
// SparkFlexConfig leftElevatorMtSparkFlexConfig = new SparkFlexConfig();


// leftElevatorMtSparkFlexConfig.inverted(false);
// leftElevatorMtSparkFlexConfig.idleMode(com.revrobotics.spark.config.SparkBaseConfig.IdleMode.kBrake);
// leftElevatorMtSparkFlexConfig.encoder.positionConversionFactor(-1.0 * ElevatorConstants.inchesPerEncRev);
// leftElevatorMtSparkFlexConfig.encoder.velocityConversionFactor(-1.0 * ElevatorConstants.inchesPerSecPerRPM);

// leftElevatorMtSparkFlexConfig.voltageCompensation(10); 

// leftElevatorMtSparkFlexConfig.smartCurrentLimit(ElevatorConstants.maxElevatorCurrentAmps);

// leftElevatorMtSparkFlexConfig.softLimit.forwardSoftLimitEnabled(true);
 // leftElevatorMtConfig.softLimit.forwardSoftLimitEnable(true);

// leftElevatorMtSparkFlexConfig.softLimit.forwardSoftLimit(ElevatorConstants.upperLimitInches);
 // leftElevatorMtConfig.softLimit.forwardSoftLimit(ElevatorConstants.upperLimitInches);


// leftElevatorEnc = leftElevatorMtr.getEncoder();

// elevatorController = new ProfiledPIDController(
//     ElevatorConstants.kP, 0.0, ElevatorConstants.kD, 
//     new Constraints(ElevatorConstants.maxVelInchesPerSec, ElevatorConstants.maxAccelInchesPerSecSq));
 }

}