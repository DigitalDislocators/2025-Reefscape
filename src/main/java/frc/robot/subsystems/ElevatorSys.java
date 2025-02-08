 package frc.robot.subsystems;

 import com.revrobotics.spark.SparkFlex;
 import com.revrobotics.RelativeEncoder;
 import com.revrobotics.spark.SparkLowLevel.MotorType;
 import com.revrobotics.spark.config.SparkFlexConfig;

 import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.controller.ProfiledPIDController;
 import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
 import edu.wpi.first.wpilibj.DriverStation;
 import edu.wpi.first.wpilibj2.command.SubsystemBase;
 import frc.robot.Constants.CANDevices;
 import frc.robot.Constants.ElevatorConstants;

 public  class ElevatorSys extends SubsystemBase{
    private final SparkFlex leftElevatorMtr;
    private final SparkFlex rightElevatorMtr;

    private final RelativeEncoder leftElevatorEnc;
    private final RelativeEncoder righElevatorEnc;

    private final ProfiledPIDController elevatorController;

    private double targetInches = 0.0;
    private double manualPower = 0.0;

public ElevatorSys() {
leftElevatorMtr = new SparkFlex(CANDevices.leftElevatorMtrID, MotorType.kBrushless);
SparkFlexConfig leftElevatorMtSparkFlexConfig = new SparkFlexConfig();

rightElevatorMtr = new SparkFlex(CANDevices.rightElevatorMtrID, MotorType.kBrushless);
SparkFlexConfig rightElevatorMtSparkFlexConfig = new SparkFlexConfig();

leftElevatorMtSparkFlexConfig.inverted(false);
leftElevatorMtSparkFlexConfig.idleMode(com.revrobotics.spark.config.SparkBaseConfig.IdleMode.kBrake);
leftElevatorMtSparkFlexConfig.encoder.positionConversionFactor(-1.0 * ElevatorConstants.inchesPerEncRev);
leftElevatorMtSparkFlexConfig.encoder.velocityConversionFactor(-1.0 * ElevatorConstants.inchesPerSecPerRPM);

rightElevatorMtSparkFlexConfig.inverted(true);
rightElevatorMtSparkFlexConfig.idleMode(com.revrobotics.spark.config.SparkBaseConfig.IdleMode.kBrake);
rightElevatorMtSparkFlexConfig.encoder.positionConversionFactor(-1.0 * ElevatorConstants.inchesPerEncRev);
rightElevatorMtSparkFlexConfig.encoder.velocityConversionFactor(-1.0 * ElevatorConstants.inchesPerSecPerRPM);

leftElevatorMtSparkFlexConfig.voltageCompensation(10); 
rightElevatorMtSparkFlexConfig.voltageCompensation(10);

leftElevatorMtSparkFlexConfig.smartCurrentLimit(ElevatorConstants.maxElevatorCurrentAmps);
rightElevatorMtSparkFlexConfig.smartCurrentLimit(ElevatorConstants.maxElevatorCurrentAmps);


leftElevatorMtSparkFlexConfig.softLimit.forwardSoftLimitEnabled(true);
rightElevatorMtSparkFlexConfig.softLimit.reverseSoftLimitEnabled(true);
// leftElevatorMtConfig.softLimit.forwardSoftLimitEnable(true);
// rightElevatorMtConfig.softLimit.reverseSoftLimitEnable(true);

leftElevatorMtSparkFlexConfig.softLimit.forwardSoftLimit(ElevatorConstants.upperLimitInches);
rightElevatorMtSparkFlexConfig.softLimit.forwardSoftLimit(ElevatorConstants.lowerLimitInches);
// leftElevatorMtConfig.softLimit.forwardSoftLimit(ElevatorConstants.upperLimitInches);
// rightElevatorMtConfig.softLimit.forwardSoftLimit(ElevatorConstants.lowerLimitInches);

leftElevatorEnc = leftElevatorMtr.getEncoder();
righElevatorEnc = rightElevatorMtr.getEncoder();

leftElevatorEnc.setPosition(ElevatorConstants.homePresetInches);
righElevatorEnc.setPosition(ElevatorConstants.homePresetInches);

elevatorController = new ProfiledPIDController(
    ElevatorConstants.kP, 0.0, ElevatorConstants.kD, 
    new Constraints(ElevatorConstants.maxVelInchesPerSec, ElevatorConstants.maxAccelInchesPerSecSq));
}
@Override
public void periodic(){
    if(manualPower == 0.0){
        leftElevatorMtr.set(elevatorController.calculate(leftElevatorEnc.getPosition(),targetInches));
        rightElevatorMtr.set(elevatorController.calculate(leftElevatorEnc.getPosition(),targetInches));

    }
    else {
        leftElevatorMtr.set(manualPower);
        rightElevatorMtr.set(manualPower);
        targetInches = getCurrentPositionInches();
        elevatorController.reset(targetInches);
    }
    if(DriverStation.isDisabled()){
        targetInches = getCurrentPositionInches();
        elevatorController.reset(targetInches);
    }
    SmartDashboard.putNumber("elevator error inches", getCurrentPositionInches());
}
public double getCurrentPositionInches() {
    return ((leftElevatorEnc.getPosition() + righElevatorEnc.getPosition()) / 2);
}
 public void setTargetInches(double inches){
      targetInches = inches;
 }
 public void setManualSpeedInchesPerSec (double inchesPersec){
    double manualPower = inchesPersec / ElevatorConstants.freeSpeedInchesPerSec;
    this.manualPower = manualPower;
 }
 public boolean isAtTarget(){
    return Math.abs(getCurrentPositionInches() - targetInches) < ElevatorConstants.toleranceInches;
 }}