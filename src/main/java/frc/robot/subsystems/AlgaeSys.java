// package frc.robot.subsystems;


// import com.revrobotics.SparkPIDController;
// import com.revrobotics.CANSparkBase.ControlType;
// import com.revrobotics.CANSparkBase.IdleMode;
// import edu.wpi.first.wpilibj.I2C.Port;
// import edu.wpi.first.wpilibj2.command.SubsystemBase;
// import frc.robot.Constants.CANDevices;
// import frc.robot.Constants.RollerConstants;
// import com.revrobotics.spark.SparkFlex;
// import com.revrobotics.spark.SparkLowLevel.MotorType;
// import com.revrobotics.spark.SparkClosedLoopController;

// public class RollersSys extends SubsystemBase {

//     // Declare actuators, sensors, and other variables here
//     //private final CANSparkFlex topRollerMtr;
//    // SparkFlex flex = new SparkFlex(2, MotorType.kBrushless);
//        // TODO place motor name
//     public SparkFlex alageRollers = new SparkFlex(motor, MotorType.kBrushless);
    
//     private final SparkPIDController topRollerController;

//     public RollersSys() {
//         topRollerMtr = new CANSparkFlex(CANDevices.leaderRollerMtrId, MotorType.kBrushless);
       
//         topRollerMtr.enableVoltageCompensation(10);
       

//         topRollerMtr.setSmartCurrentLimit(RollerConstants.maxRollerCurrentAmps);
        
//         topRollerMtr.setIdleMode(IdleMode.kBrake);

//         topRollerMtr.setInverted(true);
        
//         topRollerController = topRollerMtr.getPIDController();

//         topRollerController.setFF(RollerConstants.feedForward);
//         topRollerController.setP(RollerConstants.kP);
//         topRollerController.setD(RollerConstants.kD);
//     }

//     @Override
//     public void periodic() {
//     }

//     public void setRPM(double rpm) {
//         // leaderRollerMtr.set(rpm / RollerConstants.maxRPM);
//         // Carl was kind of here;
//         topRollerController.setReference(rpm, ControlType.kVelocity);
        

//     public void setPower(double power) {
//         topRollerMtr.set(power);  
//     }
//     public double getRPM() {
//         return (topRollerMtr.getEncoder().getVelocity() ) / 2;
//     }
// }
// }

package frc.robot.subsystems; 
 
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.spark.SparkClosedLoopController;


public class AlgaeSys extends SubsystemBase {

//     SparkFlex flex = new SparkFlex(algaeRoller, MotorType.kBrushless);
//    SparkMaxConfig config = new SparkMaxConfig();


// config
//     .inverted(true)
//     .idleMode(IdleMode.kBrake);
// config.encoder
//     .positionConversionFactor(1000)
//     .velocityConversionFactor(1000);
// config.closedLoop
//     .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
//     .pid(1.0, 0.0, 0.0);

}