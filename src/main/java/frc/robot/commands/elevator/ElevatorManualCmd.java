package frc.robot.commands.elevator;

import frc.robot.Constants;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.subsystems.ElevatorSys;
import edu.wpi.first.wpilibj2.command.Command;
import java.util.function.DoubleSupplier;
import edu.wpi.first.math.filter.SlewRateLimiter;

public class ElevatorManualCmd extends Command {
    
    private final SlewRateLimiter slewRateLimiter;
    
    private final ElevatorSys elevator;
    
    private final DoubleSupplier input;

    public ElevatorManualCmd(DoubleSupplier input, ElevatorSys elevator){
        this.elevator = elevator;
        this.input = input;

        slewRateLimiter = new SlewRateLimiter(Constants.ElevatorConstants.maxManualInchesPerSecSq);
        addRequirements(elevator);
    }
@Override 
public void initialize(){}

@Override 
public void execute(){
    elevator.setManualSpeedInchesPerSec(slewRateLimiter.calculate(input.getAsDouble() * ElevatorConstants.maxManualInchesPerSec));
}
@Override
public void end(boolean interrupted){}
@Override
public boolean isFinished(){
    return false;
}
}