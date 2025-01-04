package frc.robot.commands;

import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Config;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Mechanism;
import frc.robot.subsystems.drive.SwerveDrive;

public class SysIDRoutines {
    
    public static Command quasistaticForward(SwerveDrive swerveDrive) {
        return new SysIdRoutine(
            new Config(),
            new Mechanism(
                (volts) -> swerveDrive.applyCharacterizationVoltage(volts.in(Units.Volts)), null, swerveDrive)).quasistatic(Direction.kForward);
    }

    public static Command quasistaticReverse(SwerveDrive swerveDrive) {
        return new SysIdRoutine(
            new Config(),
            new Mechanism(
                (volts) -> swerveDrive.applyCharacterizationVoltage(volts.in(Units.Volts)), null, swerveDrive)).quasistatic(Direction.kReverse);
    }

    public static Command dynamicForward(SwerveDrive swerveDrive) {
        return new SysIdRoutine(
            new Config(),
            new Mechanism(
                (volts) -> swerveDrive.applyCharacterizationVoltage(volts.in(Units.Volts)), null, swerveDrive)).dynamic(Direction.kForward);
    }

    public static Command dynamicReverse(SwerveDrive swerveDrive) {
        return new SysIdRoutine(
            new Config(),
            new Mechanism(
                (volts) -> swerveDrive.applyCharacterizationVoltage(volts.in(Units.Volts)), null, swerveDrive)).dynamic(Direction.kReverse);
    }
}
