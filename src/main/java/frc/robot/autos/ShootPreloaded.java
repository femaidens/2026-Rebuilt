package frc.robot.autos;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.Shooting;
import frc.robot.subsystems.AutoShooter;
import frc.robot.subsystems.Hopper;

public class ShootPreloaded extends SequentialCommandGroup {
    public ShootPreloaded(Shooting shooting, AutoShooter autoshooter, Hopper hopper){
        addCommands(
            autoshooter.runShooterMotorCmd().withTimeout(3),
            
            Commands.parallel(
                autoshooter.runShooterMotorCmd(),
                shooting.prepareShotAuto(hopper, autoshooter)
            ).withTimeout(8),

            autoshooter.stopShooterMotorCmd(),
            hopper.stopSpindexer(),
            autoshooter.stopIndexerMotorCmd()
        );
    }
}
