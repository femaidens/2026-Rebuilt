package frc.robot.autos;

import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.Shooting;
import frc.robot.subsystems.AutoShooter;
import frc.robot.subsystems.Hopper;

public class ShootTrench extends SequentialCommandGroup {
    public ShootTrench(Shooting shooting, AutoShooter autoshooter, Hopper hopper) {
        addCommands(
                Commands.sequence(
                        autoshooter.resetMotorPositionCmd(),
                        autoshooter.shootTrenchCmd().withTimeout(3),

                        Commands.parallel(
                                autoshooter.shootTrenchCmd(),
                                shooting.prepareShotAuto(hopper)).withTimeout(8),

                        autoshooter.stopShooterMotorCmd(),
                        hopper.stopIndexerMotorCmd(),
                        hopper.stopSpindexer()).handleInterrupt(() -> {
                            CommandScheduler.getInstance().schedule(
                                    Commands.sequence(
                                            autoshooter.stopShooterMotorCmd(),
                                            autoshooter.stopAngleMotorCmd(),
                                            hopper.stopIndexerMotorCmd(),
                                            hopper.stopSpindexer()));
                        }));
    }
}
