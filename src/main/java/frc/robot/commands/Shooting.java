package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.AutoShooter;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.Hopper;

public class Shooting {
    //testing without auto shooter, so just button to run the spindezer and indexer after ramping up shooter
    public Command prepareShot(Hopper hopper, AutoShooter shooter){
        return 
            Commands.parallel(shooter.runIndexerMotorCmd(),
                hopper.runSpindexer())
            .finallyDo((interrupted) -> {
                shooter.stopIndexerMotorCmd().initialize();
                hopper.stopSpindexer().initialize();
             }
        );
    }

    public Command prepareShotAuto(Hopper hopper, AutoShooter shooter){
        return 
            Commands.parallel(shooter.runIndexerMotorCmd(),
                hopper.runSpindexer());
    }



    public Command reversePrepareShot(Hopper hopper, AutoShooter shooter) {
        return 
            Commands.parallel(shooter.reverseIndexerMotorCmd(),
                hopper.reverseSpindexer())
            .finallyDo((interrupted) -> {
                shooter.stopIndexerMotorCmd().initialize();
                hopper.stopSpindexer().initialize();
             }
        );
    }
    // public Command shootSequence(AutoShooter shooter, Hopper hopper, Drive drive) {
    //     return Commands.deadline(
    //         Commands.sequence(
    //             // wait for spin-up
    //             Commands.waitUntil(() -> {
    //                 double distance = drive.distanceFromTarget();
    //                 return shooter.isReadyToShoot(shooter.getTargetRPS(distance));
    //             }),
    //             // feed the note 
    //             Commands.parallel(
    //                 shooter.runIndexerMotorCmd(),
    //                 hopper.runSpindexer()
    //             ).withTimeout(3.0) 
    //         ),
    //         // this runs the whole time the sequence above is running
    //         shooter.run(() -> {
    //             double distance = drive.distanceFromTarget();
    //             shooter.autoShoot(shooter.getTargetRPS(distance), shooter.getTargetAngle(distance));
    //         })
    //     ).finallyDo((interrupted) -> {
    //         shooter.stopShooterMotorCmd().initialize();
    //         shooter.stopIndexerMotorCmd().initialize();
    //         hopper.stopSpindexer().initialize();
    //     });
    // }
}
