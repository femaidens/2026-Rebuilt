// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Hopper;
//adding
/** Add your docs here. */
public class FuelTransition {
    private Hopper hopper;
    public FuelTransition(Hopper hopper){
        this.hopper = hopper;
    }

    /** 
    public Command indexToTransition(){
        return hopper.runIndex()
        .until(hopper::isFuelStaged)
        .andThen(hopper.runTransition()).withTimeout(0.25)
        .andThen(hopper.stopIndex())
        .andThen(hopper.stopTransition());
    }
    */
    //No beambreak b/w anymore, runs while trigger pressed, stops when released
    public Command indexToTransition(){
        return hopper.runSpindexer()
        .alongWith(hopper.runIndexer());
    }

    public Command stopTransitioning(){
        return hopper.stopSpindexer()
        .alongWith(hopper.stopIndexer());
    }
   
}

