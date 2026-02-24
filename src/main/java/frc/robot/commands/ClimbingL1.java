package frc.robot.commands;
import frc.robot.Constants;
import frc.robot.subsystems.ClimbL1;
import edu.wpi.first.wpilibj2.command.Command;

public class ClimbingL1 {
    private ClimbL1 climbL1;

    public ClimbingL1(ClimbL1 climbL1) {
        this.climbL1 = climbL1;
    }

    public Command levelOneExtend() {
        return climbL1.setLevelCmd(Constants.ClimbL1Constants.SetpointConstants.LEVEL_ONE);
    }

    public Command levelOneRetract() {
        return climbL1.retractLevelCmd(Constants.ClimbL1Constants.SetpointConstants.MINIMUM);
    }

    public Command climbOneCmd() {
        return levelOneExtend()
            .andThen(levelOneRetract());
    }

}