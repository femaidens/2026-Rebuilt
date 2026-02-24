package frc.robot.commands;
import frc.robot.Constants;
import frc.robot.subsystems.Climb;
import edu.wpi.first.wpilibj2.command.Command;

public class ClimbingL1 {
    private ClimbL1 climbL1;

    public Climbing(ClimbL1 climbL1) {
        this.climbL1 = climbL1;
    }

    public Command levelOneExtend() {
        return climbL1.setLevelCmd(Constants.ClimbL1Constants.SetpointConstants.LEVEL_ONE, true);
    }

    public Command levelOneRetract() {
        return climbL1.setLevelCmd(Constants.ClimbL1Constants.SetpointConstants.MINIMUM, false);
    }

    public Command climbOneCmd() {
        return levelOneExtend()
            .andThen(levelOneRetract());
    }

}