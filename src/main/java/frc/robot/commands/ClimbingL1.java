package frc.robot.commands;
import frc.robot.Constants;
import frc.robot.subsystems.Climb;
import edu.wpi.first.wpilibj2.command.Command;

public class ClimbingL1 {
    private Climb climb;

    public Climbing(Climb climb) {
        this.climb = climb;
    }

    public Command levelOneExtend() {
        return climb.setLevelCmd(Constants.ClimbConstants.SetpointConstants.LEVEL_ONE, true);
    }

    public Command levelOneRetract() {
        return climb.setLevelCmd(Constants.ClimbConstants.SetpointConstants.MINIMUM, false);
    }

    public Command climbOneCmd() {
        return levelOneExtend()
            .andThen(levelOneRetract());
    }

}