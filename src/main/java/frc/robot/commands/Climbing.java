package frc.robot.commands;
import frc.robot.Constants;
import frc.robot.subsystems.Climb;
import edu.wpi.first.wpilibj2.command.Command;

public class Climbing {
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

    public Command levelTwoThreeExtend() {
        return climb.setLevelCmd(Constants.ClimbConstants.SetpointConstants.LEVEL_TWO_THREE, true);
    }

    public Command levelTwoThreeRetract() {
        return climb.setLevelCmd(Constants.ClimbConstants.SetpointConstants.MINIMUM, false);
    }


    //Sequential Commands
    public Command climbOneCmd() {
        return levelOneExtend()
            .andThen(levelOneRetract());
    }

    public Command climbTwoCmd() {
        return levelOneExtend()
            .andThen(levelOneRetract())
            .andThen(levelTwoThreeExtend())
            .andThen(levelTwoThreeRetract());
    }

    public Command climbThreeCmd() {
        return levelOneExtend()
            .andThen(levelOneRetract())
            .andThen(levelTwoThreeExtend())
            .andThen(levelTwoThreeRetract())
            .andThen(levelTwoThreeExtend())
            .andThen(levelTwoThreeRetract());
    }

}