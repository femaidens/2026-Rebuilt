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
        return climb.setLevelCmd(Constants.ClimbConstants.SetpointConstants.MINIMUM, false)
            .andThen(climb.stopMotorCmd());
    }

    public Command levelTwoExtend() {
        return climb.setLevelCmd(Constants.ClimbConstants.SetpointConstants.LEVEL_TWO, true);
    }

    public Command levelTwoRetract() {
        return climb.setLevelCmd(Constants.ClimbConstants.SetpointConstants.MINIMUM, false);
    }

    public Command levelThreeExtend() {
        return climb.setLevelCmd(Constants.ClimbConstants.SetpointConstants.LEVEL_THREE, true);
    }

    public Command levelThreeRetract() {
        return climb.setLevelCmd(Constants.ClimbConstants.SetpointConstants.MINIMUM, false);
    }

    // public Command resetClimb() {
    //     return climb.setLevelCmd(Constants.ClimbConstants.SetpointConstants.MINIMUM, false)
    //         .andThen(climb.stopMotorCmd());
    // }

}