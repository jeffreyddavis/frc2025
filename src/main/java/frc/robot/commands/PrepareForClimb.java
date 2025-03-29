package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.subsystems.*;

public class PrepareForClimb extends SequentialCommandGroup {
    public PrepareForClimb(Climber thClimber) {

        addRequirements(thClimber);
        addCommands(
            Commands.runOnce(() -> thClimber.releaseCage()),
            Commands.waitSeconds(.5),
            Commands.runOnce(() -> thClimber.goToHeight(Constants.Climber.TopHeight)),
            Commands.waitUntil(() -> thClimber.isAtHeight()),
            Commands.runOnce(() -> thClimber.stop()),
            Commands.runOnce(() -> thClimber.enableClimbing())
        );
    }
}
