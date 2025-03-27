package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Constants;
import frc.robot.subsystems.*;

public class Climb extends SequentialCommandGroup {
    public Climb(Climber thClimber) {

        addRequirements(thClimber);
        addCommands(
            Commands.runOnce(() -> thClimber.goToHeight(50)),
            Commands.waitUntil(() -> thClimber.isAtHeight()),
            Commands.runOnce(() -> thClimber.stop())
        );
    }
}
