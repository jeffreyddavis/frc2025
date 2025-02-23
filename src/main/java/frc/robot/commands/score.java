package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.*;

public class score extends SequentialCommandGroup {

  public score(Intake intake) {

    addRequirements(intake);
    addCommands(
        Commands.runOnce(() -> intake.ShootCoral(), intake),
        Commands.waitSeconds(.5),
        Commands.runOnce(() -> intake.stop(), intake));
  }
}
