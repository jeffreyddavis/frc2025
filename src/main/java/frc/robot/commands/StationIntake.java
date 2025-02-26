package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.*;

public class StationIntake extends SequentialCommandGroup {

  public StationIntake(Intake intake) {

    addRequirements(intake);
    addCommands(
        Commands.runOnce(() -> intake.Intake(), intake),
        Commands.waitUntil(() -> intake.SeesCoral()),
        Commands.waitSeconds(.5),
        Commands.runOnce(() -> intake.stop(), intake)
      );
  }
}
