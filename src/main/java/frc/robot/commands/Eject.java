package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.*;

public class Eject extends SequentialCommandGroup {

  public Eject(Intake intake) {

    addRequirements(intake);
    addCommands(
      Commands.runOnce(() -> intake.ShootAlgae(), intake),
      Commands.waitSeconds(.5),
      Commands.runOnce(() -> intake.stop(), intake)

    );

  }
}
