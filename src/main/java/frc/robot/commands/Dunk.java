package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.subsystems.*;

public class Dunk extends SequentialCommandGroup {

  public Dunk(Intake intake, Elevator elevator, Arm arm) {

    addRequirements(intake, elevator);
    addCommands(
        Commands.runOnce(() -> arm.dunk(), arm),
        Commands.waitSeconds(.2),
        Commands.runOnce(() -> elevator.DunkAuto(), elevator),
        Commands.runOnce(() -> intake.ShootCoralAuto(), intake),
        
        Commands.waitSeconds(.4),
        Commands.runOnce(() -> intake.stop(), intake),
        Commands.runOnce(() -> arm.goToLocation(Constants.Arm.SafeCarryAngle), arm)
        );
  }
}
