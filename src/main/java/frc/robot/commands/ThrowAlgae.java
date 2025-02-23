package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.subsystems.*;

public class ThrowAlgae extends SequentialCommandGroup {

  public ThrowAlgae(Arm arm, Intake intake, Elevator elevator) {

    addRequirements(intake, elevator);
    addCommands(
        Commands.runOnce(() -> elevator.goToLocation(Constants.Elevator.HeightNET), elevator),
        Commands.runOnce(() -> arm.goToLocation(Constants.Arm.ThrowingAngle), arm),
        Commands.waitUntil(() -> arm.armDegrees() >= Constants.Arm.ReleasingAngle),
        Commands.runOnce(() -> intake.ShootAlgae(), intake));
  }
}
