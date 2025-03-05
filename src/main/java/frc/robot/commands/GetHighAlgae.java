package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.subsystems.*;

public class GetHighAlgae extends SequentialCommandGroup {

  public GetHighAlgae(Arm arm, Intake intake, Elevator elevator) {

    addCommands(
      Commands.runOnce(() -> arm.goToLocation(Constants.Arm.SafeCarryAngle), arm),
      Commands.waitUntil(() -> arm.isAtLocation()),
      Commands.runOnce(() -> elevator.goToLocation(Constants.Elevator.HeightAlgaeHi), elevator),
      Commands.runOnce(() -> arm.goToLocation(Constants.Arm.GetAlgaeAngle), arm),
      Commands.runOnce(() -> intake.DoIntake(), intake),
      Commands.waitUntil(() -> intake.SeesCoral()),
      Commands.waitSeconds(.4),
      Commands.runOnce(() -> intake.holdAlgae(), intake),
      Commands.runOnce(() -> arm.goToLocation(Constants.Arm.CarryAngle)),
      Commands.runOnce(() -> elevator.goToLocation(Constants.Elevator.StowHeight), elevator)
    );

  }
}
