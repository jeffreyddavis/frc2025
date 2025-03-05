package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.subsystems.*;
import frc.robot.subsystems.Elevator.Targets;

public class GoToScore extends SequentialCommandGroup {

  public GoToScore(Arm arm, Elevator elevator, Elevator.Targets target) {

    addRequirements(arm, elevator);

    if (target != Targets.Floor && target != Targets.Processor) {
      addCommands(
          new ParallelCommandGroup(
              Commands.runOnce(() -> elevator.GoToTarget(target), elevator),
              Commands.runOnce(() -> arm.goToLocation(Constants.Arm.ScoringAngle), arm)));

    } else {

      if (!elevator.armClear()) {

        addCommands(
            Commands.runOnce(
                () -> elevator.goToLocation(Constants.Elevator.ArmClearHeight), elevator),
            Commands.waitUntil(() -> elevator.isAtLocation()));
      }
      addCommands(
          Commands.runOnce(() -> arm.goToLocation(Constants.Arm.CarryAngle), arm),
          Commands.waitUntil(() -> arm.isAtLocation()),
          Commands.runOnce(() -> elevator.GoToTarget(target), elevator));
    }
  }
}
