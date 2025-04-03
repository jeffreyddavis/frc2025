package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.subsystems.*;

public class GetFloorCoral extends SequentialCommandGroup {

  public GetFloorCoral(Arm arm, Intake intake, Elevator elevator) {

    addCommands(
        Commands.runOnce(() -> elevator.goToLocation(Constants.Elevator.SeaFloorHeight), elevator),
        Commands.runOnce(() -> arm.goToLocation(Constants.Arm.StraightOut), arm),
        Commands.runOnce(() -> intake.DoIntake(), intake),
        Commands.waitUntil(() -> elevator.isNearLocation()),
        Commands.waitUntil(() -> arm.isNearLocation())
        //Commands.waitUntil(() -> intake.SeesCoral()),
        //Commands.waitSeconds(.4),
        //Commands.runOnce(() -> intake.holdAlgae(), intake),
        //Commands.runOnce(() -> arm.goToLocation(Constants.Arm.CarryAngle))
    );

  }
}
