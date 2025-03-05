package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Constants;
import frc.robot.subsystems.*;

public class StationIntake extends SequentialCommandGroup {

  public StationIntake(Intake intake, Arm arm, Elevator elevator) {

    addRequirements(intake);
    addCommands(

        Commands.either(Commands.none(),
          new SequentialCommandGroup(
            Commands.runOnce(() -> arm.goToLocation(Constants.Arm.SafeCarryAngle), arm),
            new WaitUntilCommand(() -> arm.isAtLocation())),
        () -> elevator.getLocation() < Constants.Elevator.HeightL3),
        Commands.runOnce(() -> elevator.goToLocation(Constants.Elevator.IntakeHeight), elevator),
        Commands.waitUntil(() -> elevator.getLocation() < Constants.Elevator.HeightL3),

        Commands.runOnce(() -> arm.goToLocation(Constants.Arm.intakeAngle), arm),
        Commands.runOnce(() -> intake.DoIntake(), intake),
        Commands.waitUntil(() -> intake.SeesCoral()),
        Commands.waitSeconds(.8),
        Commands.runOnce(() -> intake.stop(), intake),
        Commands.runOnce(() -> arm.goToLocation(Constants.Arm.CarryAngle), arm)
      
      );
  }
}
