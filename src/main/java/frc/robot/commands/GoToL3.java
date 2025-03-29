package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Constants;
import frc.robot.subsystems.*;

public class GoToL3 extends SequentialCommandGroup {

  public GoToL3(Arm arm, Elevator elevator) {

    addCommands(
      Commands.runOnce(() -> arm.goToLocation(Constants.Arm.SafeCarryAngle), arm),
      new WaitUntilCommand(() -> arm.isAtLocation()),

      
      Commands.runOnce(() -> elevator.goToLocation(Constants.Elevator.HeightL3), elevator),
      Commands.waitUntil(() -> elevator.isAtLocation()),
      Commands.runOnce(() -> arm.goToLocation(Constants.Arm.ScoringAngle), arm)  
    );
  }
}
