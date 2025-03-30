package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Constants;
import frc.robot.subsystems.*;

public class GoToL4Auto extends SequentialCommandGroup {

  public GoToL4Auto(Arm arm, Elevator elevator) {

    addCommands(

      Commands.runOnce(() -> arm.goToLocation(Constants.Arm.SafeCarryAngle), arm),
      new WaitUntilCommand(() -> arm.isAtLocation()),
      Commands.runOnce(() -> elevator.goToLocation(Constants.Elevator.HeightL4), elevator)
    );
  }
}
