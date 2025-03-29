package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Constants;
import frc.robot.subsystems.*;

public class Stow extends SequentialCommandGroup {

  public Stow(Arm arm, Elevator elevator, Intake intake) {

    addCommands(
      Commands.runOnce(() -> intake.stop(), intake),
      Commands.runOnce(() -> arm.goToLocation(Constants.Arm.SafeCarryAngle), arm),
      new WaitUntilCommand(() -> arm.isAtLocation()),  
      Commands.runOnce(() -> elevator.goToLocation(Constants.Elevator.StowHeight), elevator),
      Commands.waitUntil(() -> elevator.isAtLocation()),
      Commands.runOnce(() -> arm.goToLocation(Constants.Arm.Max), arm)   
    );
  }
}
