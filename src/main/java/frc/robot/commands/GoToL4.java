package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Constants;
import frc.robot.subsystems.*;
import frc.robot.subsystems.Elevator.Targets;

public class GoToL4 extends SequentialCommandGroup {

  public GoToL4(Arm arm, Elevator elevator) {

    addCommands(

      Commands.runOnce(() -> arm.goToLocation(Constants.Arm.SafeCarryAngle), arm),
      new WaitUntilCommand(() -> arm.isAtLocation()),
      Commands.runOnce(() -> elevator.goToLocation(Constants.Elevator.HeightL4), elevator),
      Commands.waitUntil(() -> elevator.isAtLocation()),
      Commands.runOnce(() -> arm.goToLocation(Constants.Arm.HighScoringAngle), arm)   
    );
  }
}
