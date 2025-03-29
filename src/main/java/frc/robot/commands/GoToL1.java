package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Constants;
import frc.robot.subsystems.*;

public class GoToL1 extends SequentialCommandGroup {

  public GoToL1(Arm arm, Elevator elevator) {

    addCommands(
      Commands.either(Commands.runOnce(() -> arm.goToLocation(Constants.Arm.CarryAngle), arm),
        new SequentialCommandGroup(
          Commands.runOnce(() -> arm.goToLocation(Constants.Arm.SafeCarryAngle), arm),
          new WaitUntilCommand(() -> arm.isAtLocation())),
      () -> arm.armDegrees() < Constants.Arm.SafeCarryAngle  || elevator.getLocation() < Constants.Elevator.MaxSafeHeight),
  
      
      Commands.runOnce(() -> elevator.goToLocation(Constants.Elevator.HeightL1), elevator),
      Commands.waitUntil(() -> elevator.isAtLocation()),
      Commands.runOnce(() -> arm.goToLocation(Constants.Arm.CarryAngle), arm)   
    );
  }
}
