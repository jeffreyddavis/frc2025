package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Constants;
import frc.robot.subsystems.*;
import frc.robot.subsystems.Elevator.Targets;

public class GoToL2 extends SequentialCommandGroup {

  public GoToL2(Arm arm, Elevator elevator) {

    addCommands(
      Commands.either(Commands.runOnce(() -> arm.goToLocation(Constants.Arm.SafeCarryAngle), arm),
        new SequentialCommandGroup(
          Commands.runOnce(() -> arm.goToLocation(Constants.Arm.SafeCarryAngle), arm),
          new WaitUntilCommand(() -> arm.isAtLocation())),
      () -> arm.armDegrees() < Constants.Arm.SafeCarryAngle || elevator.getLocation() < Constants.Elevator.MaxSafeHeight),
  

      Commands.runOnce(() -> elevator.goToLocation(Constants.Elevator.HeightL2), elevator),
      Commands.runOnce(() -> arm.goToLocation(Constants.Arm.CarryAngle), arm),  
      
      Commands.waitUntil(() -> elevator.isAtLocation()),
      Commands.runOnce(() -> arm.goToLocation(Constants.Arm.ScoringAngle), arm)   
    );
  }
}
