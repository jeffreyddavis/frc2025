package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Constants;
import frc.robot.subsystems.*;
import frc.robot.subsystems.Elevator.Targets;

public class Stow extends SequentialCommandGroup {

  public Stow(Arm arm, Elevator elevator, Intake intake) {

    addCommands(
      Commands.runOnce(() -> intake.stop(), intake),
      Commands.either(Commands.none(),
        Commands.either(
          new SequentialCommandGroup(
            Commands.runOnce(() -> arm.goToLocation(Constants.Arm.CarryAngle), arm),
            new WaitUntilCommand(() -> arm.isAtLocation())
            ) 
        ,
        new SequentialCommandGroup(
          Commands.runOnce(() -> arm.goToLocation(Constants.Arm.SafeCarryAngle), arm),
          new WaitUntilCommand(() -> arm.isAtLocation()),  
          Commands.runOnce(() -> elevator.goToLocation(Constants.Elevator.StowHeight), elevator),
          Commands.runOnce(() -> arm.goToLocation(Constants.Arm.Max), arm)
          ),
        () -> elevator.getLocation() < Constants.Elevator.HeightL3)
          ,
      () -> arm.armDegrees() < Constants.Arm.SafeCarryAngle),
  
      
      Commands.runOnce(() -> elevator.goToLocation(Constants.Elevator.StowHeight), elevator),
      Commands.waitUntil(() -> elevator.isAtLocation()),
      Commands.runOnce(() -> arm.goToLocation(Constants.Arm.Max), arm)   
    );
  }
}
