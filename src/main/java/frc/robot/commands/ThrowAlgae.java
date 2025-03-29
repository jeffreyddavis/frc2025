package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.subsystems.*;

public class ThrowAlgae extends SequentialCommandGroup {

  public ThrowAlgae(Arm arm, Intake intake, Elevator elevator) {

    addRequirements(intake, elevator);
    addCommands(
        Commands.runOnce(() -> arm.goToLocation(Constants.Arm.StraightOut), arm),
        
        Commands.waitUntil(() -> arm.isNearLocation()),
        
        Commands.runOnce(() -> elevator.goToLocation(8000), elevator),


        Commands.runOnce(() -> arm.goToLocation(Constants.Arm.WindUpAngle), arm),
        Commands.waitUntil(() -> arm.isAtLocation()),
        Commands.waitUntil(() -> elevator.isAtLocation()),
        Commands.runOnce(()-> elevator.setMaxSpeedMode(true), elevator),



        Commands.runOnce(() -> arm.setMaxSpeedMode(true), arm), 

        Commands.runOnce(() -> elevator.goToLocation(10000), elevator),


        Commands.runOnce(() -> arm.goToLocation(Constants.Arm.Max+10), arm),
        
        Commands.waitSeconds(.05),
        
        Commands.runOnce(() -> intake.ShootAlgae(), intake),
        Commands.waitSeconds(.4),
        
        Commands.runOnce(() -> arm.setMaxSpeedMode(false), arm), 
        Commands.runOnce(()-> elevator.setMaxSpeedMode(false), elevator),


        
        Commands.runOnce(() -> intake.stop(), intake),
        Commands.runOnce(() -> arm.goToLocation(Constants.Arm.CarryAngle), arm)
    );

  }
}
