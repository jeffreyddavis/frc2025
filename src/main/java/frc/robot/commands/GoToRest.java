package frc.robot.commands;
import frc.robot.Constants;
import frc.robot.subsystems.*;
import frc.robot.subsystems.Elevator.Targets;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;


public class GoToRest extends SequentialCommandGroup {
    
    public GoToRest(Arm arm, Elevator elevator) {
       
        addRequirements(arm, elevator);


        if (!elevator.armClear()) {

            addCommands(
                Commands.runOnce(() -> elevator.goToLocation(Constants.Elevator.ArmClearHeight), elevator),
                Commands.waitUntil(() -> elevator.isAtLocation())

            );
        
        }
        addCommands(
            Commands.runOnce(() -> arm.goToLocation(Constants.Arm.DownAngle), arm),
            Commands.waitUntil(() -> arm.isAtLocation()),
            Commands.runOnce(() -> elevator.GoToTarget(Targets.Intake), elevator)
        );
        

    }
}