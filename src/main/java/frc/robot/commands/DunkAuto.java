package frc.robot.commands;
import frc.robot.subsystems.*;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.drive.Drive;

public class DunkAuto extends SequentialCommandGroup {
    
    public DunkAuto(Intake intake, Elevator elevator, Drive drive) {
       

        addRequirements(intake, elevator);
        addCommands(
            Commands.runOnce(() -> elevator.dunk(), elevator), 
            Commands.runOnce(() -> intake.ShootCoral(), intake),
            Commands.waitSeconds(.2),
            Commands.runOnce(() -> intake.stop(), intake)
         
        );

    }
}