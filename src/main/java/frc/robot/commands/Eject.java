package frc.robot.commands;
import frc.robot.subsystems.*;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class Eject extends SequentialCommandGroup {
    
    public Eject(Intake intake) {
       
        addRequirements(intake);
        addCommands(
         Commands.runOnce(() -> intake.ShootAlgae(), intake)

        );

    }
}