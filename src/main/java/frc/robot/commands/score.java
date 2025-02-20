package frc.robot.commands;
import frc.robot.subsystems.*;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class score extends SequentialCommandGroup {
    
    public score(Intake intake) {
       
        addRequirements(intake);
        addCommands(
         Commands.runOnce(() -> intake.ShootCoral(), intake),
         Commands.waitSeconds(.5),
         Commands.runOnce(() -> intake.stop(), intake)

        );

    }
}