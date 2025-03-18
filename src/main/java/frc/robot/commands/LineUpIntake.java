package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import frc.robot.addons.ScoringLocations;
import frc.robot.subsystems.*;
import frc.robot.subsystems.drive.*;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Commands;


public class LineUpIntake extends SequentialCommandGroup {
    public LineUpIntake(Drive drive, Intake intake, Arm arm, Elevator elevator, CommandJoystick driverController ) {
        addCommands(
            new ParallelCommandGroup(
                new DriveToPose(ScoringLocations.getClosestCoralLocation(drive.getPose(), DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red), drive, driverController, true),
                new IntakeAutoNoWait(intake, arm, elevator)
            ),
            new DriveToPose(ScoringLocations.ActualCoralLocation, drive, driverController, false),
            Commands.waitUntil(() -> intake.SeesCoral())
            
        );
    }
}
