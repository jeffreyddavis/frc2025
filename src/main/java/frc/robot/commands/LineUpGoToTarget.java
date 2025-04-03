package frc.robot.commands;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import frc.robot.RobotContainer;
import frc.robot.addons.ScoringLocations;
import frc.robot.subsystems.*;
import frc.robot.subsystems.drive.Drive;        

public class LineUpGoToTarget  extends SequentialCommandGroup {

    public LineUpGoToTarget(Drive drive, Arm arm, Elevator elevator, RobotContainer rob, CommandJoystick driverController) {
        addCommands(
            new ParallelRaceGroup(
                new SequentialCommandGroup(
                    new ParallelCommandGroup(
                        new DriveToPose(ScoringLocations.getClosestScoringLocation(drive.getPose(), DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red),drive, driverController, 4.0),
                        new GoToTarget(rob, arm, elevator)
                        
                    ),
                    new DriveToPose(ScoringLocations.ActualScoringLocation, drive, driverController, 1.0)
                ),
                Commands.waitUntil(() -> { return 
                    driverController.getRawAxis(0) > .3 || 
                    driverController.getRawAxis(1) > .3 ||
                    driverController.getRawAxis(2) > .3 || 
                    driverController.button(1).getAsBoolean()
                    ;
                }))
            );
        
    }
}
