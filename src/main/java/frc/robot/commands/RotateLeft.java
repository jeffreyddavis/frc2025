package frc.robot.commands;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.DeferredCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import frc.robot.RobotContainer;
import frc.robot.addons.ScoringLocations;
import frc.robot.subsystems.*;
import frc.robot.subsystems.drive.Drive;        
import frc.robot.commands.*;

import java.util.Map;

import edu.wpi.first.math.geometry.Pose2d;

public class RotateLeft  extends SequentialCommandGroup {

    public RotateLeft(Drive drive, Arm arm, Elevator elevator, RobotContainer rob, CommandJoystick driverController) {
        addCommands(
            new ParallelCommandGroup(
                new DriveToPose(ScoringLocations.rotateLeft(drive, DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red), drive, driverController, true),
                new GoToTarget(rob, arm, elevator)
                
            ),
            new DriveToPose(ScoringLocations.ActualScoringLocation, drive, driverController, false)




            
        );
    }
}
