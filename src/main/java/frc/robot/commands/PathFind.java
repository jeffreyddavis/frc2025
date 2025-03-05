package frc.robot.commands;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.util.Units;

public class PathFind {

    public static Command ToPose(Pose2d pose) {
        // Create the constraints to use while pathfinding
        PathConstraints constraints = new PathConstraints(
                3.0, 4.0,
                Units.degreesToRadians(540), Units.degreesToRadians(720));

        // Since AutoBuilder is configured, we can use it to build pathfinding commands
        Command pathfindingCommand = AutoBuilder.pathfindToPose(
                pose,
                constraints,
                0.0 // Goal end velocity in meters/sec
        );
        return pathfindingCommand;
    }

    public static Command FindPath(String pathName) {
            // Load the path we want to pathfind to and follow
            PathPlannerPath path;
            Command pathfindingCommand = Commands.none();

        try {
            path = PathPlannerPath.fromPathFile(pathName);
            // Create the constraints to use while pathfinding. The constraints defined in the path will only be used for the path.
            PathConstraints constraints = new PathConstraints(
                3.0, 3.0,
                Units.degreesToRadians(180), Units.degreesToRadians(720));

            // Since AutoBuilder is configured, we can use it to build pathfinding commands
            pathfindingCommand = AutoBuilder.pathfindThenFollowPath(
                path,
                constraints);
            
        }
        catch (Exception e) {

        }
        return pathfindingCommand;

        
    }
}

