package frc.robot.addons;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

import com.pathplanner.lib.util.FlippingUtil;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.subsystems.drive.*;

public class ScoringLocations {

    

    public static Pose2d[]  LineUpLocations = new Pose2d[12];
    public static Pose2d[] RedLineUpLocations = new Pose2d[12];

    public static Pose2d[] CoralLocations = new Pose2d[2];
    public static Pose2d[] RedCoralLocations = new Pose2d[2];

    @AutoLogOutput
    public static Pose2d ActualScoringLocation;
    public static Pose2d ActualCoralLocation;
    public static boolean hasValidLocation = false;

    @AutoLogOutput
    public static int locationIndex;
    public static int coralLocationIndex;

    public static void initScoringLocations() { // counterclockwise increasing from above
        LineUpLocations[0] = DriverStationLeftLineup;
        LineUpLocations[1] = DriverStationRightLineup;

        LineUpLocations[2] = CoralStationRightLeftLineup;
        LineUpLocations[3] = CoralStationRightRightLineup;

        LineUpLocations[4] = ProcessorLeftLineup;
        LineUpLocations[5] = ProcessorRightLineup;

        LineUpLocations[6] = BargeSideLeftLineup;
        LineUpLocations[7] = BargeSideRightLineup;

        LineUpLocations[8] = OtherSideLeftLineup;
        LineUpLocations[9] = OtherSideRightLineup;

        LineUpLocations[10] = CoralStationLeftLeftLineup;
        LineUpLocations[11] = CoralStationLeftRightLineup;

        for (int x = 0; x<LineUpLocations.length; x++) {
            RedLineUpLocations[x] = FlippingUtil.flipFieldPose(LineUpLocations[x]);
        }

        CoralLocations[0] = BlueCoralRightLineup;
        CoralLocations[1] = BlueCoralLeftLineup;

        RedCoralLocations[0] = FlippingUtil.flipFieldPose(CoralLocations[0]);
        RedCoralLocations[1] = FlippingUtil.flipFieldPose(CoralLocations[1]);
        
    }

    public static Pose2d rotateLeft(Drive drive, boolean isRed) {
        if (!hasValidLocation) return drive.getPose();

        locationIndex--;
        if (locationIndex < 0) locationIndex =11;
        Pose2d location = isRed ? RedLineUpLocations[locationIndex] : LineUpLocations[locationIndex];
        ActualScoringLocation = actualScoringLocation(isRed);
        return location;
    }

    public static Pose2d rotateRight(Drive drive, boolean isRed) {
        if (!hasValidLocation) return drive.getPose();

        

        locationIndex++;
        if (locationIndex > 11) locationIndex =0;
        Logger.recordOutput("target index", locationIndex);

        Pose2d location = isRed ? RedLineUpLocations[locationIndex] : LineUpLocations[locationIndex];
        Logger.recordOutput("coral target", location);

        ActualScoringLocation = actualScoringLocation(isRed);
        return location;

    }

    @AutoLogOutput
    public static Pose2d getClosestScoringLocation(Pose2d currentPose2d, boolean isRed) {
        double bestDistance = 100000;
        Pose2d bestPose = currentPose2d;

        if (isRed) {
            for (int x=0; x<RedLineUpLocations.length; x++) {
                double testDistance = currentPose2d.getTranslation().getDistance(RedLineUpLocations[x].getTranslation());
    
                if (testDistance < bestDistance) {
                    bestPose = RedLineUpLocations[x];
                    bestDistance = testDistance;
                    locationIndex = x;
                    hasValidLocation = true;
                }
            } 
        } else {
            for (int x=0; x<LineUpLocations.length; x++) {
                double testDistance = currentPose2d.getTranslation().getDistance(LineUpLocations[x].getTranslation());
    
                if (testDistance < bestDistance) {
                    bestPose = LineUpLocations[x];
                    bestDistance = testDistance;
                    locationIndex = x;
                    hasValidLocation = true;
                }
            }
        }
        

        Logger.recordOutput("target Location", bestPose);
        ActualScoringLocation = actualScoringLocation(isRed);
        return bestPose;
    }

    public static Pose2d actualCoralStation(boolean isRed) {
        Pose2d location;
        switch (coralLocationIndex) {
            case 0:
                location = BlueCoralRight;
                break;
            case 1:
                location = BlueCoralLeft;
                break;
            default:
                location = BlueCoralRight;
        }
        return isRed ? FlippingUtil.flipFieldPose(location) : location;
    }

    public static Pose2d actualScoringLocation(boolean isRed) {
        Pose2d location;
        switch(locationIndex) {
            case 0:
                location = DriverStationLeft;
                break;
            case 1:
                location = DriverStationRight;
                break;
            case 2:
                location = CoralStationRightLeft;
                break;
            case 3: 
                location = CoralStationRightRight;
                break;
            case 4:
                location = ProcessorLeft;
                break;
            case 5:
                location = ProcessorRight;
                break;
            case 6:
                location = BargeSideLeft;
                break;
            case 7: 
                location = BargeSideRight;
                break;
            case 8:
                location = OtherSideLeft;
                break;
            case 9:
                location = OtherSideRight;
                break;
            case 10:
                location = CoralStationLeftLeft;
                break;
            case 11:
                location = CoralStationLeftRight;
                break;
            default: 
                location = DriverStationLeft;
        }
        return isRed ? FlippingUtil.flipFieldPose(location) : location;
    }


    public static Pose2d OtherSideLeft = new Pose2d(5.36, 5.16, Rotation2d.fromDegrees(-120));
    public static Pose2d OtherSideLeftLineup = new Pose2d(5.75, 5.5, Rotation2d.fromDegrees(-120));

    public static Pose2d OtherSideRight = new Pose2d(5.05, 5.31, Rotation2d.fromDegrees(-120));
    public static Pose2d OtherSideRightLineup = new Pose2d(5.41, 5.63, Rotation2d.fromDegrees(-120));


    public static Pose2d CoralStationLeftLeft = new Pose2d(3.93, 5.23, Rotation2d.fromDegrees(-60));
    public static Pose2d CoralStationLeftLeftLineup = new Pose2d(3.73, 5.62, Rotation2d.fromDegrees(-60));

    public static Pose2d CoralStationLeftRight = new Pose2d(3.66, 5.14, Rotation2d.fromDegrees(-60));
    public static Pose2d CoralStationLeftRightLineup = new Pose2d(3.40, 5.64, Rotation2d.fromDegrees(-60));


    public static Pose2d BargeSideRight = new Pose2d(5.85, 4.19, Rotation2d.fromDegrees(180));
    public static Pose2d BargeSideRightLineup = new Pose2d(5.99, 4.19, Rotation2d.fromDegrees(180));

    public static Pose2d BargeSideLeft = new Pose2d(5.85, 3.89, Rotation2d.fromDegrees(180));
    public static Pose2d BargeSideLeftLineup = new Pose2d(5.99, 3.89, Rotation2d.fromDegrees(180));


    public static Pose2d DriverStationRight = new Pose2d(3.11, 3.84, Rotation2d.fromDegrees(0));
    public static Pose2d DriverStationRightLineup = new Pose2d(2.85, 3.84, Rotation2d.fromDegrees(0));

    public static Pose2d DriverStationLeft = new Pose2d(3.08, 4.19, Rotation2d.fromDegrees(0));
    public static Pose2d DriverStationLeftLineup = new Pose2d(2.85, 4.19, Rotation2d.fromDegrees(0));


    public static Pose2d CoralStationRightRight = new Pose2d(3.93, 2.75, Rotation2d.fromDegrees(60));
    public static Pose2d CoralStationRightRightLineup = new Pose2d(3.77, 2.55, Rotation2d.fromDegrees(60));

    public static Pose2d CoralStationRightLeft = new Pose2d(3.61, 2.93, Rotation2d.fromDegrees(60));
    public static Pose2d CoralStationRightLeftLineup = new Pose2d(3.47, 2.84, Rotation2d.fromDegrees(60));


    public static Pose2d ProcessorLeft = new Pose2d(5.03, 2.74, Rotation2d.fromDegrees(120));
    public static Pose2d ProcessorLeftLineup = new Pose2d(5.17, 2.45, Rotation2d.fromDegrees(120));

    public static Pose2d ProcessorRight = new Pose2d(5.305, 2.93, Rotation2d.fromDegrees(120));
    public static Pose2d ProcessorRightLineup = new Pose2d(5.30, 2.93, Rotation2d.fromDegrees(120));




    public static Pose2d BlueCoralRight = new Pose2d(1.56, .55, Rotation2d.fromDegrees(-126));
    public static Pose2d BlueCoralRightLineup = new Pose2d(2.18, 1.20, Rotation2d.fromDegrees(-126));

    public static Pose2d BlueCoralLeft = new Pose2d(1.56, 7.49, Rotation2d.fromDegrees(126));
    public static Pose2d BlueCoralLeftLineup = new Pose2d(2.18, 6.93, Rotation2d.fromDegrees(126));



    public static Pose2d getClosestCoralLocation(Pose2d currentPose2d, boolean isRed) {
        double bestDistance = 100000;
        Pose2d bestPose = currentPose2d;

        if (isRed) {
            for (int x=0; x<RedCoralLocations.length; x++) {
                double testDistance = currentPose2d.getTranslation().getDistance(RedCoralLocations[x].getTranslation());
    
                if (testDistance < bestDistance) {
                    bestPose = RedCoralLocations[x];
                    bestDistance = testDistance;
                    coralLocationIndex = x;
                    hasValidLocation = true;
                }
            } 
        } else {
            for (int x=0; x<CoralLocations.length; x++) {
                double testDistance = currentPose2d.getTranslation().getDistance(CoralLocations[x].getTranslation());
    
                if (testDistance < bestDistance) {
                    bestPose = CoralLocations[x];
                    bestDistance = testDistance;
                    coralLocationIndex = x;
                    hasValidLocation = true;
                }
            }
        }
        

        Logger.recordOutput("target Location", bestPose);
        ActualCoralLocation = actualCoralStation(isRed);
        return bestPose;
    
    }


}
