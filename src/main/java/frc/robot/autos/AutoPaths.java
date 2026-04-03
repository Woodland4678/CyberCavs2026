package frc.robot.autos;

import java.util.ArrayList;
import java.util.List;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.Constants;
import frc.robot.Constants.AutoWaypoint;
import frc.robot.lib.BLine.Path;
import frc.robot.lib.BLine.Path.TranslationTarget;
import frc.robot.lib.BLine.Path.Waypoint;
/********************************************************
 * Steps for new autos
 * 1. Create new list of AutoWaypoints
 * 2. Add left side of the same auto
 * 3. Create copy of AutoTemplate and rename it to a new auto
 * 4. In robot container add a new entry to the autos map defining the command and the paths to use
 * 
 **********************************************************/

public final class AutoPaths {
   
    static Pose2d leftReturnPose = new Pose2d(5.841, 5.622, new Rotation2d().fromDegrees(-90));
    static Pose2d rightReturnPose = new Pose2d(5.841, 2.47, new Rotation2d().fromDegrees(90));

   
    public static final List<AutoWaypoint[]> RightSideFullHopperThenClimb = List.of(
        // First segment
        new AutoWaypoint[] {
            //new AutoWaypoint(new Pose2d(6.242 , 5.276, new Rotation2d().fromDegrees(180)), 1.5, 2.0, Math.PI * 3, 0.15, 12),
            new AutoWaypoint(new Pose2d(7.404 , 1.703, new Rotation2d().fromDegrees(-145)), 1.5, 3.5, Math.PI * 3, 0.15, 12),
            new AutoWaypoint(new Pose2d(8.281 , 3.931, new Rotation2d().fromDegrees(-98)), 0.5, 1.0, Math.PI * 1, 0.04, 6),
        },
        new AutoWaypoint[] {
            new AutoWaypoint(new Pose2d(5.841, 2.351, new Rotation2d().fromDegrees(45)), 1.0, 4.2, Math.PI * 3, 0.05, 2)
        },
         new AutoWaypoint[] { //getting corral fuel
            new AutoWaypoint(new Pose2d(1.079, 2.496, new Rotation2d().fromDegrees(180)), 0.0, 3.5, Math.PI * 3, 0.02, 2)
        }
    );
    public static final List<AutoWaypoint[]> LeftSideMiddleCorralClimbPaths = List.of(
        // First segment
        new AutoWaypoint[] {
            //new AutoWaypoint(new Pose2d(6.242 , 5.276, new Rotation2d().fromDegrees(180)), 1.5, 2.0, Math.PI * 3, 0.15, 12),
            new AutoWaypoint(new Pose2d(7.240 , 5.064, new Rotation2d().fromDegrees(155)), 1.0, 1.5, Math.PI * 3, 0.15, 12),
            new AutoWaypoint(new Pose2d(7.620 , 4.900, new Rotation2d().fromDegrees(135)), 1.5, 2.0, Math.PI * 3, 0.08, 6),
            new AutoWaypoint(new Pose2d(7.950 , 4.550, new Rotation2d().fromDegrees(110)), 1.5, 2.0, Math.PI * 3, 0.08, 6),
            new AutoWaypoint(new Pose2d(8.060 , 4.100, new Rotation2d().fromDegrees(80)), 2.5, 2.0, Math.PI * 3, 0.08, 6),
            new AutoWaypoint(new Pose2d(7.980 , 3.850, new Rotation2d().fromDegrees(55)), 2.5, 2.0, Math.PI * 3, 0.08, 6),
            new AutoWaypoint(new Pose2d(7.840 , 3.751, new Rotation2d().fromDegrees(28.3)), 2.5, 4.0, Math.PI * 3, 0.08, 6)
        },
        new AutoWaypoint[] {
            new AutoWaypoint(new Pose2d(5.869, 5.390, new Rotation2d().fromDegrees(-45)), 1.0, 4.2, Math.PI * 3, 0.05, 2)
        },
         new AutoWaypoint[] { //getting corral fuel
            new AutoWaypoint(new Pose2d(0.400, 4.864, new Rotation2d().fromDegrees(-85)), 0.5, 2.5, Math.PI * 3, 0.10, 5),
            new AutoWaypoint(new Pose2d(0.400, 5.698, new Rotation2d().fromDegrees(-90)), 0.1, 0.75, Math.PI * 3, 0.15, 12),
            new AutoWaypoint(new Pose2d(0.400, 6.588, new Rotation2d().fromDegrees(-90)), 0.1, 0.75, Math.PI * 3, 0.05, 5),
            new AutoWaypoint(new Pose2d(1.812, 5.390, new Rotation2d().fromDegrees(-45)), 0.5, 1.5, Math.PI * 3, 0.05, 5)
        },
        new AutoWaypoint[] {
            new AutoWaypoint(new Pose2d(1.033, 4.968, new Rotation2d().fromDegrees(0)), 0.0, 2.0, Math.PI * 3, 0.01, 1)
        }
    );
    public static final List<AutoWaypoint[]> LeftSideMiddleThenSweepHub = List.of(
        new AutoWaypoint[] {
            //new AutoWaypoint(new Pose2d(6.242 , 5.276, new Rotation2d().fromDegrees(180)), 1.5, 2.0, Math.PI * 3, 0.15, 12),
            new AutoWaypoint(new Pose2d(7.540 , 5.064, new Rotation2d().fromDegrees(155)), 1.0, 2.5, Math.PI * 3, 0.15, 12),
            new AutoWaypoint(new Pose2d(7.920 , 4.900, new Rotation2d().fromDegrees(135)), 1.0, 1.5, Math.PI * 3, 0.08, 6),
            new AutoWaypoint(new Pose2d(8.250 , 4.550, new Rotation2d().fromDegrees(110)), 1.0, 1.5, Math.PI * 3, 0.08, 6),
            new AutoWaypoint(new Pose2d(8.360 , 4.100, new Rotation2d().fromDegrees(80)), 1.0, 1.5, Math.PI * 3, 0.08, 6),
            new AutoWaypoint(new Pose2d(8.280 , 3.850, new Rotation2d().fromDegrees(55)), 1.0, 1.5, Math.PI * 3, 0.08, 6),
            new AutoWaypoint(new Pose2d(8.140 , 3.751, new Rotation2d().fromDegrees(28.3)), 1.0, 1.5, Math.PI * 3, 0.08, 6),
            new AutoWaypoint(new Pose2d(6.47 , 4.861, new Rotation2d().fromDegrees(-75)), 1.0, 2.0, Math.PI * 3, 0.08, 6)
        },
        new AutoWaypoint[] {
            new AutoWaypoint((leftReturnPose), 0.0, 3.75, Math.PI * 3, 0.07, 0.0800)
        },
        //Sweep hub section
        new AutoWaypoint[] {
            //new AutoWaypoint(new Pose2d(6.242 , 5.276, new Rotation2d().fromDegrees(180)), 1.5, 2.0, Math.PI * 3, 0.15, 12),
            //new AutoWaypoint(new Pose2d(7.029 , 5.169, new Rotation2d().fromDegrees(-125)), 0.0, 0.0, Math.PI * 3, 100.0, 0.04),
            new AutoWaypoint(new Pose2d(7.034 , 4.116, new Rotation2d().fromDegrees(90)), 1.0, 1.5, Math.PI * 3, 0.10, 6),
            new AutoWaypoint(new Pose2d(7.034 , 3.468, new Rotation2d().fromDegrees(90)), 1.0, 1.5, Math.PI * 2, 0.10, 6),
            new AutoWaypoint(new Pose2d(6.632 , 3.168, new Rotation2d().fromDegrees(0)), 1.0, 1.5, Math.PI * 2, 0.10, 6),
            new AutoWaypoint(new Pose2d(6.23 , 3.468, new Rotation2d().fromDegrees(-90)), 0.0, 2.5, Math.PI * 3, 0.10, 6),
            new AutoWaypoint((leftReturnPose), 0.0, 3.0, Math.PI * 3, 0.07, 0.08)
        }
    );

    public static final List<AutoWaypoint[]> LeftSideDisturbedMiddlePathOption1 = List.of(
        new AutoWaypoint[] {
            //new AutoWaypoint(new Pose2d(6.242 , 5.276, new Rotation2d().fromDegrees(180)), 1.5, 2.0, Math.PI * 3, 0.15, 12),
            new AutoWaypoint(new Pose2d(6.640, 5.322, new Rotation2d().fromDegrees(142)), 1.0, 1.0, Math.PI * 3, 0.15, 12),
            new AutoWaypoint(new Pose2d(7.597, 4.559, new Rotation2d().fromDegrees(90)), 1.5, 1.0, Math.PI * 3, 0.08, 6),
            new AutoWaypoint(new Pose2d(7.025, 3.524, new Rotation2d().fromDegrees(30)), 2.5, 1.0, Math.PI * 3, 0.08, 6),            
            new AutoWaypoint(new Pose2d(6.096, 4.326, new Rotation2d().fromDegrees(-72)), 2.5, 1.0, Math.PI * 3, 0.08, 6)
        },
        new AutoWaypoint[] {
            new AutoWaypoint((leftReturnPose), 0.0, 4.2, Math.PI * 3, 0.05, 2)
        },
        //Sweep hub section
        new AutoWaypoint[] {
            new AutoWaypoint(new Pose2d(6.242 , 5.276, new Rotation2d().fromDegrees(180)), 1.5, 2.0, Math.PI * 3, 0.15, 12),
            new AutoWaypoint(new Pose2d(7.029 , 5.169, new Rotation2d().fromDegrees(-125)), 0.0, 0.0, Math.PI * 3, 100.0, 0.04),
           new AutoWaypoint(new Pose2d(6.934 , 4.116, new Rotation2d().fromDegrees(90)), 1.5, 3.0, Math.PI * 3, 0.10, 6),
            new AutoWaypoint(new Pose2d(6.934 , 3.468, new Rotation2d().fromDegrees(90)), 1.5, 3.0, Math.PI * 2, 0.10, 6),
           new AutoWaypoint(new Pose2d(6.532 , 3.168, new Rotation2d().fromDegrees(0)), 1.5, 3.0, Math.PI * 2, 0.10, 6),
           new AutoWaypoint(new Pose2d(6.13 , 3.468, new Rotation2d().fromDegrees(-90)), 0.0, 2.5, Math.PI * 3, 0.10, 6),
            new AutoWaypoint((leftReturnPose), 0.0, 3.0, Math.PI * 3, 0.10, 6)
        }
    );
    
    public static final List<AutoWaypoint[]> LeftSideDisturbedMiddlePathOption2 = List.of(
        new AutoWaypoint[] {
            //new AutoWaypoint(new Pose2d(6.242 , 5.276, new Rotation2d().fromDegrees(180)), 1.5, 2.0, Math.PI * 3, 0.15, 12),
            new AutoWaypoint(new Pose2d(6.148, 5.102, new Rotation2d().fromDegrees(90)), 1.0, 1.0, Math.PI * 3, 0.15, 12),
            new AutoWaypoint(new Pose2d(6.148, 3.990, new Rotation2d().fromDegrees(90)), 1.5, 1.0, Math.PI * 3, 0.08, 6),
            new AutoWaypoint(new Pose2d(8.128, 3.990, new Rotation2d().fromDegrees(-180)), 2.5, 1.0, Math.PI * 3, 0.08, 6),            
            new AutoWaypoint(new Pose2d(7.261, 5.258, new Rotation2d().fromDegrees(-39.04)), 2.5, 1.0, Math.PI * 3, 0.08, 6)
        },
        new AutoWaypoint[] {
            new AutoWaypoint((leftReturnPose), 0.0, 4.0, Math.PI * 3, 0.05, 0.0875)
        },
        //Sweep hub section
        new AutoWaypoint[] {
            new AutoWaypoint(new Pose2d(6.242 , 5.276, new Rotation2d().fromDegrees(180)), 1.5, 2.0, Math.PI * 3, 0.15, 12),
            new AutoWaypoint(new Pose2d(7.029 , 5.169, new Rotation2d().fromDegrees(-125)), 0.0, 0.0, Math.PI * 3, 100.0, 0.04),
           new AutoWaypoint(new Pose2d(6.934 , 4.116, new Rotation2d().fromDegrees(90)), 1.5, 3.0, Math.PI * 3, 0.10, 6),
            new AutoWaypoint(new Pose2d(6.934 , 3.468, new Rotation2d().fromDegrees(90)), 1.5, 3.0, Math.PI * 2, 0.10, 6),
           new AutoWaypoint(new Pose2d(6.532 , 3.168, new Rotation2d().fromDegrees(0)), 1.5, 3.0, Math.PI * 2, 0.10, 6),
           new AutoWaypoint(new Pose2d(6.13 , 3.468, new Rotation2d().fromDegrees(-90)), 0.0, 2.5, Math.PI * 3, 0.10, 6),
            new AutoWaypoint((leftReturnPose), 0.0, 3.0, Math.PI * 3, 0.10, 6)
        }
    );
    
    public static final List<AutoWaypoint[]> LeftSideHubSweepCorralClimb = List.of(
        // First segment
        new AutoWaypoint[] {
            //new AutoWaypoint(new Pose2d(6.242 , 5.276, new Rotation2d().fromDegrees(180)), 1.5, 2.0, Math.PI * 3, 0.15, 12),
            new AutoWaypoint(new Pose2d(5.857 , 4.812, new Rotation2d().fromDegrees(90)), 1.0, 1.5, Math.PI * 3, 0.15, 12),
            new AutoWaypoint(new Pose2d(5.857 , 3.501, new Rotation2d().fromDegrees(90)), 1.5, 2.0, Math.PI * 3, 0.08, 6),
            new AutoWaypoint(new Pose2d(6.2455 , 3.189, new Rotation2d().fromDegrees(-135)), 1.5, 2.0, Math.PI * 2, 0.08, 6),
            new AutoWaypoint(new Pose2d(6.634 , 3.889, new Rotation2d().fromDegrees(-90)), 1.5, 2.0, Math.PI * 3, 0.08, 6),
            new AutoWaypoint(new Pose2d(6.440 , 4.942, new Rotation2d().fromDegrees(-90)), 2.5, 2.0, Math.PI * 3, 0.08, 6)
        },
        new AutoWaypoint[] {
            new AutoWaypoint(leftReturnPose, 1.0, 4.2, Math.PI * 3, 0.05, 2)
        },
         new AutoWaypoint[] { //getting corral fuel
            new AutoWaypoint(new Pose2d(0.400, 4.864, new Rotation2d().fromDegrees(-85)), 0.5, 2.5, Math.PI * 3, 0.10, 5),
            new AutoWaypoint(new Pose2d(0.400, 5.698, new Rotation2d().fromDegrees(-90)), 0.1, 0.75, Math.PI * 3, 0.15, 12),
            new AutoWaypoint(new Pose2d(0.400, 6.588, new Rotation2d().fromDegrees(-90)), 0.1, 0.75, Math.PI * 3, 0.05, 5),
            new AutoWaypoint(new Pose2d(1.812, 5.390, new Rotation2d().fromDegrees(-45)), 0.5, 1.5, Math.PI * 3, 0.05, 5)
        },
        new AutoWaypoint[] {
            new AutoWaypoint(new Pose2d(1.033, 4.968, new Rotation2d().fromDegrees(0)), 0.0, 2.0, Math.PI * 3, 0.01, 1)
        }
    );

    public static final List<AutoWaypoint[]> LeftSideSweepWallCorral = List.of(
        // First segment
        new AutoWaypoint[] {
            //new AutoWaypoint(new Pose2d(6.242 , 5.276, new Rotation2d().fromDegrees(180)), 1.5, 2.0, Math.PI * 3, 0.15, 12),
            new AutoWaypoint(new Pose2d(7.428 , 5.962, new Rotation2d().fromDegrees(-145)), 1.0, 1.5, Math.PI * 3, 0.15, 12),
            new AutoWaypoint(new Pose2d(8.189 , 7.020, new Rotation2d().fromDegrees(-90)), 2.5, 3.0, Math.PI * 3, 0.08, 6),
            new AutoWaypoint(new Pose2d(5.911 , 7.020, new Rotation2d().fromDegrees(-90)), 2.5, 4.2, Math.PI * 2, 0.08, 6),
            //new AutoWaypoint(new Pose2d(6.634 , 3.889, new Rotation2d().fromDegrees(-90)), 1.5, 2.0, Math.PI * 3, 0.08, 6),
            //new AutoWaypoint(new Pose2d(6.440 , 4.942, new Rotation2d().fromDegrees(-90)), 2.5, 2.0, Math.PI * 3, 0.08, 6)
        },
        new AutoWaypoint[] {
            new AutoWaypoint(leftReturnPose, 1.0, 4.2, Math.PI * 3, 0.05, 2)
        },
         new AutoWaypoint[] { //getting corral fuel
            new AutoWaypoint(new Pose2d(0.400, 4.864, new Rotation2d().fromDegrees(-85)), 0.5, 2.5, Math.PI * 3, 0.10, 5),
            new AutoWaypoint(new Pose2d(0.400, 5.698, new Rotation2d().fromDegrees(-90)), 0.1, 0.75, Math.PI * 3, 0.15, 12),
            new AutoWaypoint(new Pose2d(0.400, 6.588, new Rotation2d().fromDegrees(-90)), 0.1, 0.75, Math.PI * 3, 0.05, 5),
            new AutoWaypoint(new Pose2d(1.812, 5.390, new Rotation2d().fromDegrees(-45)), 0.5, 1.5, Math.PI * 3, 0.05, 5)
        },
        new AutoWaypoint[] {
            new AutoWaypoint(new Pose2d(1.033, 4.968, new Rotation2d().fromDegrees(0)), 0.0, 2.0, Math.PI * 3, 0.01, 1)
        }
    );
    

    public static final List<AutoWaypoint[]> LeftSideTestCorral = List.of(
        // First segment
         new AutoWaypoint[] { //getting corral fuel
            new AutoWaypoint(new Pose2d(0.400, 4.864, new Rotation2d().fromDegrees(-85)), 0.5, 2.5, Math.PI * 3, 0.10, 5),
            new AutoWaypoint(new Pose2d(0.400, 5.698, new Rotation2d().fromDegrees(-90)), 0.1, 0.75, Math.PI * 3, 0.15, 12),
            new AutoWaypoint(new Pose2d(0.400, 6.588, new Rotation2d().fromDegrees(-90)), 0.1, 0.75, Math.PI * 3, 0.05, 5),
            new AutoWaypoint(new Pose2d(1.812, 5.390, new Rotation2d().fromDegrees(-45)), 0.5, 1.5, Math.PI * 3, 0.05, 5)
        },
        new AutoWaypoint[] {
            new AutoWaypoint(new Pose2d(1.033, 4.968, new Rotation2d().fromDegrees(0)), 0.0, 2.0, Math.PI * 3, 0.01, 1)
        }
    );
    public static final List<AutoWaypoint[]> LeftSideTestCorral2 = List.of(
        // First segment
         new AutoWaypoint[] { //getting corral fuel
            new AutoWaypoint(new Pose2d(15.58, 2.77, new Rotation2d().fromDegrees(144)), 0.5, 2.5, Math.PI * 3, 0.03, 0.05),
            new AutoWaypoint(new Pose2d(15.67, 1.79, new Rotation2d().fromDegrees(150)), 0.1, 0.75, Math.PI * 3, 0.02, 0.05),
           // new AutoWaypoint(new Pose2d(0.400, 6.588, new Rotation2d().fromDegrees(-90)), 0.1, 0.75, Math.PI * 3, 0.05, 5),
            //new AutoWaypoint(new Pose2d(1.812, 5.390, new Rotation2d().fromDegrees(-45)), 0.5, 1.5, Math.PI * 3, 0.05, 5)
        },
        new AutoWaypoint[] {
            new AutoWaypoint(new Pose2d(1.033, 4.968, new Rotation2d().fromDegrees(0)), 0.0, 2.0, Math.PI * 3, 0.01, 1)
        }
    );
    public static final List<AutoWaypoint[]> LeftSideDisturbedMiddlePathOption2ThenDepot = List.of(
        LeftSideDisturbedMiddlePathOption2.get(0),
        new AutoWaypoint[] {
            new AutoWaypoint((leftReturnPose), 0.0, 4.0, Math.PI * 3, 0.05, 0.0875)
        },
        //Sweep hub section
        LeftSideTestCorral.get(0),
        LeftSideTestCorral.get(1)
    );

    public static final List<AutoWaypoint[]> DriveToClimberLeftSide = List.of(
        // First segment
        new AutoWaypoint[] {
            new AutoWaypoint(new Pose2d(15.552, 3.156, new Rotation2d().fromDegrees(180)), 0.0, 2.0, Math.PI * 3, 0.01, 2)
            //new AutoWaypoint(new Pose2d(15.492, 3.43, new Rotation2d().fromDegrees(180)), 0.0, 1.0, Math.PI * 3, 0.02, 2),
        },
        // Second segment
        new AutoWaypoint[] {
            new AutoWaypoint(new Pose2d(6.391, 1.330, new Rotation2d().fromDegrees(-90)), 1.5, 2.0, Math.PI * 3, 0.15, 12),
            new AutoWaypoint(new Pose2d(7.914, 0.974, new Rotation2d().fromDegrees(-90)), 1.0, 1.5, Math.PI * 3, 0.15, 12),
            new AutoWaypoint(new Pose2d(7.914, 2.48, new Rotation2d().fromDegrees(-90)), 1.0, 1.5, Math.PI * 3, 0.15, 12),
            new AutoWaypoint(new Pose2d(5.873, 2.399, new Rotation2d().fromDegrees(45)), 2, 3.0, Math.PI * 3, 0.15, 6)
        }
    );


    public static final List<AutoWaypoint[]> RightSideShallowSemicircle = List.of(
        new AutoWaypoint[] {
            //new AutoWaypoint(new Pose2d(6.242 , 5.276, new Rotation2d().fromDegrees(180)), 1.5, 2.0, Math.PI * 3, 0.15, 12),
            new AutoWaypoint(new Pose2d(7.3950 , 3.041, new Rotation2d().fromDegrees(180)), 1.0, 3.5, Math.PI * 2, 0.15, 12),
            new AutoWaypoint(new Pose2d(8.108 , 3.041, new Rotation2d().fromDegrees(180)), 1.0, 1.75, Math.PI * 2, 0.08, 6),
            new AutoWaypoint(new Pose2d(8.529 , 2.570, new Rotation2d().fromDegrees(90)), 1.0, 1.75, Math.PI * 2, 0.08, 6),
            new AutoWaypoint(new Pose2d(8.108 , 2.399, new Rotation2d().fromDegrees(0)), 1.0, 1.75, Math.PI * 2, 0.08, 6),
            new AutoWaypoint(new Pose2d(7.039 , 2.399, new Rotation2d().fromDegrees(0)), 1.0, 2.0, Math.PI * 2, 0.1, 6)
        },
        new AutoWaypoint[] {
            new AutoWaypoint(rightReturnPose, 0.0, 3.2, Math.PI * 3, 0.08, 0.080)
        },
        //Sweep hub section
        new AutoWaypoint[] {
            //new AutoWaypoint(new Pose2d(6.242 , 5.276, new Rotation2d().fromDegrees(180)), 1.5, 2.0, Math.PI * 3, 0.15, 12),
            //new AutoWaypoint(new Pose2d(5.792 , 3.565, new Rotation2d().fromDegrees(180)), 1.0, 3.5, Math.PI * 3, 0.15, 12),
            new AutoWaypoint(new Pose2d(6.391 , 4.197, new Rotation2d().fromDegrees(133)), 1.5, 2.0, Math.PI * 3, 0.08, 6),
            new AutoWaypoint(new Pose2d(7.379 , 4.537, new Rotation2d().fromDegrees(180)), 1.5, 2.0, Math.PI * 2, 0.08, 6),
            new AutoWaypoint(new Pose2d(7.962 , 4.408, new Rotation2d().fromDegrees(135)), 1.5, 2.0, Math.PI * 2, 0.08, 6),
            new AutoWaypoint(new Pose2d(8.157 , 4.109, new Rotation2d().fromDegrees(90)), 1.5, 2.0, Math.PI * 3, 0.08, 6),
            new AutoWaypoint(new Pose2d(7.849 , 3.565, new Rotation2d().fromDegrees(45)), 1.5, 2.0, Math.PI * 2, 0.08, 6),
            new AutoWaypoint(new Pose2d(7.541 , 3.565, new Rotation2d().fromDegrees(0)), 2.5, 2.0, Math.PI * 3, 0.08, 6)
        }
    );
    public static final List<AutoWaypoint[]> RightSideFullHopperThenFill = List.of(
        new AutoWaypoint[] {
            //new AutoWaypoint(new Pose2d(6.242 , 5.276, new Rotation2d().fromDegrees(180)), 1.5, 2.0, Math.PI * 3, 0.15, 12),
            new AutoWaypoint(new Pose2d(7.039 , 3.258, new Rotation2d().fromDegrees(125)), 1.0, 3.5, Math.PI * 3, 0.15, 12),
            new AutoWaypoint(new Pose2d(8.189 , 2.172, new Rotation2d().fromDegrees(160)), 0.0, 1.0, Math.PI * 3, 0.08, 6),
        },
        new AutoWaypoint[] {
            new AutoWaypoint((rightReturnPose), 0.0, 4.0, Math.PI * 3, 0.05, 0.0875)
        },
        //Sweep hub section
        new AutoWaypoint[] {
            new AutoWaypoint(new Pose2d(6.242 , 2.793, new Rotation2d().fromDegrees(180)), 1.5, 2.0, Math.PI * 3, 0.15, 12),
           new AutoWaypoint(new Pose2d(6.934 , 3.953, new Rotation2d().fromDegrees(90)), 1.5, 3.0, Math.PI * 3, 0.10, 6),
            new AutoWaypoint(new Pose2d(6.934 , 4.601, new Rotation2d().fromDegrees(90)), 1.5, 3.0, Math.PI * 2, 0.10, 6),
           new AutoWaypoint(new Pose2d(6.532 , 4.901, new Rotation2d().fromDegrees(0)), 1.5, 3.0, Math.PI * 2, 0.10, 6),
           new AutoWaypoint(new Pose2d(6.13 , 4.601, new Rotation2d().fromDegrees(-90)), 0.0, 2.5, Math.PI * 3, 0.10, 6),
           // new AutoWaypoint((rightReturnPose), 0.0, 3.0, Math.PI * 3, 0.06, 0.0875)
        }
    );
     public static final List<AutoWaypoint[]> RightSideSemiCircleThenSweepHub = List.of(
        new AutoWaypoint[] {
            //new AutoWaypoint(new Pose2d(6.242 , 5.276, new Rotation2d().fromDegrees(180)), 1.5, 2.0, Math.PI * 3, 0.15, 12),
           //new AutoWaypoint(new Pose2d(7.3950 , 3.141, new Rotation2d().fromDegrees(130)), 0.0, 0.0, Math.PI * 3, 100, 0.087),
            //new AutoWaypoint(new Pose2d(7.3950 , 3.141, new Rotation2d().fromDegrees(180)), 1.0, 3.5, Math.PI * 3, 0.08, 0.087),
            new AutoWaypoint(new Pose2d(8.108 , 3.141, new Rotation2d().fromDegrees(180)), 1.5, 2.0, Math.PI * 3, 0.08, 6),
            new AutoWaypoint(new Pose2d(8.529 , 2.82, new Rotation2d().fromDegrees(90)), 1.5, 2.0, Math.PI * 3, 0.08, 6),
            new AutoWaypoint(new Pose2d(8.108 , 2.499, new Rotation2d().fromDegrees(0)), 1.5, 2.0, Math.PI * 3, 0.08, 6),
            new AutoWaypoint(new Pose2d(7.039 , 2.499, new Rotation2d().fromDegrees(45)), 2.5, 4.0, Math.PI * 3, 0.08, 6)
        },
        new AutoWaypoint[] {
            new AutoWaypoint(new Pose2d(5.841, 2.47, new Rotation2d().fromDegrees(90)), 0.0, 4.2, Math.PI * 3, 0.07, 2)
        },
        //Sweep hub section
        mirrorBlueRightToLeft(LeftSideMiddleThenSweepHub.get(2), Constants.FIELD_WIDTH_METERS)
    );
    

    public static final List<AutoWaypoint[]> RightSideDisruption = List.of(
        new AutoWaypoint[] {
            //new AutoWaypoint(new Pose2d(6.242 , 5.276, new Rotation2d().fromDegrees(180)), 1.5, 2.0, Math.PI * 3, 0.15, 12),
            new AutoWaypoint(new Pose2d(7.136 , 2.237, new Rotation2d().fromDegrees(180)), 2.0, 3.75, Math.PI * 3, 0.15, 12),
            new AutoWaypoint(new Pose2d(7.930 , 2.237, new Rotation2d().fromDegrees(-133.134)), 2.75, 3.75, Math.PI * 3, 0.15, 12),
            new AutoWaypoint(new Pose2d(8.400 , 2.317, new Rotation2d().fromDegrees(-89.842)), 2.75, 3.75, Math.PI * 3, 0.15, 6),
            new AutoWaypoint(new Pose2d(8.643 , 3.548, new Rotation2d().fromDegrees(-42.692)), 2.75, 3.75, Math.PI * 3, 0.15, 6),
            new AutoWaypoint(new Pose2d(8.254 , 4.301, new Rotation2d().fromDegrees(47.490)), 2.75, 3.75, Math.PI * 3, 0.15, 6),
            new AutoWaypoint(new Pose2d(6.877 , 2.49, new Rotation2d().fromDegrees(24.775)), 2.75, 3.75, Math.PI * 3, 0.15, 6),
        },
        new AutoWaypoint[] {
            new AutoWaypoint(new Pose2d(5.841, 2.49, new Rotation2d().fromDegrees(45)), 1.0, 4.2, Math.PI * 3, 0.02, 2)
        },
        mirrorBlueRightToLeft(LeftSideMiddleThenSweepHub.get(2), Constants.FIELD_WIDTH_METERS)
        //Sweep hub section
        // new AutoWaypoint[] {
        //     //new AutoWaypoint(new Pose2d(6.242 , 5.276, new Rotation2d().fromDegrees(180)), 1.5, 2.0, Math.PI * 3, 0.15, 12),
        //     //new AutoWaypoint(new Pose2d(5.792 , 3.565, new Rotation2d().fromDegrees(180)), 1.0, 3.5, Math.PI * 3, 0.15, 12),
        //     new AutoWaypoint(new Pose2d(6.391 , 4.197, new Rotation2d().fromDegrees(180)), 1.5, 2.0, Math.PI * 3, 0.08, 6),
        //     new AutoWaypoint(new Pose2d(7.379 , 4.537, new Rotation2d().fromDegrees(180)), 1.5, 2.0, Math.PI * 2, 0.08, 6),
        //     new AutoWaypoint(new Pose2d(7.962 , 4.408, new Rotation2d().fromDegrees(135)), 1.5, 2.0, Math.PI * 2, 0.08, 6),
        //     new AutoWaypoint(new Pose2d(8.157 , 4.109, new Rotation2d().fromDegrees(90)), 1.5, 2.0, Math.PI * 3, 0.08, 6),
        //     new AutoWaypoint(new Pose2d(7.849 , 3.565, new Rotation2d().fromDegrees(45)), 1.5, 2.0, Math.PI * 2, 0.08, 6),
        //     new AutoWaypoint(new Pose2d(7.541 , 3.565, new Rotation2d().fromDegrees(0)), 2.5, 2.0, Math.PI * 3, 0.08, 6)
        // }
    );
    public static final List<AutoWaypoint[]> RightSideDisruptionLegal = List.of(
        new AutoWaypoint[] {
            //new AutoWaypoint(new Pose2d(6.242 , 5.276, new Rotation2d().fromDegrees(180)), 1.5, 2.0, Math.PI * 3, 0.15, 12),
            new AutoWaypoint(new Pose2d(7.136 , 2.237, new Rotation2d().fromDegrees(180)), 2.0, 3.75, Math.PI * 3, 0.15, 12),
            new AutoWaypoint(new Pose2d(7.730 , 2.237, new Rotation2d().fromDegrees(-133.134)), 2.75, 3.75, Math.PI * 3, 0.15, 12),
            new AutoWaypoint(new Pose2d(8.200 , 2.317, new Rotation2d().fromDegrees(-89.842)), 2.75, 3.75, Math.PI * 3, 0.15, 6),
            new AutoWaypoint(new Pose2d(8.443 , 3.548, new Rotation2d().fromDegrees(-42.692)), 2.75, 3.75, Math.PI * 3, 0.15, 6),
            new AutoWaypoint(new Pose2d(8.054 , 4.301, new Rotation2d().fromDegrees(47.490)), 2.75, 3.75, Math.PI * 3, 0.15, 6),
            new AutoWaypoint(new Pose2d(6.677 , 2.49, new Rotation2d().fromDegrees(24.775)), 2.75, 3.75, Math.PI * 3, 0.15, 6),
        },
        new AutoWaypoint[] {
            new AutoWaypoint(new Pose2d(5.841, 2.49, new Rotation2d().fromDegrees(45)), 1.0, 4.2, Math.PI * 3, 0.02, 2)
        },
        mirrorBlueRightToLeft(LeftSideMiddleThenSweepHub.get(2), Constants.FIELD_WIDTH_METERS)
        //Sweep hub section
        // new AutoWaypoint[] {
        //     //new AutoWaypoint(new Pose2d(6.242 , 5.276, new Rotation2d().fromDegrees(180)), 1.5, 2.0, Math.PI * 3, 0.15, 12),
        //     //new AutoWaypoint(new Pose2d(5.792 , 3.565, new Rotation2d().fromDegrees(180)), 1.0, 3.5, Math.PI * 3, 0.15, 12),
        //     new AutoWaypoint(new Pose2d(6.391 , 4.197, new Rotation2d().fromDegrees(180)), 1.5, 2.0, Math.PI * 3, 0.08, 6),
        //     new AutoWaypoint(new Pose2d(7.379 , 4.537, new Rotation2d().fromDegrees(180)), 1.5, 2.0, Math.PI * 2, 0.08, 6),
        //     new AutoWaypoint(new Pose2d(7.962 , 4.408, new Rotation2d().fromDegrees(135)), 1.5, 2.0, Math.PI * 2, 0.08, 6),
        //     new AutoWaypoint(new Pose2d(8.157 , 4.109, new Rotation2d().fromDegrees(90)), 1.5, 2.0, Math.PI * 3, 0.08, 6),
        //     new AutoWaypoint(new Pose2d(7.849 , 3.565, new Rotation2d().fromDegrees(45)), 1.5, 2.0, Math.PI * 2, 0.08, 6),
        //     new AutoWaypoint(new Pose2d(7.541 , 3.565, new Rotation2d().fromDegrees(0)), 2.5, 2.0, Math.PI * 3, 0.08, 6)
        // }
    );
    
   
    public static final List<AutoWaypoint[]> LeftSideSemiCircleThenSweepHub =
    RightSideSemiCircleThenSweepHub.stream()
        .map(segment -> mirrorBlueRightToLeft(segment, Constants.FIELD_WIDTH_METERS))
        .toList();

    public static final List<AutoWaypoint[]> LeftSideDisruption =
    RightSideDisruption.stream()
        .map(segment -> mirrorBlueRightToLeft(segment, Constants.FIELD_WIDTH_METERS))
        .toList();

    public static final List<AutoWaypoint[]> LeftSideDisruptionLegal =
    RightSideDisruptionLegal.stream()
        .map(segment -> mirrorBlueRightToLeft(segment, Constants.FIELD_WIDTH_METERS))
        .toList();


    public static final List<AutoWaypoint[]> LeftSideShallowSemicircle =
    RightSideShallowSemicircle.stream()
        .map(segment -> mirrorBlueRightToLeft(segment, Constants.FIELD_WIDTH_METERS))
        .toList();

    public static final List<AutoWaypoint[]> RightSideMiddleThenSweepHub = LeftSideMiddleThenSweepHub.stream()
        .map(segment -> mirrorBlueRightToLeft(segment, Constants.FIELD_WIDTH_METERS))
        .toList();

    public static final List<AutoWaypoint[]> LeftSideDisruptionDepot = List.of(
        // First segment
        LeftSideDisruption.get(0),
        LeftSideDisruption.get(1),
         new AutoWaypoint[] { //getting corral fuel
            new AutoWaypoint(new Pose2d(1.06, 5.229, new Rotation2d().fromDegrees(-36)), 0.5, 2.5, Math.PI * 3, 0.03, 0.05),
            new AutoWaypoint(new Pose2d(0.94, 6.279, new Rotation2d().fromDegrees(-30)), 0.1, 0.75, Math.PI * 3, 0.02, 0.05),
            new AutoWaypoint(new Pose2d(0.94, 6.479, new Rotation2d().fromDegrees(0)), 0.1, 0.75, Math.PI * 3, 0.02, 0.03)
            //new AutoWaypoint(new Pose2d(1.812, 5.390, new Rotation2d().fromDegrees(-45)), 0.5, 1.5, Math.PI * 3, 0.05, 5)
        },
        new AutoWaypoint[] {
           new AutoWaypoint(new Pose2d(2.294, 5.152, new Rotation2d().fromDegrees(-20)), 0.5, 2.0, Math.PI * 3, 0.05, 5),
        }
    );

    public static final List<Path> rightSideBLineVsTrench = List.of(
        new Path("ReturnToBump"),
        new Path("RunMiddle"),
        new Path("SweepHub")
    );
    public static final List<Path> ZachAndEliSpecial = List.of(
        new Path("ReturnToBump"),
        new Path("Test2"),
        new Path("AntiSid")
    );
     public static final List<Path> rightSideShallowSemiCircle = List.of(
        new Path("ReturnToBump"),
        new Path("ShallowSemiCircle"),
        new Path("ShallowSemiCircle2")
    );
     public static final List<Path> rightSideDisruption = List.of(
        new Path("ReturnToBump"),
        new Path("Disruption"),
        new Path("SweepHub")
    );
    public static final List<Path> rightSideDisruptionReverseSweep = List.of(
        new Path("ReturnToBump"),
        new Path("Disruption"),
        new Path("ReverseHubSweep")
    );

    public static final List<Path> leftSideBLineVsTrench;

    static {
        // initialize the left side paths in a static block
        Path p0 = new Path("ReturnToBump");
        p0.mirror();

        Path p1 = new Path("RunMiddle");
        p1.mirror();

        Path p2 = new Path("SweepHub");
        p2.mirror();

        leftSideBLineVsTrench = List.of(p0, p1, p2);
    }
    public static final List<Path> leftSideDisruption;

    static {
        // initialize the left side paths in a static block
        Path p0 = new Path("ReturnToBump");
        p0.mirror();

        Path p1 = new Path("Disruption");
        p1.mirror();

        Path p2 = new Path("SweepHub");
        p2.mirror();

        leftSideDisruption = List.of(p0, p1, p2);
    }
    public static final List<Path> leftSideShallowSemiCircle;

    static {
        // initialize the left side paths in a static block
        Path p0 = new Path("ReturnToBump");
        p0.mirror();

        Path p1 = new Path("ShallowSemiCircle");
        p1.mirror();

        Path p2 = new Path("ShallowSemiCircle2");
        p2.mirror();

        leftSideShallowSemiCircle = List.of(p0, p1, p2);
    }

    public static Pose2d[] extractPoses(List<AutoWaypoint[]> waypointSegments) {
        // Count total number of waypoints
        int total = waypointSegments.stream().mapToInt(arr -> arr.length).sum();
        Pose2d[] poses = new Pose2d[total];

        int index = 0;
        for (AutoWaypoint[] segment : waypointSegments) {
            for (AutoWaypoint wp : segment) {
                poses[index++] = wp.waypoint;
            }
        }

        return poses;
    }


    public static AutoWaypoint[] mirrorBlueRightToLeft(AutoWaypoint[] original,
        double fieldWidthMeters
    ) {
        AutoWaypoint[] out = new AutoWaypoint[original.length];

        for (int i = 0; i < original.length; i++) {
            AutoWaypoint wp = original[i];
            out[i] = new AutoWaypoint(
                mirrorBlueRightToLeft(wp.waypoint, fieldWidthMeters),
                wp.cruiseSpeed,
                wp.maxSpeed,
                wp.maxAngularSpeed,
                wp.translationTolerance,
                wp.rotationTolerance
            );
        }
        return out;
    }
    public static Pose2d mirrorBlueRightToLeft(Pose2d original, double fieldWidthMeters) {
        return new Pose2d(
            original.getX(),
            fieldWidthMeters - original.getY(),
            original.getRotation().unaryMinus()
        );
    }
    public static Pose2d rotateBlueToRed(
        Pose2d original,
        double fieldLengthMeters,
        double fieldWidthMeters
    ) {
        return new Pose2d(
            fieldLengthMeters - original.getX(),
            fieldWidthMeters  - original.getY(),
            original.getRotation().plus(Rotation2d.fromRadians(Math.PI))
        );
    }
    public static AutoWaypoint[] rotateBlueToRed(
        AutoWaypoint[] original,
        double fieldLengthMeters,
        double fieldWidthMeters
    ) {
        AutoWaypoint[] out = new AutoWaypoint[original.length];

        for (int i = 0; i < original.length; i++) {
            AutoWaypoint wp = original[i];
            out[i] = new AutoWaypoint(
                rotateBlueToRed(wp.waypoint, fieldLengthMeters, fieldWidthMeters),
                wp.cruiseSpeed,
                wp.maxSpeed,
                wp.maxAngularSpeed,
                wp.translationTolerance,
                wp.rotationTolerance
            );
        }
        return out;
    }

}
