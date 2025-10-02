package frc.trigon.robot.constants;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathfindingCommand;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.pathfinding.Pathfinding;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import frc.trigon.robot.RobotContainer;
import lib.hardware.RobotHardwareStats;
import lib.utilities.LocalADStarAK;
import lib.utilities.flippable.Flippable;
import lib.utilities.flippable.FlippablePose2d;
import org.json.simple.parser.ParseException;

import java.io.IOException;

/**
 * A class that contains the constants and configurations for everything related to the 15-second autonomous period at the start of the match.
 */
public class AutonomousConstants {
    public static final String DEFAULT_AUTO_NAME = "DefaultAutoName";
    public static final RobotConfig ROBOT_CONFIG = getRobotConfig();
    public static final double FEEDFORWARD_SCALAR = 0.5;//TODO: Calibrate
    public static final PathConstraints DRIVE_TO_REEF_CONSTRAINTS = new PathConstraints(2.5, 4, Units.degreesToRadians(450), Units.degreesToRadians(900));
    public static final double MINIMUM_DISTANCE_FROM_REEF_TO_OPEN_ELEVATOR = 2.2;
    public static final double
            REEF_RELATIVE_X_TOLERANCE_METERS = 0.085,
            REEF_RELATIVE_Y_TOLERANCE_METERS = 0.03;
    
    private static final double
            AUTO_FIND_CORAL_POSE_X = 3.3,
            AUTO_FIND_CORAL_POSE_LEFT_Y = 5.5,
            AUTO_FIND_FIRST_CORAL_POST_LEFT_Y = 6.5;
    private static final Rotation2d
            AUTO_FIND_CORAL_POSE_LEFT_ROTATION = Rotation2d.fromDegrees(130),
            AUTO_FIND_FIRST_CORAL_POSE_LEFT_ROTATION = Rotation2d.fromDegrees(170);
    public static final FlippablePose2d
            AUTO_FIND_CORAL_POSE_LEFT = new FlippablePose2d(AUTO_FIND_CORAL_POSE_X, AUTO_FIND_CORAL_POSE_LEFT_Y, AUTO_FIND_CORAL_POSE_LEFT_ROTATION, true),
            AUTO_FIND_CORAL_POSE_RIGHT = new FlippablePose2d(AUTO_FIND_CORAL_POSE_X, FieldConstants.FIELD_WIDTH_METERS - AUTO_FIND_CORAL_POSE_LEFT_Y, AUTO_FIND_CORAL_POSE_LEFT_ROTATION.unaryMinus(), true),
            AUTO_FIND_FIRST_CORAL_POSE_LEFT = new FlippablePose2d(AUTO_FIND_CORAL_POSE_X, AUTO_FIND_FIRST_CORAL_POST_LEFT_Y, AUTO_FIND_FIRST_CORAL_POSE_LEFT_ROTATION, true),
            AUTO_FIND_FIRST_CORAL_POSE_RIGHT = new FlippablePose2d(AUTO_FIND_CORAL_POSE_X, FieldConstants.FIELD_WIDTH_METERS - AUTO_FIND_FIRST_CORAL_POST_LEFT_Y, AUTO_FIND_FIRST_CORAL_POSE_LEFT_ROTATION.unaryMinus(), true);
    public static final double
            AUTO_FIND_CORAL_END_VELOCITY_METERS_PER_SECOND = 2.5,
            AUTO_FIND_CORAL_ROTATION_POWER = 0.2;

    private static final PIDConstants
            AUTO_TRANSLATION_PID_CONSTANTS = RobotHardwareStats.isSimulation() ?
            new PIDConstants(0, 0, 0) :
            new PIDConstants(0, 0, 0),
            AUTO_ROTATION_PID_CONSTANTS = RobotHardwareStats.isSimulation() ?
                    new PIDConstants(0, 0, 0) :
                    new PIDConstants(0, 0, 0);


    public static final PIDController GAME_PIECE_AUTO_DRIVE_Y_PID_CONTROLLER = RobotHardwareStats.isSimulation() ?
            new PIDController(0.5, 0, 0) :
            new PIDController(0.3, 0, 0.03);
    public static final ProfiledPIDController GAME_PIECE_AUTO_DRIVE_X_PID_CONTROLLER = RobotHardwareStats.isSimulation() ?
            new ProfiledPIDController(0.5, 0, 0, new TrapezoidProfile.Constraints(2.8, 5)) :
            new ProfiledPIDController(2.4, 0, 0, new TrapezoidProfile.Constraints(2.65, 5.5));
    public static final double AUTO_COLLECTION_INTAKE_OPEN_CHECK_DISTANCE_METERS = 2;

    private static final PPHolonomicDriveController AUTO_PATH_FOLLOWING_CONTROLLER = new PPHolonomicDriveController(
            AUTO_TRANSLATION_PID_CONSTANTS,
            AUTO_ROTATION_PID_CONSTANTS
    );

    /**
     * Initializes PathPlanner. This needs to be called before any PathPlanner function can be used.
     */
    public static void init() {
        Pathfinding.setPathfinder(new LocalADStarAK());
        PathfindingCommand.warmupCommand().schedule();
        configureAutoBuilder();
        registerCommands();
    }

    private static void configureAutoBuilder() {
        AutoBuilder.configure(
                RobotContainer.ROBOT_POSE_ESTIMATOR::getEstimatedRobotPose,
                RobotContainer.ROBOT_POSE_ESTIMATOR::resetPose,
                RobotContainer.SWERVE::getSelfRelativeVelocity,
                (chassisSpeeds -> RobotContainer.SWERVE.drivePathPlanner(chassisSpeeds, true)),
                AUTO_PATH_FOLLOWING_CONTROLLER,
                ROBOT_CONFIG,
                Flippable::isRedAlliance,
                RobotContainer.SWERVE
        );
    }

    private static RobotConfig getRobotConfig() {
        try {
            return RobotConfig.fromGUISettings();
        } catch (IOException | ParseException e) {
            throw new RuntimeException(e);
        }
    }

    private static void registerCommands() {
        //TODO: Implement
    }
}