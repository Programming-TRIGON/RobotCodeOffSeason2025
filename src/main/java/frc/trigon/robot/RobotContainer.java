// Copyright (c) FIRST and other WPILib contributors.

// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.trigon.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.trigon.robot.commands.CommandConstants;
import frc.trigon.robot.commands.commandfactories.*;
import frc.trigon.robot.constants.AutonomousConstants;
import frc.trigon.robot.constants.CameraConstants;
import frc.trigon.robot.constants.LEDConstants;
import frc.trigon.robot.constants.OperatorConstants;
import frc.trigon.robot.misc.objectdetectioncamera.ObjectPoseEstimator;
import frc.trigon.robot.misc.simulatedfield.SimulatedGamePieceConstants;
import frc.trigon.robot.misc.simulatedfield.SimulationFieldHandler;
import frc.trigon.robot.poseestimation.poseestimator.PoseEstimator;
import frc.trigon.robot.subsystems.MotorSubsystem;
import frc.trigon.robot.subsystems.armelevator.ArmElevator;
import frc.trigon.robot.subsystems.armelevator.ArmElevatorCommands;
import frc.trigon.robot.subsystems.climber.Climber;
import frc.trigon.robot.subsystems.climber.ClimberCommands;
import frc.trigon.robot.subsystems.climber.ClimberConstants;
import frc.trigon.robot.subsystems.endeffector.EndEffector;
import frc.trigon.robot.subsystems.endeffector.EndEffectorCommands;
import frc.trigon.robot.subsystems.endeffector.EndEffectorConstants;
import frc.trigon.robot.subsystems.intake.Intake;
import frc.trigon.robot.subsystems.intake.IntakeCommands;
import frc.trigon.robot.subsystems.intake.IntakeConstants;
import frc.trigon.robot.subsystems.swerve.Swerve;
import frc.trigon.robot.subsystems.transporter.Transporter;
import frc.trigon.robot.subsystems.transporter.TransporterCommands;
import frc.trigon.robot.subsystems.transporter.TransporterConstants;
import lib.utilities.flippable.Flippable;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

import java.util.List;

public class RobotContainer {
    public static final PoseEstimator ROBOT_POSE_ESTIMATOR = new PoseEstimator(
            CameraConstants.INTAKE_SIDE_REEF_TAG_CAMERA,
            CameraConstants.LEFT_REEF_TAG_CAMERA,
            CameraConstants.RIGHT_REEF_TAG_CAMERA
    );
    public static final ObjectPoseEstimator CORAL_POSE_ESTIMATOR = new ObjectPoseEstimator(
            CameraConstants.OBJECT_POSE_ESTIMATOR_DELETION_THRESHOLD_SECONDS,
            ObjectPoseEstimator.DistanceCalculationMethod.TRANSLATION,
            SimulatedGamePieceConstants.GamePieceType.CORAL,
            CameraConstants.OBJECT_DETECTION_CAMERA
    );
    public static final Swerve SWERVE = new Swerve();
    public static final ArmElevator ARM_ELEVATOR = new ArmElevator();
    public static final Climber CLIMBER = new Climber();
    public static final EndEffector END_EFFECTOR = new EndEffector();
    public static final Intake INTAKE = new Intake();
    public static final Transporter TRANSPORTER = new Transporter();

    private LoggedDashboardChooser<Command> autoChooser;

    public RobotContainer() {
        initializeGeneralSystems();
        buildAutoChooser();
        configureBindings();
    }

    /**
     * @return the command to run in autonomous mode
     */
    public Command getAutonomousCommand() {
        return autoChooser.get();
    }

    private void configureBindings() {
        bindDefaultCommands();
        bindControllerCommands();
    }

    private void bindDefaultCommands() {
        SWERVE.setDefaultCommand(GeneralCommands.getFieldRelativeDriveCommand());
        ARM_ELEVATOR.setDefaultCommand(ArmElevatorCommands.getDefaultCommand());
        CLIMBER.setDefaultCommand(ClimberCommands.getSetTargetStateCommand(ClimberConstants.ClimberState.REST));
        END_EFFECTOR.setDefaultCommand(EndEffectorCommands.getSetTargetStateCommand(EndEffectorConstants.EndEffectorState.REST));
        INTAKE.setDefaultCommand(IntakeCommands.getSetTargetStateCommand(IntakeConstants.IntakeState.REST));
        TRANSPORTER.setDefaultCommand(TransporterCommands.getSetTargetStateCommand(TransporterConstants.TransporterState.REST));
    }

    /**
     * Initializes the general systems of the robot.
     * Some systems need to be initialized at the start of the robot code so that others can use their functions.
     * For example, the LEDConstants need to be initialized so that the other systems can use them.
     */
    private void initializeGeneralSystems() {
        Flippable.init();
        LEDConstants.init();
        AutonomousConstants.init();
    }

    private void bindControllerCommands() {
        OperatorConstants.RESET_HEADING_TRIGGER.onTrue(CommandConstants.RESET_HEADING_COMMAND);
//        OperatorConstants.DRIVE_FROM_DPAD_TRIGGER.whileTrue(CommandConstants.SELF_RELATIVE_DRIVE_FROM_DPAD_COMMAND);
        OperatorConstants.TOGGLE_BRAKE_TRIGGER.onTrue(GeneralCommands.getToggleBrakeCommand());

        OperatorConstants.FLOOR_ALGAE_COLLECTION_TRIGGER.toggleOnTrue(AlgaeManipulationCommands.getFloorAlgaeCollectionCommand());
        OperatorConstants.REEF_ALGAE_COLLECTION_TRIGGER.toggleOnTrue(AlgaeManipulationCommands.getReefAlgaeCollectionCommand());

        OperatorConstants.CORAL_COLLECTION_TRIGGER.whileTrue(CoralCollectionCommands.getCoralCollectionCommand());
        OperatorConstants.SCORE_CORAL_LEFT_TRIGGER.whileTrue(CoralPlacingCommands.getScoreInReefCommand(false));
        OperatorConstants.SCORE_CORAL_RIGHT_TRIGGER.whileTrue(CoralPlacingCommands.getScoreInReefCommand(true));
        OperatorConstants.EJECT_CORAL_TRIGGER.whileTrue(CoralEjectionCommands.getCoralEjectionCommand());

        OperatorConstants.SPAWN_CORAL_IN_SIMULATION_TRIGGER.onTrue(new InstantCommand(() -> SimulationFieldHandler.updateCoralSpawning(ROBOT_POSE_ESTIMATOR.getEstimatedRobotPose())));
        OperatorConstants.FLIP_ARM_TRIGGER.onTrue(new InstantCommand(() -> OperatorConstants.SHOULD_FLIP_ARM_OVERRIDE = !OperatorConstants.SHOULD_FLIP_ARM_OVERRIDE));
        OperatorConstants.LOLLIPOP_ALGAE_TOGGLE_TRIGGER.onTrue(new InstantCommand(AlgaeManipulationCommands::toggleLollipopCollection));
        OperatorConstants.CLIMB_TRIGGER.toggleOnTrue(ClimbCommands.getClimbCommand());
    }

    private void configureSysIDBindings(MotorSubsystem subsystem) {
        OperatorConstants.FORWARD_QUASISTATIC_CHARACTERIZATION_TRIGGER.whileTrue(subsystem.getQuasistaticCharacterizationCommand(SysIdRoutine.Direction.kForward));
        OperatorConstants.BACKWARD_QUASISTATIC_CHARACTERIZATION_TRIGGER.whileTrue(subsystem.getQuasistaticCharacterizationCommand(SysIdRoutine.Direction.kReverse));
        OperatorConstants.FORWARD_DYNAMIC_CHARACTERIZATION_TRIGGER.whileTrue(subsystem.getDynamicCharacterizationCommand(SysIdRoutine.Direction.kForward));
        OperatorConstants.BACKWARD_DYNAMIC_CHARACTERIZATION_TRIGGER.whileTrue(subsystem.getDynamicCharacterizationCommand(SysIdRoutine.Direction.kReverse));
        subsystem.setDefaultCommand(Commands.idle(subsystem));
    }

    @SuppressWarnings("All")
    private void buildAutoChooser() {
        autoChooser = new LoggedDashboardChooser<>("AutoChooser");

        final List<String> autoNames = AutoBuilder.getAllAutoNames();
        boolean hasDefault = false;

        for (String autoName : autoNames) {
            final PathPlannerAuto autoNonMirrored = new PathPlannerAuto(autoName);
            final PathPlannerAuto autoMirrored = new PathPlannerAuto(autoName, true);

            if (!AutonomousConstants.DEFAULT_AUTO_NAME.isEmpty() && AutonomousConstants.DEFAULT_AUTO_NAME.equals(autoName)) {
                hasDefault = true;
                autoChooser.addDefaultOption(autoNonMirrored.getName(), autoNonMirrored);
                autoChooser.addOption(autoMirrored.getName() + "Mirrored", autoMirrored);
            } else if (!AutonomousConstants.DEFAULT_AUTO_NAME.isEmpty() && AutonomousConstants.DEFAULT_AUTO_NAME.equals(autoName + "Mirrored")) {
                hasDefault = true;
                autoChooser.addDefaultOption(autoMirrored.getName() + "Mirrored", autoMirrored);
                autoChooser.addOption(autoNonMirrored.getName(), autoNonMirrored);
            } else {
                autoChooser.addOption(autoNonMirrored.getName(), autoNonMirrored);
                autoChooser.addOption(autoMirrored.getName() + "Mirrored", autoMirrored);
            }
        }

        if (!hasDefault)
            autoChooser.addDefaultOption("None", Commands.none());
        else
            autoChooser.addOption("None", Commands.none());

        addCommandsToChooser(
                AutonomousCommands.getFloorAutonomousCommand(true),
                AutonomousCommands.getFloorAutonomousCommand(false)
        );
    }

    private void addCommandsToChooser(Command... commands) {
        for (Command command : commands)
            autoChooser.addOption(command.getName(), command);
    }
}