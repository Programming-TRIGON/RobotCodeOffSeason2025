package frc.trigon.robot.commands.commandfactories;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.trigon.robot.RobotContainer;
import frc.trigon.robot.constants.OperatorConstants;
import frc.trigon.robot.subsystems.armelevator.ArmElevatorCommands;
import frc.trigon.robot.subsystems.armelevator.ArmElevatorConstants;
import frc.trigon.robot.subsystems.climber.ClimberCommands;
import frc.trigon.robot.subsystems.climber.ClimberConstants;
import frc.trigon.robot.subsystems.intake.IntakeCommands;
import frc.trigon.robot.subsystems.intake.IntakeConstants;
import frc.trigon.robot.subsystems.swerve.SwerveCommands;

public class ClimbCommands {
    private static boolean IS_CLIMBING = false;

    public static Command getClimbCommand() {
        return new SequentialCommandGroup(
                new InstantCommand(() -> IS_CLIMBING = true),
                ClimberCommands.getSetTargetStateCommand(ClimberConstants.ClimberState.BREAK_ZIP_TIE)
                        .until(() -> RobotContainer.CLIMBER.atState(ClimberConstants.ClimberState.BREAK_ZIP_TIE)),
                ClimberCommands.getSetTargetStateCommand(ClimberConstants.ClimberState.PREPARE_FOR_CLIMB)
                        .until(OperatorConstants.CONTINUE_TRIGGER),
                ClimberCommands.getSetTargetStateCommand(ClimberConstants.ClimberState.CLIMB)
                        .until(RobotContainer.CLIMBER::atTargetState),
                getAdjustClimbManuallyCommand()
        )
                .alongWith(getSetSubsystemsToRestForClimbCommand())
                .finallyDo(() -> IS_CLIMBING = false);
    }

    public static boolean isClimbing() {
        return IS_CLIMBING;
    }

    private static Command getAdjustClimbManuallyCommand() {
        return new ParallelCommandGroup(
                ClimberCommands.getSetTargetSpeedCommand(() -> OperatorConstants.DRIVER_CONTROLLER.getRightY() * 3),
                SwerveCommands.getClosedLoopSelfRelativeDriveCommand(
                        () -> 0,
                        () -> 0,
                        () -> 0
                ).asProxy()
        );
    }

    private static Command getSetSubsystemsToRestForClimbCommand() {
        return new ParallelCommandGroup(
                ArmElevatorCommands.getSetTargetStateCommand(ArmElevatorConstants.ArmElevatorState.REST_FOR_CLIMB),
                IntakeCommands.getSetTargetStateCommand(IntakeConstants.IntakeState.OPEN_REST)
        );
    }
}
