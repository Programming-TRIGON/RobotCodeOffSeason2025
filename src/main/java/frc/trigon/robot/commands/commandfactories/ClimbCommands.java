package frc.trigon.robot.commands.commandfactories;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.trigon.robot.RobotContainer;
import frc.trigon.robot.constants.OperatorConstants;
import frc.trigon.robot.subsystems.climber.ClimberCommands;
import frc.trigon.robot.subsystems.climber.ClimberConstants;
import frc.trigon.robot.subsystems.swerve.SwerveCommands;

public class ClimbCommands {
    public static boolean IS_CLIMBING = false;

    public static Command getClimbCommand() {
        return new SequentialCommandGroup(
                new InstantCommand(() -> IS_CLIMBING = true),
                ClimberCommands.getSetTargetStateCommand(ClimberConstants.ClimberState.PREPARE_FOR_CLIMB)
                        .until(() -> RobotContainer.CLIMBER.hasCage() || OperatorConstants.CONTINUE_TRIGGER.getAsBoolean()),
                ClimberCommands.getSetTargetStateCommand(ClimberConstants.ClimberState.CLIMB)
                        .until(RobotContainer.CLIMBER::atTargetState),
                new ParallelCommandGroup(
                        ClimberCommands.getSetTargetSpeedCommand(OperatorConstants.DRIVER_CONTROLLER.getRightY()),
                        SwerveCommands.getClosedLoopSelfRelativeDriveCommand(
                                () -> 0,
                                () -> 0,
                                () -> 0
                        )
                )
        ).finallyDo(() -> IS_CLIMBING = false);
    }
}
