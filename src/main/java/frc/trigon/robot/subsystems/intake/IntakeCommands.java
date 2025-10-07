package frc.trigon.robot.subsystems.intake;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import frc.trigon.robot.RobotContainer;
import frc.trigon.robot.constants.OperatorConstants;
import frc.trigon.robot.subsystems.swerve.SwerveCommands;
import lib.commands.ExecuteEndCommand;
import lib.commands.NetworkTablesCommand;

import java.util.Set;

public class IntakeCommands {
    public static Command getDebuggingCommand() {
        return new NetworkTablesCommand(
                (targetAngleDegrees, targetVoltage) -> RobotContainer.INTAKE.setTargetState(Rotation2d.fromDegrees(targetAngleDegrees), targetVoltage),
                false,
                Set.of(RobotContainer.INTAKE),
                "Debugging/IntakeTargetAngleDegrees",
                "Debugging/IntakeTargetVoltage"
        );
    }

    public static Command resetIntakePositionCommand() {
        return new ExecuteEndCommand(
                () -> RobotContainer.INTAKE.setAngleMotorVoltage(-OperatorConstants.DRIVER_CONTROLLER.getRightX() * 2),
                RobotContainer.INTAKE::resetIntakePosition,
                RobotContainer.INTAKE
        ).alongWith(SwerveCommands.getOpenLoopFieldRelativeDriveCommand(() -> 0, () -> 0, () -> 0));
    }

    public static Command getPrepareForStateCommand(IntakeConstants.IntakeState targetState) {
        return new StartEndCommand(
                () -> RobotContainer.INTAKE.prepareForState(targetState),
                RobotContainer.INTAKE::stop,
                RobotContainer.INTAKE
        );
    }

    public static Command getSetTargetStateCommand(IntakeConstants.IntakeState targetState) {
        return new StartEndCommand(
                () -> RobotContainer.INTAKE.setTargetState(targetState),
                RobotContainer.INTAKE::stop,
                RobotContainer.INTAKE
        );
    }

    public static Command getSetTargetStateCommand(Rotation2d targetAngle, double targetVoltage) {
        return new StartEndCommand(
                () -> RobotContainer.INTAKE.setTargetState(targetAngle, targetVoltage),
                RobotContainer.INTAKE::stop,
                RobotContainer.INTAKE
        );
    }
}