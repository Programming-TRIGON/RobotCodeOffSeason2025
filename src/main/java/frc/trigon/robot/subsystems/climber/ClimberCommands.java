package frc.trigon.robot.subsystems.climber;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import frc.trigon.robot.RobotContainer;
import trigon.commands.ExecuteEndCommand;
import trigon.commands.NetworkTablesCommand;

import java.util.Set;

public class ClimberCommands {
    public static Command getDebuggingCommand() {
        return new NetworkTablesCommand(
                (targetAngleDegrees, targetServoSpeed) -> RobotContainer.CLIMBER.setTargetState(Rotation2d.fromDegrees(targetAngleDegrees), targetServoSpeed, false),
                false,
                Set.of(RobotContainer.CLIMBER),
                "Debugging/TargetAngleDegrees",
                "Debugging/TargetServoSpeed"
        );
    }

    public static Command getSetTargetStateCommand(ClimberConstants.ClimberState targetState) {
        return new StartEndCommand(
                () -> RobotContainer.CLIMBER.setTargetState(targetState),
                RobotContainer.CLIMBER::stop,
                RobotContainer.CLIMBER
        );
    }

    public static Command getSetTargetStateCommand(Rotation2d targetAngle, double targetServoSpeed, boolean affectedByRobotWeight) {
        return new StartEndCommand(
                () -> RobotContainer.CLIMBER.setTargetState(targetAngle, targetServoSpeed, affectedByRobotWeight),
                RobotContainer.CLIMBER::stop,
                RobotContainer.CLIMBER
        );
    }

    public static Command getSetTargetSpeedCommand(double targetSpeed) {
        return new ExecuteEndCommand(
                () -> RobotContainer.CLIMBER.setTargetVoltage(targetSpeed * ClimberConstants.MAXIMUM_MANUAL_CONTROL_VOLTAGE),
                RobotContainer.CLIMBER::stop,
                RobotContainer.CLIMBER
        );
    }
}
