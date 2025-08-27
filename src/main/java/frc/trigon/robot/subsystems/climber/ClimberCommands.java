package frc.trigon.robot.subsystems.climber;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import frc.trigon.robot.RobotContainer;
import trigon.commands.NetworkTablesCommand;

import java.util.Set;

public class ClimberCommands {
    public static Command getDebuggingCommand() {
        return new NetworkTablesCommand(
                RobotContainer.CLIMBER::setTargetState,
                false,
                Set.of(RobotContainer.CLIMBER),
                "TargetPositionRotations",
                "TargetServoSpeed"
        );
    }

    public static Command getSetTargetStateCommand(ClimberConstants.ClimberState targetState) {
        return new StartEndCommand(
                () -> RobotContainer.CLIMBER.setTargetState(targetState),
                RobotContainer.CLIMBER::stop,
                RobotContainer.CLIMBER
        );
    }

    public static Command getSetTargetStateCommand(double targetPositionRotations, double targetServoSpeed) {
        return new StartEndCommand(
                () -> RobotContainer.CLIMBER.setTargetState(targetPositionRotations, targetServoSpeed),
                RobotContainer.CLIMBER::stop,
                RobotContainer.CLIMBER
        );
    }
}
