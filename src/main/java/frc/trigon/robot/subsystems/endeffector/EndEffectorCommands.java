package frc.trigon.robot.subsystems.endeffector;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import frc.trigon.robot.RobotContainer;
import lib.commands.NetworkTablesCommand;

import java.util.Set;

public class EndEffectorCommands {
    public static Command getDebuggingCommand() {
        return new NetworkTablesCommand(
                RobotContainer.END_EFFECTOR::setTargetState,
                false,
                Set.of(RobotContainer.END_EFFECTOR),
                "Debugging/EndEffectorTargetVoltage"
        );
    }

    public static Command getSetTargetStateCommand(EndEffectorConstants.EndEffectorState targetState) {
        return new StartEndCommand(
                () -> RobotContainer.END_EFFECTOR.setTargetState(targetState),
                RobotContainer.END_EFFECTOR::stop,
                RobotContainer.END_EFFECTOR
        );
    }
}
