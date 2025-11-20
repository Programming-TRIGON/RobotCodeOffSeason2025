package lib.commands;

import com.ctre.phoenix6.controls.VoltageOut;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import lib.hardware.phoenix6.talonfx.TalonFXMotor;
import lib.hardware.phoenix6.talonfx.TalonFXSignal;
import org.littletonrobotics.junction.Logger;

import java.util.HashMap;

public class ArmCalibrationCommand extends Command {
    //Find the gravity offset, kG, and kS
    private final TalonFXMotor motor;
    private static final double VOLTAGE_INCREMENT = 0.001;
    private Rotation2d gravityOffset;
    private double kG, kS, currentVoltage, minimumVoltage, maximumVoltage;
    private boolean isCalculationFinished = false;
    private double previousVelocity;
    HashMap<Double, Rotation2d> name = new HashMap<>();


    public ArmCalibrationCommand(TalonFXMotor motor, SubsystemBase... requirements) {
        this.motor = motor;
        addRequirements(requirements);
    }

    @Override
    public void initialize() {
    }


    @Override
    public void execute() {
        runCalculateGravityOffset();
        System.out.println(isVelocityIncreasing() + "\ncurrent velocity: " + motor.getSignal(TalonFXSignal.VELOCITY) + " \nprevious velocity: " + previousVelocity);
        System.out.println("\nmaximumPosition: " + gravityOffset.getRotations() + "\nminimumPosition: " +  name.get(minimumVoltage).getRotations());
    }

    @Override
    public boolean isFinished() {
        return isCalculationFinished;
    }

    @Override
    public void end(boolean interrupted) {
        System.out.println(interrupted);
        calculateKG();
        calculateKS();
        logValues();
        printResults();
    }

    private void logValues() {
        Logger.recordOutput("/SmartDashboard/ArmCalibrationCommand/GravityOffset", gravityOffset);
        Logger.recordOutput("/SmartDashboard/ArmCalibrationCommand/kG", kG);
        Logger.recordOutput("/SmartDashboard/ArmCalibrationCommand/kS", kS);
    }

    private void printResults() {
        System.out.println("GravityOffset: " + gravityOffset);
        System.out.println("kG: " + kG);
        System.out.println("kS: " + kS);
        System.out.println("Maximum Voltage: " + maximumVoltage);
        System.out.println("Minimum Voltage: " + minimumVoltage);
    }

    private void runCalculateGravityOffset() {
        if (isArmStoppedMoving()) {
            currentVoltage += VOLTAGE_INCREMENT;
            motor.setControl(new VoltageOut(currentVoltage));
            logMotorSignalsToHashmap();
        }
        if (isVelocityIncreasing()) {
            maximumVoltage = currentVoltage - VOLTAGE_INCREMENT;
            gravityOffset = name.get(maximumVoltage);
            getMinimumVoltage();
            isCalculationFinished = true;
        }
    }

    private void getMinimumVoltage() {
        minimumVoltage = maximumVoltage;
        while ((Math.abs(gravityOffset.getRotations() - name.get(minimumVoltage).getRotations()) > 0.0001)) {
            minimumVoltage -= VOLTAGE_INCREMENT;
        }
    }

    private boolean isVelocityIncreasing() {
        return Math.abs(motor.getSignal(TalonFXSignal.VELOCITY) - previousVelocity) > 0.001;
    }

    private boolean isArmStoppedMoving() {
        return Math.abs(motor.getSignal(TalonFXSignal.VELOCITY)) < 0.01;
    }

    private void logMotorSignalsToHashmap() {
        name.put(motor.getSignal(TalonFXSignal.MOTOR_VOLTAGE), Rotation2d.fromRotations(motor.getSignal(TalonFXSignal.POSITION)));
        previousVelocity = motor.getSignal(TalonFXSignal.VELOCITY);
    }

    private void calculateKG() {
        kG = (maximumVoltage + minimumVoltage) / 2;
    }

    private void calculateKS() {
        kS = (maximumVoltage - minimumVoltage) / 2;
    }
}
