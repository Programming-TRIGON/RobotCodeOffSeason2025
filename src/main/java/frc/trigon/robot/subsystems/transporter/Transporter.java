package frc.trigon.robot.subsystems.transporter;

import com.ctre.phoenix6.controls.VoltageOut;
import frc.trigon.robot.subsystems.MotorSubsystem;
import trigon.hardware.phoenix6.talonfx.TalonFXMotor;
import trigon.hardware.phoenix6.talonfx.TalonFXSignal;

public class Transporter extends MotorSubsystem {
    private final TalonFXMotor
            rightMotor = TransporterConstants.RIGHT_MOTOR,
            leftMotor = TransporterConstants.LEFT_MOTOR;
    private final VoltageOut voltageRequest = new VoltageOut(0).withEnableFOC(TransporterConstants.FOC_ENABLED);
    private TransporterConstants.TransporterState targetState = TransporterConstants.TransporterState.REST;

    public Transporter() {
        setName("Transporter");
    }

    @Override
    public void updatePeriodically() {
        rightMotor.update();
        leftMotor.update();
        TransporterConstants.BEAM_BREAK.updateSensor();
    }

    @Override
    public void updateMechanism() {
        TransporterConstants.RIGHT_MOTOR_MECHANSIM.update(rightMotor.getSignal(TalonFXSignal.MOTOR_VOLTAGE));
        TransporterConstants.LEFT_MOTOR_MECHANSIM.update(leftMotor.getSignal(TalonFXSignal.MOTOR_VOLTAGE));
    }

    @Override
    public void stop() {
        rightMotor.stopMotor();
        leftMotor.stopMotor();
        TransporterConstants.RIGHT_MOTOR_MECHANSIM.setTargetVelocity(0);
        TransporterConstants.LEFT_MOTOR_MECHANSIM.setTargetVelocity(0);
    }

    public TransporterConstants.TransporterState getTargetState() {
        return targetState;
    }

    public boolean hasCoral() {
        return TransporterConstants.HAS_CORAL_BOOLEAN_EVENT.getAsBoolean();
    }

    void setTargetState(TransporterConstants.TransporterState targetState) {
        this.targetState = targetState;
        setTargetVoltage(targetState.targetVoltage, targetState.targetVoltage * targetState.voltageScalar);
    }

    void setTargetVoltage(double rightMotorVoltage, double leftMotorVoltage) {
        setTargetRightMotorVoltage(rightMotorVoltage);
        setTargetLeftMotorVoltage(leftMotorVoltage);
    }

    private void setTargetRightMotorVoltage(double targetVoltage) {
        TransporterConstants.RIGHT_MOTOR_MECHANSIM.setTargetVelocity(targetVoltage);
        rightMotor.setControl(voltageRequest.withOutput(targetVoltage));
    }

    private void setTargetLeftMotorVoltage(double targetVoltage) {
        TransporterConstants.LEFT_MOTOR_MECHANSIM.setTargetVelocity(targetVoltage);
        leftMotor.setControl(voltageRequest.withOutput(targetVoltage));
    }
}