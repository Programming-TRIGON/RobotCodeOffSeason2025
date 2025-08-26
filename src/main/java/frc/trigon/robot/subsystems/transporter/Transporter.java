package frc.trigon.robot.subsystems.transporter;

import com.ctre.phoenix6.controls.VoltageOut;
import frc.trigon.robot.subsystems.MotorSubsystem;
import trigon.hardware.phoenix6.talonfx.TalonFXMotor;
import trigon.hardware.phoenix6.talonfx.TalonFXSignal;

public class Transporter extends MotorSubsystem {
    private final TalonFXMotor masterMotor = TransporterConstants.MASTER_MOTOR;
    private final VoltageOut voltageRequest = new VoltageOut(0).withEnableFOC(TransporterConstants.FOC_ENABLED);
    private TransporterConstants.TransporterState targetState = TransporterConstants.TransporterState.REST;

    public Transporter() {
        setName("Transporter");
    }

    @Override
    public void updatePeriodically() {
        masterMotor.update();
    }

    @Override
    public void updateMechanism() {
        TransporterConstants.MECHANISM.update(masterMotor.getSignal(TalonFXSignal.MOTOR_VOLTAGE));
    }

    @Override
    public void stop() {
        masterMotor.stopMotor();
    }

    void setTargetState(TransporterConstants.TransporterState targetState) {
        this.targetState = targetState;
        setTargetVoltage(targetState.targetVoltage);
    }

    void setTargetVoltage(double targetVoltage) {
        masterMotor.setControl(voltageRequest.withOutput(targetVoltage));
    }
}