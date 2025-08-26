package frc.trigon.robot.subsystems.transporter;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.system.plant.DCMotor;
import trigon.hardware.misc.simplesensor.SimpleSensor;
import trigon.hardware.phoenix6.talonfx.TalonFXMotor;
import trigon.hardware.phoenix6.talonfx.TalonFXSignal;
import trigon.hardware.simulation.SimpleMotorSimulation;
import trigon.utilities.mechanisms.SpeedMechanism2d;

import java.util.function.DoubleSupplier;

public class TransporterConstants {
    private static final int
            MASTER_MOTOR_ID = 11,
            FOLLOWER_MOTOR_ID = 12,
            BEAM_BREAK_CHANNEL = 3;
    private static final String
            MASTER_MOTOR_NAME = "TransporterMasterMotor",
            FOLLOWER_MOTOR_NAME = "TransporterFollowerMotor",
            BEAM_BREAK_NAME = "TransporterBeamBreak";
    static final TalonFXMotor
            MASTER_MOTOR = new TalonFXMotor(MASTER_MOTOR_ID, MASTER_MOTOR_NAME),
            FOLLOWER_MOTOR = new TalonFXMotor(FOLLOWER_MOTOR_ID, FOLLOWER_MOTOR_NAME);
    static final SimpleSensor BEAM_BREAK = SimpleSensor.createAnalogSensor(BEAM_BREAK_CHANNEL, BEAM_BREAK_NAME);

    private static double GEAR_RATIO = 3;
    private static final boolean SHOULD_FOLLOWER_OPPOSE_MASTER = true;
    static boolean FOC_ENABLED = true;

    private static int MOTOR_AMOUNT = 2;
    private static DCMotor GEARBOX = DCMotor.getKrakenX60(MOTOR_AMOUNT);
    private static final double MOMENT_OF_INERTIA = 0.003;
    private static final SimpleMotorSimulation SIMULATION = new SimpleMotorSimulation(
            GEARBOX,
            GEAR_RATIO,
            MOMENT_OF_INERTIA
    );
    private static final DoubleSupplier BEAM_BREAK_SIMULATION_SUPPLIER = () -> 0; //TODO: implement

    private static final double MAXIMUM_DISPLAYABLE_VELOCITY = 12;
    static final SpeedMechanism2d MECHANISM = new SpeedMechanism2d(
            "TranporterMechanism",
            MAXIMUM_DISPLAYABLE_VELOCITY
    );

    static {
        configureMasterMotor();
        configureFollowerMotor();
        configureBeamBreak();
    }

    private static void configureMasterMotor() {
        final TalonFXConfiguration config = new TalonFXConfiguration();

        config.Audio.BeepOnBoot = false;
        config.Audio.BeepOnConfig = false;

        config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        config.Feedback.SensorToMechanismRatio = GEAR_RATIO;

        MASTER_MOTOR.applyConfiguration(config);
        MASTER_MOTOR.setPhysicsSimulation(SIMULATION);

        MASTER_MOTOR.registerSignal(TalonFXSignal.MOTOR_VOLTAGE, 100);
        MASTER_MOTOR.registerSignal(TalonFXSignal.STATOR_CURRENT, 100);
    }

    private static void configureFollowerMotor() {
        final TalonFXConfiguration config = new TalonFXConfiguration();

        config.Audio.BeepOnBoot = false;
        config.Audio.BeepOnConfig = false;

        config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

        FOLLOWER_MOTOR.applyConfiguration(config);

        final Follower followerRequest = new Follower(MASTER_MOTOR.getID(), SHOULD_FOLLOWER_OPPOSE_MASTER);
        FOLLOWER_MOTOR.setControl(followerRequest);
    }


    private static void configureBeamBreak() {
        BEAM_BREAK.setSimulationSupplier(BEAM_BREAK_SIMULATION_SUPPLIER);
    }

    public enum TransporterState {
        REST(0),
        COLLECT(5),
        EJECT(-5);

        public double targetVoltage;

        TransporterState(double targetVoltage) {
            this.targetVoltage = targetVoltage;
        }
    }
}