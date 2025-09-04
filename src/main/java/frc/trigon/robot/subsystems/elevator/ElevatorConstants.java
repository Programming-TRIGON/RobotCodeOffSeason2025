package frc.trigon.robot.subsystems.elevator;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.signals.*;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.event.BooleanEvent;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import trigon.hardware.RobotHardwareStats;
import trigon.hardware.misc.simplesensor.SimpleSensor;
import trigon.hardware.phoenix6.talonfx.TalonFXMotor;
import trigon.hardware.phoenix6.talonfx.TalonFXSignal;
import trigon.hardware.simulation.ElevatorSimulation;
import trigon.utilities.mechanisms.ElevatorMechanism2d;

import java.util.function.DoubleSupplier;

public class ElevatorConstants {
    private static final int
            MASTER_MOTOR_ID = 16,
            FOLLOWER_MOTOR_ID = 17,
            REVERSE_LIMIT_SENSOR_CHANNEL = 0;
    private static final String
            MASTER_MOTOR_NAME = "ElevatorMasterMotor",
            FOLLOWER_MOTOR_NAME = "ElevatorFollowerMotor",
            REVERSE_LIMIT_SENSOR_NAME = "ElevatorReverseLimitSensor";
    static final TalonFXMotor
            MASTER_MOTOR = new TalonFXMotor(MASTER_MOTOR_ID, MASTER_MOTOR_NAME),
            FOLLOWER_MOTOR = new TalonFXMotor(FOLLOWER_MOTOR_ID, FOLLOWER_MOTOR_NAME);
    private static final SimpleSensor REVERSE_LIMIT_SENSOR = SimpleSensor.createDigitalSensor(REVERSE_LIMIT_SENSOR_CHANNEL, REVERSE_LIMIT_SENSOR_NAME);

    private static final double GEAR_RATIO = 4;
    private static final double REVERSE_LIMIT_SENSOR_RESET_POSITION = 0;
    private static final boolean SHOULD_FOLLOWER_OPPOSE_MASTER = false;
    static final double
            DEFAULT_MAXIMUM_VELOCITY = RobotHardwareStats.isSimulation() ? 80 : 20,
            DEFAULT_MAXIMUM_ACCELERATION = RobotHardwareStats.isSimulation() ? 80 : 20;
    static final boolean FOC_ENABLED = true;

    private static final int MOTOR_AMOUNT = 2;
    private static final DCMotor GEARBOX = DCMotor.getKrakenX60(MOTOR_AMOUNT);
    private static final double
            ELEVATOR_MASS_KILOGRAMS = 7,
            DRUM_RADIUS_METERS = 0.04,
            MINIMUM_ELEVATOR_HEIGHT_METERS = 0.78,
            MAXIMUM_ELEVATOR_HEIGHT_METERS = 1.752;
    private static final boolean SHOULD_SIMULATE_GRAVITY = true;
    private static final ElevatorSimulation SIMULATION = new ElevatorSimulation(
            GEARBOX,
            GEAR_RATIO,
            ELEVATOR_MASS_KILOGRAMS,
            DRUM_RADIUS_METERS,
            MINIMUM_ELEVATOR_HEIGHT_METERS,
            MAXIMUM_ELEVATOR_HEIGHT_METERS,
            SHOULD_SIMULATE_GRAVITY
    );
    static final SysIdRoutine.Config SYSID_CONFIG = new SysIdRoutine.Config(
            Units.Volts.of(1.25).per(Units.Seconds),
            Units.Volts.of(3),
            Units.Second.of(1000)
    );

    public static final Pose3d FIRST_STAGE_VISUALIZATION_ORIGIN_POINT = new Pose3d(
            new Translation3d(0, 0, 0),
            new Rotation3d(0, 0, 0)
    );
    static final Pose3d SECOND_STAGE_VISUALIZATION_ORIGIN_POINT = new Pose3d(
            new Translation3d(0, 0 ,0),
            new Rotation3d(0, 0, 0)
    );
    static final ElevatorMechanism2d MECHANISM = new ElevatorMechanism2d(
            "ElevatorMechanism",
            MAXIMUM_ELEVATOR_HEIGHT_METERS + 0.1,
            MINIMUM_ELEVATOR_HEIGHT_METERS,
            Color.kYellow
    );

    private static final double REVERSE_LIMIT_SENSOR_DEBOUNCE_TIME_SECONDS = 0.1;
    private static final BooleanEvent REVERSE_LIMIT_SENSOR_BOOLEAN_EVENT = new BooleanEvent(
            CommandScheduler.getInstance().getActiveButtonLoop(),
            REVERSE_LIMIT_SENSOR::getBinaryValue
    ).debounce(REVERSE_LIMIT_SENSOR_DEBOUNCE_TIME_SECONDS);

    private static final DoubleSupplier REVERSE_LIMIT_SENSOR_SIMULATION_SUPPLIER = () -> 0;
    static final double FIRST_ELEVATOR_COMPONENT_EXTENDED_LENGTH_METERS = 0.78;
    static final double DRUM_DIAMETER_METERS = DRUM_RADIUS_METERS * 2;

    static {
        configureMasterMotor();
        configureFollowerMotor();
        configureReverseLimitSensor();
    }

    private static void configureMasterMotor() {
        final TalonFXConfiguration config = new TalonFXConfiguration();

        config.Audio.BeepOnBoot = false;
        config.Audio.BeepOnConfig = false;

        config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        config.Feedback.SensorToMechanismRatio = GEAR_RATIO;

        config.Slot0.kP = RobotHardwareStats.isSimulation() ? 6:0;
        config.Slot0.kI = RobotHardwareStats.isSimulation() ? 0:0;
        config.Slot0.kD = RobotHardwareStats.isSimulation() ? 0.3:0;
        config.Slot0.kS = RobotHardwareStats.isSimulation() ? 0.016527:0;
        config.Slot0.kV = RobotHardwareStats.isSimulation() ? 0.4771:0;
        config.Slot0.kA = RobotHardwareStats.isSimulation() ? 0.014197:0;
        config.Slot0.kG = RobotHardwareStats.isSimulation() ? 0.58959:0;

        config.Slot0.GravityType = GravityTypeValue.Elevator_Static;
        config.Slot0.StaticFeedforwardSign = StaticFeedforwardSignValue.UseVelocitySign;

        config.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
        config.SoftwareLimitSwitch.ReverseSoftLimitThreshold = 0;

        config.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
        config.SoftwareLimitSwitch.ForwardSoftLimitThreshold = 6.8;

        config.MotionMagic.MotionMagicCruiseVelocity = DEFAULT_MAXIMUM_VELOCITY;
        config.MotionMagic.MotionMagicAcceleration = DEFAULT_MAXIMUM_ACCELERATION;
        config.MotionMagic.MotionMagicJerk = config.MotionMagic.MotionMagicAcceleration * 10;

        MASTER_MOTOR.applyConfiguration(config);
        MASTER_MOTOR.setPhysicsSimulation(SIMULATION);

        MASTER_MOTOR.registerSignal(TalonFXSignal.POSITION, 100);
        MASTER_MOTOR.registerSignal(TalonFXSignal.VELOCITY, 100);
        MASTER_MOTOR.registerSignal(TalonFXSignal.MOTOR_VOLTAGE, 100);
        MASTER_MOTOR.registerSignal(TalonFXSignal.CLOSED_LOOP_REFERENCE, 100);
        MASTER_MOTOR.registerSignal(TalonFXSignal.STATOR_CURRENT, 100);
    }

    private static void configureFollowerMotor() {
        final TalonFXConfiguration config = new TalonFXConfiguration();

        config.Audio.BeepOnBoot = false;
        config.Audio.BeepOnConfig = false;

        config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

        FOLLOWER_MOTOR.applyConfiguration(config);

        final Follower followerRequest = new Follower(MASTER_MOTOR.getID(), SHOULD_FOLLOWER_OPPOSE_MASTER);
        FOLLOWER_MOTOR.setControl(followerRequest);
    }

    private static void configureReverseLimitSensor() {
        REVERSE_LIMIT_SENSOR.setSimulationSupplier(REVERSE_LIMIT_SENSOR_SIMULATION_SUPPLIER);
        REVERSE_LIMIT_SENSOR_BOOLEAN_EVENT.ifHigh(() -> MASTER_MOTOR.setPosition(REVERSE_LIMIT_SENSOR_RESET_POSITION));
    }

    public enum ElevatorState {
        REST(0, 0.7),
        SCORE_L1(0, 1),
        SCORE_L2(0, 1),
        SCORE_L3(0.4, 1),
        SCORE_L4(1.045, 1),
        COLLECT_ALGAE_FROM_L3(0.35, 1),
        REST_WITH_ALGAE(0, 0.3),
        SCORE_NET(1.045, 0.3);

        public final double targetPositionMeters;
        final double speedScalar;

        ElevatorState(double targetPositionMeters, double speedScalar) {
            this.targetPositionMeters = targetPositionMeters;
            this.speedScalar = speedScalar;
        }
    }
}
