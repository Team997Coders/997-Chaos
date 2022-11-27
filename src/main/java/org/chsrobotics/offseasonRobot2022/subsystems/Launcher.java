/**
Copyright 2022 FRC Team 997

This program is free software: 
you can redistribute it and/or modify it under the terms of the 
GNU General Public License as published by the Free Software Foundation, 
either version 3 of the License, or (at your option) any later version.

This program is distributed in the hope that it will be useful, 
but WITHOUT ANY WARRANTY; without even the implied warranty of 
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. 
See the GNU General Public License for more details.

You should have received a copy of the GNU General Public License along with this program. 
If not, see <https://www.gnu.org/licenses/>.
*/
package org.chsrobotics.offseasonRobot2022.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.chsrobotics.lib.telemetry.Logger;
import org.chsrobotics.offseasonRobot2022.Constants.SubsystemConstants.LauncherConstants;
import org.chsrobotics.offseasonRobot2022.Robot;

/** */
public class Launcher extends SubsystemBase {
    private final CANSparkMax forwardFlywheelMotor =
            new CANSparkMax(LauncherConstants.FORWARD_FLYWHEEL_MOTOR_CANID, MotorType.kBrushless);

    private final CANSparkMax aftFlywheelMotor =
            new CANSparkMax(LauncherConstants.AFT_FLYWHEEL_MOTOR_CANID, MotorType.kBrushless);

    private final LinearFilter flywheelVelocityFilter =
            LinearFilter.movingAverage(
                    LauncherConstants.FLYWHEEL_ENCODER_VELOCITY_SMOOTHING_SAMPLES);

    private final TalonSRX hoodMotor = new TalonSRX(LauncherConstants.HOOD_MOTOR_CANID);

    private final DigitalInput hoodLimitSwitch =
            new DigitalInput(LauncherConstants.HOOD_LIMIT_SWITCH_DIO);

    private final LinearFilter hoodVelocityFilter =
            LinearFilter.movingAverage(LauncherConstants.HOOD_ENCODER_VELOCITY_SMOOTHING_SAMPLES);

    private final RelativeEncoder flywheelExternalEncoder;

    private final SingleJointedArmSim hoodSim =
            new SingleJointedArmSim(
                    DCMotor.getBag(1),
                    (1 / LauncherConstants.MOTOR_TO_HOOD.toDoubleRatioInputToOutput()),
                    LauncherConstants.HOOD_MOMENT,
                    LauncherConstants.HOOD_RADIUS_METERS,
                    0,
                    LauncherConstants.HOOD_MAX_ANGLE_RADIANS,
                    LauncherConstants.HOOD_MASS_KG,
                    true,
                    VecBuilder.fill(LauncherConstants.HOOD_ENCODER_ERROR_RADIANS_STDDEV));

    private final FlywheelSim flywheelSim =
            new FlywheelSim(
                    DCMotor.getNEO(2),
                    1 / LauncherConstants.MOTOR_TO_FLYWHEEL.toDoubleRatioInputToOutput(),
                    LauncherConstants.FLYWHEEL_MOMENT,
                    VecBuilder.fill(LauncherConstants.FLYWHEEL_ENCODER_ERROR_RADIANS_STDDEV));

    private double hoodAbsDistanceRadians = 0;
    private double flywheelAbsDisanceRadians = 0;

    private double previousHoodAngleRadians = 0;
    private double previousFlywheelVelocityRadiansPerSecond = 0;

    private double approximateFlywheelAcceleration = 0;

    private final String subdirString = "launcher";

    private final Logger<Double> hoodAngleRadiansLogger =
            new Logger<>("hoodAngleRadians", subdirString);

    private final Logger<Boolean> hoodLimitSwitchPressedLogger =
            new Logger<>("hoodLimitSwitchPressed", subdirString);

    private final Logger<Double> flywheelVelocityRadsPerSecondLogger =
            new Logger<>("flywheelVelocityRadsPerSecond", subdirString);

    private final Logger<Double> hoodAppliedVoltageLogger =
            new Logger<>("hoodAppliedVoltage", subdirString);

    private final Logger<Double> flywheelAppliedVoltageLogger =
            new Logger<>("flywheelAppliedVoltage", subdirString);

    private final Logger<Double> flywheelForwardMotorCurrentLogger =
            new Logger<>("flywheelForwardMotorCurrentAmps", subdirString);
    private final Logger<Double> flywheelAftMotorCurrentLogger =
            new Logger<>("flywheelAftMotorCurrentAmps", subdirString);

    private final Logger<Double> hoodMotorCurrentLogger =
            new Logger<>("hoodMotorCurrentAmps", subdirString);

    private final Logger<Double> flywheelForwardMotorTempCLogger =
            new Logger<>("flywheelForwardMotorTempC", subdirString);
    private final Logger<Double> flywheelAftMotorTempCLogger =
            new Logger<>("flywheelAftMotorTempC", subdirString);

    /** */
    public Launcher() {
        forwardFlywheelMotor.setInverted(LauncherConstants.FORWARD_FLYWHEEL_MOTOR_INVERTED);
        aftFlywheelMotor.setInverted(LauncherConstants.AFT_FLYWHEEL_MOTOR_INVERTED);

        hoodMotor.setInverted(LauncherConstants.HOOD_MOTOR_INVERTED);

        hoodMotor.configSelectedFeedbackSensor(LauncherConstants.HOOD_FEEDBACK_DEVICE);
        hoodMotor.setSensorPhase(LauncherConstants.HOOD_ENCODER_INVERTED);

        flywheelExternalEncoder =
                forwardFlywheelMotor.getAlternateEncoder(
                        LauncherConstants.FLYWHEEL_ENCODER_TYPE,
                        LauncherConstants.FLYWHEEL_ENCODER_CPR);
        flywheelExternalEncoder.setInverted(LauncherConstants.FLYWHEEL_ENCODER_INVERTED);
    }

    /**
     * @return
     */
    public boolean hoodLimitSwitchActivated() {
        if (LauncherConstants.HOOD_LIMIT_SWITCH_TRUE_WHEN_ACTIVATED) return hoodLimitSwitch.get();
        else return !hoodLimitSwitch.get();
    }

    /**
     * @param voltage
     */
    public void setFlywheelVoltage(double voltage) {
        flywheelAppliedVoltageLogger.update(voltage);
        forwardFlywheelMotor.setVoltage(voltage);
        aftFlywheelMotor.setVoltage(voltage);

        flywheelSim.setInputVoltage(voltage);
    }

    /** */
    public void setHoodVoltage(double voltage) {
        double toApply;

        if (getHoodPositionRadians() >= LauncherConstants.HOOD_MAX_ANGLE_RADIANS && voltage > 0) {
            toApply = 0; // if at or above max angle, cancel all + (up) voltages
        } else if (Robot.isReal()) {
            if (hoodLimitSwitchActivated() && voltage < 0) {
                toApply = 0;
            } else {
                toApply = voltage;
            } // if limit switch active (or at lower limit in sim), cancel all - (down) voltages
        } else {
            if (getHoodPositionRadians() <= 0 && voltage < 0) {
                toApply = 0;
            } else {
                toApply = voltage;
            }
        }

        hoodSim.setInputVoltage(toApply);
        hoodMotor.set(ControlMode.PercentOutput, toApply / RobotController.getBatteryVoltage());
        hoodAppliedVoltageLogger.update(toApply);
    }

    /**
     * @return
     */
    public double getHoodPositionRadians() {
        if (Robot.isReal()) {
            return LauncherConstants.MOTOR_TO_HOOD.outputFromInput(
                    hoodMotor.getSelectedSensorPosition() / LauncherConstants.HOOD_ENCODER_CPR);
        } else {
            return hoodSim.getAngleRads();
        }
    }

    /**
     * @return
     */
    public double getHoodVelocityRadiansPerSecond() {
        if (Robot.isReal()) {
            return hoodVelocityFilter.calculate(
                    LauncherConstants.MOTOR_TO_HOOD.outputFromInput(
                            (hoodMotor.getSelectedSensorVelocity()
                                            / LauncherConstants.HOOD_ENCODER_CPR)
                                    * 10));
        } else {
            return hoodSim.getVelocityRadPerSec();
        }
    }

    /**
     * @return
     */
    public double getFlywheelVelocityRadiansPerSecond() {
        if (Robot.isReal()) {
            return flywheelVelocityFilter.calculate(
                    LauncherConstants.MOTOR_TO_FLYWHEEL.outputFromInput(
                            Units.rotationsPerMinuteToRadiansPerSecond(
                                    flywheelExternalEncoder.getVelocity())));
        } else {
            return flywheelSim.getAngularVelocityRadPerSec();
        }
    }

    /**
     * @return
     */
    public double getFlywheelAccelerationRadiansPerSecondSquared() {
        return approximateFlywheelAcceleration;
    }

    /**
     * @return
     */
    public double getTotalFlywheelMotorCurrent() {
        return Robot.getCurrentAmps(LauncherConstants.FORWARD_FLYWHEEL_MOTOR_PDP_CHANNEL)
                + Robot.getCurrentAmps(LauncherConstants.AFT_FLYWHEEL_MOTOR_PDP_CHANNEL);
    }

    /**
     * @return
     */
    public double getHoodAbsDistanceRadians() {
        return hoodAbsDistanceRadians;
    }

    /**
     * @return
     */
    public double getFlywheelAbsDistanceRadians() {
        return flywheelAbsDisanceRadians;
    }

    @Override
    public void periodic() {
        hoodAngleRadiansLogger.update(getHoodPositionRadians());

        hoodLimitSwitchPressedLogger.update(hoodLimitSwitchActivated());

        flywheelVelocityRadsPerSecondLogger.update(getFlywheelVelocityRadiansPerSecond());

        flywheelForwardMotorCurrentLogger.update(
                Robot.getCurrentAmps(LauncherConstants.FORWARD_FLYWHEEL_MOTOR_PDP_CHANNEL));
        flywheelAftMotorCurrentLogger.update(
                Robot.getCurrentAmps(LauncherConstants.AFT_FLYWHEEL_MOTOR_PDP_CHANNEL));

        hoodMotorCurrentLogger.update(
                Robot.getCurrentAmps(LauncherConstants.HOOD_MOTOR_PDP_CHANNEL));

        flywheelForwardMotorTempCLogger.update(forwardFlywheelMotor.getMotorTemperature());
        flywheelAftMotorTempCLogger.update(aftFlywheelMotor.getMotorTemperature());

        hoodAbsDistanceRadians += Math.abs(getHoodPositionRadians() - previousHoodAngleRadians);
        flywheelAbsDisanceRadians += Math.abs(getFlywheelVelocityRadiansPerSecond() * 0.02);

        approximateFlywheelAcceleration =
                (getFlywheelVelocityRadiansPerSecond() - previousFlywheelVelocityRadiansPerSecond)
                        / 0.02;

        previousHoodAngleRadians = getHoodPositionRadians();
        previousFlywheelVelocityRadiansPerSecond = getFlywheelVelocityRadiansPerSecond();
    }

    @Override
    public void simulationPeriodic() {
        hoodSim.update(0.02);
        flywheelSim.update(0.02);
    }
}
