// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package cowlib;

import com.ctre.phoenix6.hardware.CANcoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.DriveConstants.SwervePID;

/** Add your docs here. */
public class SwerveModule {
	private CANSparkMax angleMotor;
	private CANSparkMax speedMotor;
	private PIDController pidController;
	private CANcoder encoder;
	private boolean inverted;
	private double offset;

	public SwerveModule(int angleMotor, int speedMotor, int encoder, boolean drive_inverted, double offset) {
		this.angleMotor = new CANSparkMax(angleMotor, MotorType.kBrushless);
		this.speedMotor = new CANSparkMax(speedMotor, MotorType.kBrushless);
		this.pidController = new PIDController(SwervePID.p, SwervePID.i, SwervePID.d);
		this.encoder = new CANcoder(encoder);
		this.inverted = drive_inverted;
		this.offset = offset;

		this.pidController.enableContinuousInput(-180, 180);
	}
	
	public SwerveModule(SwerveModuleConfig config, double maxVelocity, double maxVoltage) {
		this(config.angleMotorId, config.driveMotorId, config.encoderId, config.drive_inverted, config.offset);
		angleMotor.setSmartCurrentLimit(DriveConstants.currentLimit);
		speedMotor.setSmartCurrentLimit(DriveConstants.currentLimit);
	}

	public void drive(double speed, double angle) {
		speedMotor.setVoltage(speed * (this.inverted ? -1 : 1));
		angleMotor.setVoltage(-pidController.calculate(this.getEncoder(), angle));
		// angleMotor.setVoltage(-pidController.calculate(clamp(this.getEncoder() + offset), angle));
	}

	public void drive(SwerveModuleState state) {
		this.drive(state.speedMetersPerSecond, state.angle.getDegrees());
	}

	public double getEncoder() {
		return encoder.getAbsolutePosition().getValueAsDouble();
	}

	public double getEncoderRadians() {
		return encoder.getAbsolutePosition().getValueAsDouble()*2*Math.PI/360.0;
	}

	public double getRelativeEncoder() {
		return clamp(getEncoder() + offset);
	}

	// public void resetEncoder() {
	// 	offset = -getEncoder();
	// 	// encoder.configMagnetOffset(offset);
	// }

	public double clamp(double n) {
		if(n > 180) {
			return n - 360;
		}
		else if(n < -180) {
			return 360 + n;
		}
		return n;
	}
}
