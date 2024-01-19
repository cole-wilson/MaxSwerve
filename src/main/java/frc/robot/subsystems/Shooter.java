package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Shooter extends SubsystemBase {
    private CANSparkMax motor_1 = new CANSparkMax(9, MotorType.kBrushless);
    private CANSparkMax motor_2 = new CANSparkMax(10, MotorType.kBrushless);

    double motor_speed = 0.5;
    boolean isrunning = false;

    public Shooter() {
        motor_2.follow(motor_1, true);

        SmartDashboard.putNumber("shooter_speed", motor_speed);
    }

    @Override
    public void periodic() {
        this.motor_speed = SmartDashboard.getNumber("shooter_speed", 0.5);
        if (isrunning)
            motor_1.set(motor_speed);
    }

    public void start() {
        isrunning = true;
        motor_1.set(motor_speed);
    }
    public void stop() {
        isrunning = false;
        motor_1.stopMotor();
    }
    
}
