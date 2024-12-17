package org.firstinspires.ftc.teamcode.swerveftclibtesting.examples;

import com.arcrobotics.ftclib.command.SubsystemBase;
import org.firstinspires.ftc.teamcode.swerveftclibtesting.Swerve.SwerveDrive;

public class ExampleSwerveSubsystem extends SubsystemBase {
    private final SwerveDrive m_swerveDrive;

    public ExampleSwerveSubsystem(SwerveDrive drive) {
        this.m_swerveDrive = drive;
    }

    @Override
    public void periodic() {
        this.m_swerveDrive.update();
    }

    public SwerveDrive getDrive() {
        return this.m_swerveDrive;
    }
}
