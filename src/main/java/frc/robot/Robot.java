// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import org.ironmaple.simulation.SimulatedArena;

import com.ctre.phoenix6.SignalLogger;

// import org.ironmaple.simulation.SimulatedArena;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Simulation.Constants;
import frc.robot.subsystems.Simulation.ElevatorSimulation;
import frc.robot.subsystems.Simulation.MapleSim;

public class Robot extends TimedRobot {
  private Command m_autonomousCommand;

  private final RobotContainer m_robotContainer;
  final ElevatorSimulation m_elevator = new ElevatorSimulation();

  public Robot() {
    m_robotContainer = new RobotContainer();

  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
    m_elevator.updateTelemetry();
  }

  @Override
  public void disabledInit() {
    // This just makes sure that our simulation code knows that the motor's off.
    m_elevator.stop();
  }

  @Override
  public void disabledPeriodic() {
  }

  @Override
  public void disabledExit() {
  }

  @Override
  public void autonomousInit() {
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }

  @Override
  public void autonomousPeriodic() {
  }

  @Override
  public void autonomousExit() {
    // m_robotContainer.io.chassis.adjustRotation();
  }

  @Override
  public void teleopInit() {
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
  }

  @Override
  public void teleopPeriodic() {
    if (true) {
      // Here, we set the constant setpoint of 0.75 meters.
      m_elevator.reachGoal(Constants.kSetpointMeters);
    } else {
      // Otherwise, we update the setpoint to 0.
      m_elevator.reachGoal(0.0);
    }
  }

  @Override
  public void teleopExit() {
  }

  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
  }

  @Override
  public void testPeriodic() {
  }

  @Override
  public void testExit() {
  }

  @Override
  public void simulationPeriodic() {
    // Update the simulation model.
    m_elevator.simulationPeriodic();
    MapleSim.Arena().getInstance().simulationPeriodic();
    // Logger.recordOutput("FieldSimulation/Algae",
    //     SimulatedArena.getInstance().getGamePiecesArrayByType("Algae"));
    // Logger.recordOutput("FieldSimulation/Coral",
    //     SimulatedArena.getInstance().getGamePiecesArrayByType("Coral"));

  }

  @Override
  public void close() {
    m_elevator.close();
    super.close();
  }
}
