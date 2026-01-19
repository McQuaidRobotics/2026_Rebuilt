# FRC 2026 Robot Code ("2026_Rebuilt")

## Project Overview

This is the Java codebase for the FRC 2026 robot, developed by the **IgKnighters** team. It utilizes the WPILib framework (2026 beta) and Gradle for build management. The project features a custom subsystem architecture, advanced swerve drive control, and integrated simulation support.

**Key Technologies:**
*   **Language:** Java 17
*   **Framework:** WPILib 2026 (Beta), Command-Based
*   **Build System:** Gradle
*   **Path Planning:** ChoreoLib
*   **Logging:** DogLog, Monologue
*   **Simulation:** Standard WPILib Simulation

## Reference Implementation

The **2025_Reefscape** project (located at `../2025_Reefscape`) serves as the primary example for project layout, architectural patterns, and coding standards. When implementing new features or refactoring, refer to the 2025 codebase for guidance on:
*   **Subsystem Structure:** How to organize `ExclusiveSubsystem` and `SharedSubsystem`.
*   **Logging:** Proper integration of Monologue and DogLog.
*   **Command Usage:** Patterns for auto routines and teleop commands.
*   **Note:** While the 2025 codebase uses the "Sham" library for simulation, **this project (2026) exclusively uses standard WPILib simulation classes.** Do not copy "Sham" patterns.

## Architecture

The codebase follows a command-based architecture with some custom extensions for resource management and simulation.

### Entry Point
*   `src/java/igknighters/Main.java`: Standard entry point.
*   `src/java/igknighters/Robot.java`: The main robot class (TimedRobot), responsible for initializing subsystems, handling modes (Teleop, Auto, Test), and the main control loop.

### Subsystems (`src/java/igknighters/subsystems`)
Subsystems are centralized in `Subsystems.java`. The project distinguishes between:
*   **ExclusiveSubsystem:** Standard WPILib subsystems (require command requirements, e.g., Swerve Drive, LED).
*   **SharedSubsystem:** "Lockless" resources that can be accessed by multiple commands/loops simultaneously (e.g., Vision, Luma).

### Simulation
The project uses standard WPILib simulation support (e.g., `PhysicsSim`, `DCMotorSim`, `RoboRioSim`) instead of custom hardware abstraction layers. Simulation logic should be embedded within subsystems or handled via parallel simulation classes that mirror the hardware implementation using WPILib's `RobotBase.isSimulation()` check.

## Building and Running

The project uses the Gradle wrapper (`gradlew`).

### Common Commands
*   **Build:** Compile the code.
    ```bash
    ./gradlew build
    ```
*   **Deploy:** Build and deploy code to the RoboRIO.
    ```bash
    ./gradlew deploy
    ```
*   **Format Code:** Apply Spotless formatting (Required for CI).
    ```bash
    ./gradlew spotlessApply
    ```
*   **Run Tests:** Execute unit tests.
    ```bash
    ./gradlew test
    ```
*   **Simulation:** Run the robot code in desktop simulation.
    ```bash
    ./gradlew simulateJava
    ```

## Key Directories

*   `src/java/igknighters/`: Main source code.
    *   `commands/`: WPILib Commands (actions).
    *   `subsystems/`: Hardware interface classes.
    *   `monologue/`: Logging framework integration.
    *   `wayfinder/`: Path following and trajectory logic.
    *   `controllers/`: Driver input handling (gamepads).
*   `src/main/deploy/`: Files deployed to the robot (e.g., Choreo `.traj` paths).
*   `vendordeps/`: JSON files for 3rd party libraries (CTRE, Rev, etc.).

## Development Conventions

*   **Formatting:** The project enforces strict code formatting using **Spotless** (Google Java Format + AOSP style). Always run `./gradlew spotlessApply` before committing.
*   **Tuning:** Use `TunableValues` (e.g., `TunableDouble`) for constants that may need adjustment via the dashboard at runtime.
*   **Logging:** Use `DogLog` for high-frequency data logging and `Monologue` for structure-based logging.
