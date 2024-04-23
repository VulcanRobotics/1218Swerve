# 1218Swerve
Team 1218's common swerve repository:
* Based on the initial 3.0.0 release of Team 6328's AdvantageKit - AdvancedSwerveDriveProject code, distributed here:
    * https://github.com/Mechanical-Advantage/AdvantageKit/releases

This code base provides an ideal starting point for a robot using a swerve drive and photon vision:
* Swerve drive using TalonFX motors (via CTRE's Phoenix libraries)
* Full integration of AdvantageKit for input-output logging and simulaton
* Highly accurate swerve drive odometry, logged and visualized using AdvantageKit/Scope
* Full support for PathPlanner, for accurate and reliable auton modes
* Use of Shuffleboard for drive station instrumentation (no more SmartDashboard!)
* Well-tuned vision system that uses WPILib's SwerveDrivePoseEstimator and photonvision to give highly accurate pose estimations.
