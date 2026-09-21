Place your .auto files here (created in PathPlanner desktop app).
Required auto names referenced in RobotContainer.java:
  - BlueDepot.auto
  - BlueOutpost.auto

Named commands available for events on paths:
  - HomeIntake   : Home the intake at start of auto
  - IntakeDeployOnly: Home if needed, then tilt intake down (no roller spin)
  - IntakeBalls  : Spin intake rollers only (stall-protected, timed)
  - IntakeFuel   : Deploy intake, spin rollers with stall detection
  - AutoShoot    : Align to HUB via vision + shoot (6s timeout)

Chooser options also include:
  - Do Nothing
  - Calibrate CANcoders

Add more autos by updating RobotAutoCatalog.competitionPathPlannerAutos().
