# FRC 2713 2022 Robot Code

### Gradle Commands

Some important gradle commands:

* `gradlew spotlessApply` to run code formatting
* `gradlew deploy` to both build and deploy code to the rio
* `gradlew build` to build robot code (but **does not deploy**)

Some other useful gradle commands:

* `gradlew tasks` to view all available tasks
* `gradlew clean` to remove build cache (can be used to troubleshoot odd build errors)
* `gradlew Glass` to run Glass (WPILib graphing tool)
* `gradlew PathWeaver` to run PathWeaver (WPILib path gen tool)
* `gradlew ShuffleBoard` to run ShuffleBoard (WPILib Dashboard tool)
* `gradlew SysId` to run SysID (WPILib characterization tool)

### Docs


* WPILib
    * General docs: https://docs.wpilib.org/en/stable/
    * JavaDocs: https://first.wpi.edu/wpilib/allwpilib/docs/release/java/index.html
    * WPILib releases: https://github.com/wpilibsuite/allwpilib/releases
* REV
    * REVLib JavaDocs: https://codedocs.revrobotics.com/java/com/revrobotics/package-summary.html
    * REVLib releases: https://github.com/REVrobotics/REV-Software-Binaries/releases
    * Spark MAX general docs: https://docs.revrobotics.com/sparkmax/
