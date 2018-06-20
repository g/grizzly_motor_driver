^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package grizzly_motor_driver
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.0.2 (2018-06-20)
------------------
* [grizzly_motor_driver] Installed library and node.
* Contributors: Tony Baltovski

0.0.1 (2018-06-06)
------------------
* Ignored linter error.
* Removed tests and fixed linter errors.
* Added checking for the TPM being enabled during configuring and reduced logging.
* Added ability to detect disconnects. Slowed down status requests and cleaned-up logging.
* Added motor temperature sensing.
* Fixed motor voltage and current readings.
* Added new parameters for TPM400s.
* Added check runtime errors to state machine.  Added stopping state to aid with estop handling.
* Added action to receiving a runttime fault. Removed checking of startup errors durring run.
* Added Estop reset handling.
* Removed diagnostic updater
* lint changes
* Added gear ratio. Moved where velocity is published.
* Moved diagnostic updater.
* Added new registers and switched to sending motion commands instead of heading.  Updated feedback publishing
* Change status updates.
* Changes to allow multiple drivers. Request status in sequence.
* Changed mutex strategy (std::atomic). Needed to change main to acomodate driver no longer being copyable.
* Remove always switching, reduce neutral to stop time, added getOutputVoltage to driver.
* Added mutex, main now is only thread sending commands.
* Create temp sub for experimenting with speeds.
* [diagnostics] Changes to match ddbc78b
* [diagnostics] Split temperature and power diagnostic info.
* Added diagnostic object to main.
* Start of diagnostics
* Added more status feedback.
* More work to control via ROS.
* Updated accel/decel scaling.
* Added messages.
* Basics of configuring TPM.
* More initial work.
* General initial layout.
* Contributors: Michael Hosmar, Tony Baltovski
