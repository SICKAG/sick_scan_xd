============================
ROS 2 Lifecycle Node Support
============================

Overview
========

The sick_scan_xd driver provides optional ROS 2 Lifecycle Node support for advanced state management and controlled initialization/shutdown sequences. This feature is particularly useful for:

* **System Integration**: Coordinate scanner startup with other lifecycle-managed nodes
* **Fault Tolerance**: Implement graceful error recovery and state transitions
* **Deterministic Behavior**: Control exactly when the scanner starts and stops publishing data

.. important::
   Lifecycle support is **optional** and **opt-in**. The driver defaults to standard ROS 2 node behavior for backward compatibility.

----

Quick Start
===========

Standard Mode (Default)
-----------------------

.. code-block:: bash

   # Standard autostart mode - scanner starts immediately
   ros2 launch sick_scan_xd sick_tim_7xx.launch.py

The scanner automatically initializes, connects, and starts publishing data.

Lifecycle Mode
--------------

.. code-block:: bash

   # Lifecycle mode - scanner waits for state transitions
   ros2 launch sick_scan_xd sick_tim_7xx.launch.py lifecycle_managed_node:=true

The scanner starts in the **Unconfigured** state and waits for lifecycle commands.

Transition to Active State
~~~~~~~~~~~~~~~~~~~~~~~~~~~

.. code-block:: bash

   # Configure the node (load parameters, establish connection)
   ros2 lifecycle set /sick_scan configure

   # Activate the node (start data acquisition and publishing)
   ros2 lifecycle set /sick_scan activate

----

Understanding Lifecycle States
===============================

The ROS 2 Lifecycle Node follows a standardized state machine with four primary states.

.. note::
   For the official ROS 2 lifecycle state machine diagram and detailed explanation, see:
   
   * `ROS 2 Lifecycle Design Document <https://design.ros2.org/articles/node_lifecycle.html>`_
   * `ROS 2 Lifecycle Tutorial <https://docs.ros.org/en/rolling/Tutorials/Intermediate/Lifecycle.html>`_

State Diagram
-------------

::

    ┌─────────────────┐
    │  Unconfigured   │  ← Initial state, no resources allocated
    └────────┬────────┘
             │ configure
             ▼
    ┌─────────────────┐
    │    Inactive     │  ← Parameters loaded, ready for activation
    └────────┬────────┘
             │ activate
             ▼
    ┌─────────────────┐
    │     Active      │  ← Scanner running, data being published
    └────────┬────────┘
             │ deactivate
             ▼
    ┌─────────────────┐
    │    Inactive     │  ← Publishing stopped, connection maintained
    └────────┬────────┘
             │ cleanup
             ▼
    ┌─────────────────┐
    │  Unconfigured   │  ← Resources released, ready to reconfigure
    └─────────────────┘

    Additional transitions:
    - shutdown: From any state → Finalized (emergency stop)
    - error handling: Automatic transition to appropriate error state

State Descriptions
------------------

.. list-table::
   :header-rows: 1
   :widths: 20 40 20 20

   * - State
     - Description
     - Resources
     - Publishing
   * - **Unconfigured**
     - Initial state, no initialization
     - None
     - No
   * - **Inactive**
     - Configured but not active
     - Parameters loaded, ready for connection
     - No
   * - **Active**
     - Fully operational
     - All resources active
     - Yes
   * - **Finalized**
     - Shutdown complete
     - All released
     - No

----

Enabling Lifecycle Mode
========================

Command Line Method (Recommended)
----------------------------------

The simplest way is to pass the argument when launching:

.. code-block:: bash

   ros2 launch sick_scan_xd sick_tim_7xx.launch.py lifecycle_managed_node:=true

This works with **any existing launch file** without modification.

Examples with Different Scanners
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

.. code-block:: bash

   # TiM 7xx
   ros2 launch sick_scan_xd sick_tim_7xx.launch.py lifecycle_managed_node:=true

   # LMS 1xx
   ros2 launch sick_scan_xd sick_lms_1xx.launch.py lifecycle_managed_node:=true

   # multiScan100
   ros2 launch sick_scan_xd sick_multiscan.launch.py lifecycle_managed_node:=true

   # picoScan100
   ros2 launch sick_scan_xd sick_picoscan.launch.py lifecycle_managed_node:=true

----

State Transitions
=================

Configure (Unconfigured → Inactive)
------------------------------------

**What happens:**

* Parameters are synchronized from the lifecycle node to the internal driver
* Scanner configuration is validated
* Internal node is prepared for hardware communication
* **Note**: Hardware connection (TCP/UDP) is established during the next step (activate), not during configure

**Command:**

.. code-block:: bash

   ros2 lifecycle set /sick_scan configure

Activate (Inactive → Active)
-----------------------------

**What happens:**

* Hardware connection established
* Scanner initialization runs
* Data acquisition starts
* Publishers become active

**Command:**

.. code-block:: bash

   ros2 lifecycle set /sick_scan activate

**Verify:**

.. code-block:: bash

   ros2 topic echo /scan

Deactivate (Active → Inactive)
-------------------------------

**What happens:**

* Publishing stops
* Connection remains active

**Command:**

.. code-block:: bash

   ros2 lifecycle set /sick_scan deactivate

Cleanup (Inactive → Unconfigured)
----------------------------------

**What happens:**

* Scanner threads are stopped gracefully
* Hardware connection (TCP/UDP) is closed
* All resources are released
* Node returns to initial unconfigured state

.. important::
   **Enhanced Shutdown for multiScan/picoScan**: This transition includes a critical fix for thread deadlock that occurred in earlier versions. The cleanup sequence now properly signals background threads (MsgPack exporter) to stop before attempting to join them, preventing indefinite hangs during shutdown. This fix is particularly important for multiScan100 and picoScan100 devices.

**Command:**

.. code-block:: bash

   ros2 lifecycle set /sick_scan cleanup

----

Supported Devices
=================

Lifecycle mode works with **all** sick_scan_xd compatible devices:

* **2D LiDAR**: TiM series, LMS series, NAV series
* **3D LiDAR**: multiScan100, picoScan100, MRS series, LRS4000
* **RADAR**: RMS series

----

Command Reference
=================

.. code-block:: bash

   # Check current state
   ros2 lifecycle get /sick_scan

   # List available transitions
   ros2 lifecycle list /sick_scan

   # Monitor state changes
   ros2 topic echo /sick_scan/transition_event

   # Full lifecycle sequence
   ros2 lifecycle set /sick_scan configure
   ros2 lifecycle set /sick_scan activate
   ros2 lifecycle set /sick_scan deactivate
   ros2 lifecycle set /sick_scan cleanup

----

Additional Resources
====================

* `ROS 2 Lifecycle Design <https://design.ros2.org/articles/node_lifecycle.html>`_
* `sick_scan_xd GitHub <https://github.com/SICKAG/sick_scan_xd>`_

----

.. note::
   **Version**: 3.5.0+
   
   **Last Updated**: February 17, 2026
