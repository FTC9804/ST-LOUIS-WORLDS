# ST-LOUIS-WORLDS
Written by the programmers of FTC Team 9804 Bomb Squad for WORLDS in St. Louis during April 2016

Standard Naming Conventions

*Worlds_9804_RED_Auto_Name_v1

*Worlds_9804_TeleOp_v1

==========================================================

* the IR Distance Sensor returns a value beteeen 0 and 1024 according to Team Torch. 0 is no light returned. 
* Use "floorIR.getLightDetectedRaw() < LIGHT_THRESHOLD && opModeIsActive()" as the conditional for line detected 
