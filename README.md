# Mosaic2RosBridge

This Java project contains two different applications:
## RosVehicleApp
Mosaic Application to be executed on the simulated vehicle representing the Hardware-in-the-Loop node.
## LeadingVehicle_CamSendingApp
Mosaic Application that, when mapped on a simulated vehicle, sends (according to a configurable time period) CAM messages informing its neighbours about its (x,y,z) position and current speed (module).
(CAM messages are then propagated according to the network simulator and its settings configured in the Mosaic scenario)

# Recompilation
In the root of the project: $ mvn clean install
Then, place the .jar file under /mosaic_23.1/scenarios/scenario_name/application/

