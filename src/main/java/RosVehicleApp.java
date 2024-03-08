/**
 * RosVehicleApp
 * Alessandro Monaco
 *
 * This is an application to enable Hardware-In-the-Loop (HIL) simulation in Eclipse Mosaic with bidirectional UDP
 * socket communication with remote C/C++/ROS2 executables.
 *
 *  .__________________________________.                  .__________________________________.
 *  |                                  |   UDP port 1234  |                                  |
 *  |      RosVehicleApp      outSocket|---(mosaic2hil)-->|inSocket      HIL applicative     |
 *  |   (this applicative)             |                  |                 (remote)         |
 *  |                          inSocket|<--(hil2mosaic)---|outSocket                         |
 *  L__________________________________|   UDP port 1235  L__________________________________|
 *
 **/

package org.eclipse.mosaic.app.alex;

// Generic Java imports
import java.io.ByteArrayInputStream;
import java.io.ByteArrayOutputStream;
import java.io.DataInputStream;
import java.io.DataOutputStream;
import java.io.IOException;
import java.io.Serializable;
import java.lang.Double;
import java.lang.Math;
import static java.lang.Math.abs;
import java.net.DatagramPacket;
import java.net.DatagramSocket;
import java.net.InetAddress;
import java.net.SocketException;
import java.net.SocketTimeoutException;
import java.net.UnknownHostException;
import java.nio.ByteBuffer;
import java.nio.ByteOrder;
import java.nio.charset.StandardCharsets;
import java.util.List;
import javax.annotation.Nonnull;
import javax.annotation.Nullable;
import org.apache.commons.lang3.Validate;

// Eclipse Mosaic imports
import org.eclipse.mosaic.fed.application.ambassador.simulation.communication.AdHocModuleConfiguration;
import org.eclipse.mosaic.fed.application.ambassador.simulation.communication.CamBuilder;
import org.eclipse.mosaic.fed.application.ambassador.simulation.communication.ReceivedAcknowledgement;
import org.eclipse.mosaic.fed.application.ambassador.simulation.communication.ReceivedV2xMessage;
import org.eclipse.mosaic.fed.application.app.AbstractApplication;
import org.eclipse.mosaic.fed.application.app.api.CommunicationApplication;
import org.eclipse.mosaic.fed.application.app.api.VehicleApplication;
import org.eclipse.mosaic.fed.application.app.api.os.VehicleOperatingSystem;
import org.eclipse.mosaic.interactions.vehicle.VehicleSensorActivation.SensorType;
import org.eclipse.mosaic.interactions.communication.V2xMessageTransmission;
import org.eclipse.mosaic.lib.enums.AdHocChannel;
import org.eclipse.mosaic.lib.objects.v2x.V2xMessage;
import org.eclipse.mosaic.lib.objects.v2x.etsi.Cam;
import org.eclipse.mosaic.lib.objects.vehicle.VehicleData;
import org.eclipse.mosaic.lib.util.scheduling.Event;
import org.eclipse.mosaic.lib.util.SerializationUtils;

public class RosVehicleApp extends AbstractApplication<VehicleOperatingSystem> implements VehicleApplication, CommunicationApplication {

    // CONFIGURATION PARAMETERS

    // NETWORK configuration
    // inSocketPort: on which UDP port I'm listening
    int inSocketPort = 1235;
    // outSocketAddress: ROS2 host's address
    String outSocketAddress = "127.0.0.1";
    // outSocketPort: ROS2 host's UDP port
    int outSocketPort = 1234;

    DatagramSocket inSocket;
    DatagramSocket outSocket;

    // informations about myself
    public Double projectedPositionX = 0.0;
    public Double projectedPositionY = 0.0;
    public Double projectedPositionZ = 0.0;
    public Double speed = 0.0;

    // informations about the leader
    public Double leader_projectedPositionX = 0.0;
    public Double leader_projectedPositionY = 0.0;
    public Double leader_projectedPositionZ = 0.0;
    public Double leader_speed = 0.0;

    boolean initSpeed = false;
    int m2hNumber = 0;

    // Data structure RECEIVED FROM the HIL node.
    // It contains all the relevant information the simulation needs to know about the HIL node.
    static public class hil2mosaic {
        // progressive number
        public Integer number;
        // HIL node's current speed
        public Double speed;
    }

    // Data structure SENT TO the HIL node.
    // It contains all the relevant information the HIL node needs to know about its simulated counterpart.
    static public class mosaic2hil {
        // distance from the leading vehicle
        public Double leader_distance;
        // speed of the leading vehicle
        public Double leader_velocity;

        public String toString() {
            return "leader_distance=" + leader_distance + " | leader_velocity=" + leader_velocity;
        }

        public byte[] getBytes() {
            // Calculate the total size of the byte array needed
            int size = 2*Double.BYTES;
            // Create a ByteBuffer with the calculated size
            ByteBuffer buffer = ByteBuffer.allocate(size).order(ByteOrder.LITTLE_ENDIAN);
            // Put the fields into the ByteBuffer
            buffer.putDouble(leader_distance);
            buffer.putDouble(leader_velocity);
            // Get the resulting byte array
            return buffer.array();
        }
    }

    hil2mosaic h2m = new hil2mosaic();
    mosaic2hil m2h = new mosaic2hil();

    // The onStartup() method is executed once, at the application's startup (when the vehicle is spawned)
    @Override
    public void onStartup() {
        getLog().info("[INIT] RosVehicleApp started");

        // Setting up the communication module
        getOs().getAdHocModule().enable(new AdHocModuleConfiguration()
                .addRadio().channel(AdHocChannel.CCH).power(50).create()
        );
        getLog().infoSimTime(this, "[INIT] Communication module started");

        // inSocket initialization
        getLog().info("[INIT] Creating inSocket (listening on UDP port " + inSocketPort + ")");
        try {
            inSocket = new DatagramSocket(inSocketPort);
        } catch (SocketException e) {
            throw new RuntimeException(e);
        }
        getLog().info("[INIT] inSocket created");

        // outSocket initialization
        getLog().info("[INIT] Creating outSocket (destination " + outSocketAddress + ":" + outSocketPort + ")");
        try {
            outSocket = new DatagramSocket();
        } catch (SocketException e) {
            throw new RuntimeException(e);
        }
        getLog().info("[INIT] outSocket created.");
    }

    // The onVehicleUpdated() method is executed at each simulation step when one (or more) vehicle parameters are
    // modified/updated.
    @Override
    public void onVehicleUpdated(VehicleData previousVehicleData, VehicleData updatedVehicleData) {

        // update informations about myself
        projectedPositionX = updatedVehicleData.getProjectedPosition().getX();
        projectedPositionY = updatedVehicleData.getProjectedPosition().getY();
        projectedPositionZ = updatedVehicleData.getProjectedPosition().getZ();
        speed = updatedVehicleData.getSpeed();

        // UPDATE mosaic2hil data structure
        if (leader_projectedPositionX == 0.0) {
            // no information about the leading vehicle have been received yet
            // so, the leading vehicle is considered stopped and indefinitely away
            m2h.leader_distance = 10000.0;
            m2h.leader_velocity = 0.0;
        } else {
            // information about the leading vehicle has been received
            // calculate leading vehicle's distance and update mosaic2hil data structure
            m2h.leader_distance = Math.sqrt(Math.pow((leader_projectedPositionX - projectedPositionX), 2.0) + Math.pow((leader_projectedPositionY - projectedPositionY), 2.0));
            m2h.leader_velocity = leader_speed;
        }

        // SEND updated mosaic2hil data structure to HIL node
        byte[] sendData = m2h.getBytes();
        InetAddress serverAddress = null;
        try {
            serverAddress = InetAddress.getByName(outSocketAddress);
            DatagramPacket packet = new DatagramPacket(sendData, sendData.length, serverAddress, outSocketPort);
            getLog().info("[MOSAIC2HIL][" + m2hNumber + "] Sending (" + m2h + ") to HIL node");
            m2hNumber = m2hNumber + 1;
            outSocket.send(packet);
        } catch (UnknownHostException e) {
            throw new RuntimeException(e);
        } catch (IOException e) {
            throw new RuntimeException(e);
        }

        // initialize HIL vehicle's speed at 0.0 m/s
        if (initSpeed == false) {
            initSpeed = true;
            getOs().changeSpeedWithInterval(0, (long) 0.0);
            getLog().info("[INIT] Setting my speed to 0.0 m/s");
        }

        // RECEIVE hil2mosaic data structure from HIL node
        byte[] buffer = new byte[16];
        DatagramPacket inPacket = new DatagramPacket(buffer, buffer.length);

        // packetReceived = how many packets there were in the UDP socket's buffer (waiting to be received)
        int packetReceived = 0;

        try {
            // set a maximum wait timer for the UDP packet's blocking inSocket.receive() (5 ms)
            inSocket.setSoTimeout(5);

            // this cycle flushes all the packets that were in the UDP socket's buffer (waiting to be received)
            // effectively only elaborating the most up-to-date one (if at least one's there)
            while (true) {
                try {
                    inSocket.receive(inPacket);
                    // if the inSocket.receive() function doesn't generate a SocketTimeoutException,
                    // I've received a packet. So, the packetReceived counter is increased by 1
                    packetReceived++;
                } catch (SocketTimeoutException e) {
                    // if the inSocket.receive() function generates a SocketTimeoutException,
                    // there are no more packets to receive (if any, packetReceived may be 0 at this point)
                    break;
                }
            }
        } catch (IOException e) {
            throw new RuntimeException(e);
        }

        // check if I've received any packet
        if (packetReceived >= 1) {
            // if there was at least one, the most recent one overwrote all others (it's the desired behavior)
            // it has the most up-to-date data about the HIL node, so that data is used
            h2m.number = bytesToInt(buffer, 0, 4);
            h2m.speed = bytesToDouble(buffer, 8, 8);
            getOs().changeSpeedWithInterval(Double.valueOf(h2m.speed), (long) 0.0);
            getLog().info("[HIL2MOSAIC][" + h2m.number + "] Applied (latest) speed (" + h2m.speed + ") from HIL node");
        } else {
            // this branch is executed when the timeout expires and no packets were received
            // we may want to stop the vehicle or do some other actions
            // in the current statee, we do nothing (the HIL vehicle continues to move with the last received speed)
        }

    }

    // The onMessageReceived() method is executed when the HIL vehicle receives a V2x message
    // in this test case, a CAM message from the leading vehicle
    @Override
    public void onMessageReceived(ReceivedV2xMessage receivedV2xMessage) {
        V2xMessage msg = receivedV2xMessage.getMessage();
        LeadingVehicle_CamSendingApp.LeadingVehicleInfo lvi = new LeadingVehicle_CamSendingApp.LeadingVehicleInfo();

        if (msg instanceof Cam) {
            try {
                // extract (deserialization) the custom user-tagged data from the CAM message (x,y,z position and speed
                // of the leading vehicle)
                lvi = LeadingVehicle_CamSendingApp.DEFAULT_OBJECT_SERIALIZATION.fromBytes(((Cam) msg).getUserTaggedValue(), this.getClass().getClassLoader());
                // update our data about the leading vehicle
                leader_projectedPositionX = lvi.x;
                leader_projectedPositionY = lvi.y;
                leader_projectedPositionZ = lvi.z;
                leader_speed = lvi.speed;

                getLog().info("[CAM][" + lvi.camNumber + "] Received LeadingVehicleInfo: " + lvi);
            } catch (IOException | ClassNotFoundException e) {
                getLog().error("[CAM][ERROR] An error occurred while receiving a CAM message", e);
            }

        } else {
            getLog().infoSimTime(this, "[CAM][ERROR] Arrived message was not a CAM, but a {} msg from {}", msg.getSimpleClassName(), msg.getRouting().getSource().getSourceName());
        }
    }

    @Override
    public void onAcknowledgementReceived(ReceivedAcknowledgement receivedAcknowledgement) {
        // nop
    }

    @Override
    public void onCamBuilding(CamBuilder camBuilder) {
        // nop
    }

    @Override
    public void onMessageTransmitted(V2xMessageTransmission v2xMessageTransmission) {
        // nop
    }

    @Override
    public void onShutdown() {
        getLog().info("[SHUTDOWN] Closing sockets");
        inSocket.close();
        outSocket.close();
        getLog().info("[SHUTDOWN] Sockets closed");
        getLog().info("[SHUTDOWN] AlexApp1 stopped");
    }

    @Override
    public void processEvent(Event event) {
        // nop
    }

    // utility function to receive integers over a network socket (from a C/C++ applicative)
    private int bytesToInt(byte[] bytes, int offset, int length) {
        // Convert byte array to integer (assuming big-endian)
        int result = 0;
        for (int i = length - 1; i >= 0; i--) {
            result = (result << 8) | (bytes[offset + i] & 0xFF);
        }
        return result;
    }

    // utility function to receive doubles over a network socket (from a C/C++ applicative)
    private double bytesToDouble(byte[] bytes, int offset, int length) {
        ByteBuffer buffer = ByteBuffer.wrap(bytes, offset, length).order(ByteOrder.LITTLE_ENDIAN);
        return buffer.getDouble();
    }




}