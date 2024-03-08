/**
 * LeadingVehicle_CamSendingApp
 * Alessandro Monaco
 *
 * This is a simple application which sends CAMs (Cooperative Awareness Message) with an additional information
 * (user tagged value) by using the {@link CamBuilder#userTaggedValue(byte[])}) method.
 *
 * Through this mechanism, additional byte field(s) can be sent via CAM.
 * In this case, the sender's current speed (its module) and its current (x,y,z) position are embedded into the CAMs to
 * inform nearby vehicles about these informations.
 *
 * The CAMs will be sent by an ad hoc module so that only vehicles with an enabled ad hoc module can receive it.
 **/

package org.eclipse.mosaic.app.alex;

// Generic Java imports
import java.io.IOException;
import java.io.Serializable;
import javax.annotation.Nonnull;
import javax.annotation.Nullable;

// Eclipse Mosaic imports
import org.eclipse.mosaic.fed.application.ambassador.simulation.communication.AdHocModuleConfiguration;
import org.eclipse.mosaic.fed.application.ambassador.simulation.communication.CamBuilder;
import org.eclipse.mosaic.fed.application.ambassador.simulation.communication.ReceivedAcknowledgement;
import org.eclipse.mosaic.fed.application.ambassador.simulation.communication.ReceivedV2xMessage;
import org.eclipse.mosaic.fed.application.app.AbstractApplication;
import org.eclipse.mosaic.fed.application.app.api.CommunicationApplication;
import org.eclipse.mosaic.fed.application.app.api.VehicleApplication;
import org.eclipse.mosaic.fed.application.app.api.os.VehicleOperatingSystem;
import org.eclipse.mosaic.interactions.communication.V2xMessageTransmission;
import org.eclipse.mosaic.lib.enums.AdHocChannel;
import org.eclipse.mosaic.lib.objects.vehicle.VehicleData;
import org.eclipse.mosaic.lib.util.SerializationUtils;
import org.eclipse.mosaic.lib.util.scheduling.Event;
import org.eclipse.mosaic.rti.TIME;

public class LeadingVehicle_CamSendingApp extends AbstractApplication<VehicleOperatingSystem>
        implements VehicleApplication, CommunicationApplication {

    // This parameter sets the sending rate of the CAM messages. A CAM message is sent by the application every
    // sendingPeriodMilliseconds ms.
    public int sendingPeriodMilliseconds = 20;
    public int stopTimeSeconds = 35;

    // Content of CAM message sent by the sender (the leading vehicle) to nearby vehicle(s)
    public static class LeadingVehicleInfo implements Serializable {
        public int camNumber = 0;
        // leading vehicle's current (most up to date) position (x,y,z)
        public Double x = 0.0;
        public Double y = 0.0;
        public Double z = 0.0;
        // leading vehicle's current (most up to date) speed (module)
        public Double speed = 0.0;

        @Override
        public String toString() {
            return "position (" + x + ", " + y + ", " + z + ") | speed " + speed + " m/s";
        }
    }

    // This class converts an object into a byte field and vice versa
    public static final SerializationUtils<LeadingVehicleInfo> DEFAULT_OBJECT_SERIALIZATION = new SerializationUtils<>();

    // Create a new LeadingVehicleInfo object, containing the informations kept updated about myself (the leading vehicle).
    LeadingVehicle_CamSendingApp.LeadingVehicleInfo lvi = new LeadingVehicle_CamSendingApp.LeadingVehicleInfo();

    // The onStartup() method is executed once, at the application's startup (when the vehicle is spawned)
    @Override
    public void onStartup() {
        getLog().info("[INIT] LeadingVehicle_CamSendingApp started");

        // Setting up the communication module
        getOs().getAdHocModule().enable(new AdHocModuleConfiguration()
                .camMinimalPayloadLength(200L)
                .addRadio().channel(AdHocChannel.CCH).power(50).create()
        );
        getLog().infoSimTime(this, "[INIT] Communication module started");

        // Scheduling first event (after sendingPeriodMilliseconds ms)
        getOs().getEventManager().addEvent(getOs().getSimulationTime() + sendingPeriodMilliseconds*TIME.MILLI_SECOND, this);
    }

    // The processEvent(Event event) method is executed at every event's scheduled time and does these two things:
    @Override
    public void processEvent(Event event) {
        // stop the vehicle if the desired time is surpassed
        if(getOs().getSimulationTime() > stopTimeSeconds*TIME.SECOND) {
            getOs().changeSpeedWithInterval(0, (long) 0.0);
        }

        // Sends the CAM message
        sendCam();
        // Schedules next event (after sendingPeriodMilliseconds ms)
        getOs().getEventManager().addEvent(getOs().getSimulationTime() + sendingPeriodMilliseconds*TIME.MILLI_SECOND, this);
    }

    // The sendCam() method actually sends the message on the adhoc module.
    private void sendCam() {
        getLog().infoSimTime(this, "[CAM][" + lvi.camNumber + "] Sending CAM");
        getOs().getAdHocModule().sendCam();
    }

    // No action is currently done when the leading vehicle receives a message.
    @Override
    public void onMessageReceived(ReceivedV2xMessage receivedV2xMessage) {
        // nop
    }

    // No action is currently done when the leading vehicle receives an acknowledgement.
    @Override
    public void onAcknowledgementReceived(ReceivedAcknowledgement acknowledgedMessage) {
        // nop
    }

    // The onCamBuilding(CamBuilder camBuilder) method is automatically triggered from the operating system (when a CAM
    // or DENM message is prepared to send) and handles its construction.
    @Override
    public void onCamBuilding(CamBuilder camBuilder) {
        try {
            byte[] byteArray = DEFAULT_OBJECT_SERIALIZATION.toBytes(lvi);
            // increase progressive number of cam message
            lvi.camNumber = lvi.camNumber + 1;
            camBuilder.userTaggedValue(byteArray);
        } catch (IOException ex) {
            getLog().error("[LVEHICLE][ERROR] Error during a serialization.", ex);
        }
    }

    @Override
    public void onMessageTransmitted(V2xMessageTransmission v2xMessageTransmission) {
        // nop
    }

    @Override
    public void onShutdown() {
        getLog().infoSimTime(this, "[LVEHICLE] Shutdown");
    }

    // The onVehicleUpdated() method is executed at each simulation step when one (or more) vehicle parameters are
    // modified/updated.
    @Override
    public void onVehicleUpdated(@Nullable VehicleData previousVehicleData, @Nonnull VehicleData updatedVehicleData) {
        // Update position (x,y,z)

        lvi.x = updatedVehicleData.getProjectedPosition().getX();
        lvi.y = updatedVehicleData.getProjectedPosition().getY();
        lvi.z = updatedVehicleData.getProjectedPosition().getZ();
        // Update speed
        lvi.speed = updatedVehicleData.getSpeed();
    }

}
