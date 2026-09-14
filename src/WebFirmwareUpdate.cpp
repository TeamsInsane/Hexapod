#include <Arduino.h>
#include <Update.h>
#include <esp_ota_ops.h>
#include <esp_partition.h>

#include "HexapodState.h"
#include "CannonPreparation.h"
#include "GaitController.h"
#include "ObstacleNavigation.h"
#include "CannonController.h"
#include "WebFirmwareUpdate.h"


namespace Hexapod {

void handleFirmwareDownload() {
    const esp_partition_t *partition = esp_ota_get_running_partition();
    const size_t firmwareSize = ESP.getSketchSize();
    if (partition == nullptr || firmwareSize == 0U || firmwareSize > partition->size) {
        webServer.send(500, "text/plain", "Firmware unavailable");
        return;
    }

    webServer.sendHeader("Content-Disposition", "attachment; filename=\"hexapod-current-firmware.bin\"");
    webServer.sendHeader("Cache-Control", "no-store");
    webServer.setContentLength(firmwareSize);
    webServer.send(200, "application/octet-stream", "");

    WiFiClient client = webServer.client();
    uint8_t buffer[1024];
    size_t offset = 0U;
    while (offset < firmwareSize && client.connected()) {
        const size_t chunkSize = min(sizeof(buffer), firmwareSize - offset);
        if (esp_partition_read(partition, offset, buffer, chunkSize) != ESP_OK || client.write(buffer, chunkSize) != chunkSize) {
            break;
        }

        offset += chunkSize;
        delay(0);
    }
}

void handleUpdateUpload() {
    HTTPUpload &upload = webServer.upload();

    if (upload.status == UPLOAD_FILE_START) {
        cancelCannonPreparation("cancelled for OTA update");
        cancelObstacleNavigation();
        standState.stage = StandStage::Idle;
        cancelManualAim();
        cannonState.aimTrackingEnabled = false;
        updateAimServo();
        stopShootServo();
        stopTiltServo();
        stopChargeServo();

        if (!Update.begin(UPDATE_SIZE_UNKNOWN)) {
            Update.printError(Serial);
        }
    } else if (upload.status == UPLOAD_FILE_WRITE) {
        if (Update.write(upload.buf, upload.currentSize) != upload.currentSize) {
            Update.printError(Serial);
        }
    } else if (upload.status == UPLOAD_FILE_END) {
        if (!Update.end(true)) {
            Update.printError(Serial);
        }
    }
}

void handleUpdateFinished() {
    const bool ok = !Update.hasError();
    webServer.send(ok ? 200 : 500, "text/plain", ok ? "Update complete. Rebooting..." : "Update failed.");
    delay(500);

    if (ok) {
        ESP.restart();
    }
}

} // namespace Hexapod
