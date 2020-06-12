/*
This program is free software: you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation, either version 3 of the License, or
(at your option) any later version.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with this program.  If not, see <https://www.gnu.org/licenses/>.
*/

// Patrick Tissot, 12 June 2020
// based on work from https://github.com/khancyr/TTGO_T_BEAM
// ported to Lolin32 Lite board with external NEO-6M and SSD1306 modules
// Added:
//          - Screen navigation (rotate) using button short press
//              1st screen: GPS status/coordinates
//              2nd screen: Home location as memorized during startup
//              3rd screen: Model ID selection
//          - OLED auto power-off after 30sec of button inactivity (only if GPS home position already set)
//          - Multiple ID table with selection via press Button (long press to enter setting mode/long press to exit setting mode, only valid in 3rd screen, short press to select next model)
//          - store last ID used in emulated EEPROM (Flash of ESP32) and automatic recall at next startup
//          - Battery voltage monitoring (requires 100K+100K voltage divider from VBAT to pin 34) reported in bottom line
//			- R counter reported in bottom line left shows number of seconds the beacons have started
//			- F counter reported in bottom line right shows number of seconds it took to get a GPS fix
// Fixed (in droneID_FR.h file):
//          - distance travelled (absolute value and call to distanceBetween() with correct parameters in double)
//          - memorise last gps lon and lat only when sending a beacon (not every received GPS frame)


#include <Arduino.h>
#include "board_def.h"
#include <WiFi.h>
#include <Wire.h>
#include <Button2.h>
#include <Ticker.h>
// include library to read and write from flash memory
#include <EEPROM.h>
#include "droneID_FR.h"

extern "C" {
    #include "esp_wifi.h"
    esp_err_t esp_wifi_80211_tx(wifi_interface_t ifx, const void *buffer, int len, bool en_sys_seq);
}

SSD1306 oled(SSD1306_ADDRESS, I2C_SDA, I2C_SCL);
OLEDDisplayUi ui(&oled);

UBLOX_GPS_OBJECT();

droneIDFR drone_idfr;

/********************************************************************************************************************
* MODIFIEZ LES VALEURS ICI
********************************************************************************************************************/
// Set these to your desired credentials.
/**
* Le nom du point d'acces wifi CHANGEZ LE par ce que vous voulez !!!
*/
const char ssid[] = "ILLEGAL_DRONE_AP";

// Mot de pass du wifi
const char *password = "123456789";
/**
* CHANGEZ l'ID du drone par celui que Alphatango vous a fourni (Trigramme + Modèle + numéro série) !
* Le nom en-dessous est libre, il est utilisé pour le choix sur l'écran OLED
*/
const char drone_id0[] = "ILLEGAL_DRONE_1";
const char drone_name0[] = "Phantom 3";

const char drone_id1[] = "ILLEGAL_DRONE_2";
const char drone_name1[] = "Planeur";

const char drone_id2[] = "ILLEGAL_DRONE_3";
const char drone_name2[] = "Avion";

const char drone_id3[] = "ILLEGAL_DRONE_4";
const char drone_name3[] = "Helicoptère";

const char drone_id4[] = "ILLEGAL_DRONE_5";
const char drone_name4[] = "Mon Drone";


#define NB_MODEL  5
const char * drone_id[NB_MODEL] = {drone_id0, drone_id1, drone_id2, drone_id3, drone_id4};
const char * drone_name[NB_MODEL] = {drone_name0, drone_name1, drone_name2, drone_name3, drone_name4};

/********************************************************************************************************************/
// NE PAS TOUCHEZ A PARTIR D'ICI !!!
// Le wifi est sur le channel 6 conformement à la spécification
static constexpr uint8_t wifi_channel = 6;
// Ensure the drone_id is max 30 letters
static_assert((sizeof(ssid)/sizeof(*ssid))<=32, "AP SSID should be less than 32 letters");
// Ensure the drone_id is max 30 letters
static_assert((sizeof(drone_id0)/sizeof(*drone_id))<=31, "Drone ID should be less that 30 letters !");  // 30 lettres + null termination
// beacon frame definition
static constexpr uint16_t MAX_BEACON_SIZE = 40 + 32 + droneIDFR::FRAME_PAYLOAD_LEN_MAX;  // default beaconPacket size + max ssid size + max drone id frame size
uint8_t beaconPacket[MAX_BEACON_SIZE] = {
    0x80, 0x00,							            // 0-1: Frame Control
    0x00, 0x00,							            // 2-3: Duration
    0xff, 0xff, 0xff, 0xff, 0xff, 0xff,				// 4-9: Destination address (broadcast)
    0x24, 0x62, 0xab, 0xdd, 0xb0, 0xbd,				// 10-15: Source address FAKE  // TODO should be set manually
    0x24, 0x62, 0xab, 0xdd, 0xb0, 0xbd,				// 16-21: Source address FAKE
    0x00, 0x00,							            // 22-23: Sequence / fragment number (done by the SDK)
    0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07,	// 24-31: Timestamp (GETS OVERWRITTEN TO 0 BY HARDWARE)
    0xB8, 0x0B,							            // 32-33: Beacon interval: set to 3s == 3000TU== BB8, bytes in reverse order  // TODO: manually set it
    0x21, 0x04,							            // 34-35: Capability info
    0x03, 0x01, 0x06,						        // 36-38: DS Parameter set, current channel 6 (= 0x06), // TODO: manually set it
    0x00, 0x20,                     				// 39-40: SSID parameter set, 0x20:maxlength:content
    // 41-XX: SSID (max 32)
};

bool has_set_home = false;
double home_alt = 0.0;
double home_lat = 0.0;
double home_long = 0.0;

bool ssd1306_found = false;

char buff[5][60];
char home_lg[40];
char home_la[40];
char home_al[40];
char battery[40];

uint64_t gpsSec = 0;
#define BUTTONS_MAP {BUTTON_PIN}

Button2 *pBtns = nullptr;
uint8_t g_btns[] =  BUTTONS_MAP;
#define ARRARY_SIZE(a)   (sizeof(a) / sizeof(a[0]))

Ticker btnTick;

String recv = "";

uint64_t runSec;
uint64_t button_press;
uint8_t oled_status;
uint8_t frame_disp;
uint8_t model;
uint8_t old_model;
uint8_t status;

/************************************
*      BUTTON
* *********************************/
void button_callback(Button2 &b)
{
    for (int i = 0; i < ARRARY_SIZE(g_btns); ++i) {
        if (pBtns[i] == b) {
            if (pBtns[0].wasPressedFor() < LONGCLICK_MS) {
                if (status == OPERATING) {
                    if (oled_status == 0) {
                        Serial.println("Display On");
                        oled.displayOn();
                        oled_status = 1;
                    } else {
                        ui.nextFrame();
                    }
                }
                if (status == EDIT) {
                    model++;
                    if (model == NB_MODEL)
                        model = 0;
                    ui.update(); // refresh screen
                    Serial.print("New ID = ");
                    Serial.println(drone_name[model]);
                }
            }
            button_press = millis();
        }
    }
}

void buttonlong_callback(Button2 &b)
{
    if ((status == OPERATING) &&  (frame_disp == 2)){
        Serial.println("Settings Mode");
        status = EDIT;
    }
    else if (status == EDIT) {
        Serial.println("Operating Mode");
        status = OPERATING;
        if (model != old_model) {
            // Apply new Drone ID setting
            drone_idfr.set_drone_id(drone_id[model]);
            Serial.print("New ID = ");
            Serial.println(drone_name[model]);
            old_model = model;
            // save the model in flash memory
            EEPROM.write(0, model);
            EEPROM.commit();
        }
    }
}

void button_loop()
{
    for (int i = 0; i < ARRARY_SIZE(g_btns); ++i) {
        pBtns[i].loop();
    }
}

void button_init()
{
    uint8_t args = ARRARY_SIZE(g_btns);
    pBtns = new Button2 [args];
    for (int i = 0; i < args; ++i) {
        pBtns[i] = Button2(g_btns[i]);
        pBtns[i].setReleasedHandler(button_callback);
    }
    pBtns[0].setLongClickHandler(buttonlong_callback);
}

#ifdef ENABLE_SSD1306
// Take care of showing all "static" display info
// MODEL Name + R counter + Bat Voltage + F counter
void msOverlay(OLEDDisplay *display, OLEDDisplayUiState *state)
{
    static uint8_t count;

    display->setTextAlignment(TEXT_ALIGN_CENTER);
    display->setFont(ArialMT_Plain_10);
    if (status != EDIT) { // show model name
        display->drawString(64, 0, drone_name[model]);
    } else { // blink model name during selection process
        count++;
        if (count < 7)
            display->drawString(64, 0, drone_name[model]);
        else
            display->drawString(64, 0, "                ");
        if (count == 14)
            count = 0;
    }

	// Display battery voltage
    display->drawString(64, 53, battery);
    display->setTextAlignment(TEXT_ALIGN_LEFT);
	// Display seconds running counter (transmiting beacon)
    display->drawString(0, 53, "R"+String((uint32_t)runSec)+"s");
    display->setTextAlignment(TEXT_ALIGN_RIGHT);
	// Display seconds counter to reach GPS Fix
    display->drawString(127, 53, "F"+String((uint32_t)gpsSec)+"s");
}

// Display GPS info
void drawFrame1(OLEDDisplay *display, OLEDDisplayUiState *state, int16_t x, int16_t y)
{
    frame_disp = 0;
    display->setFont(ArialMT_Plain_10);
    display->setTextAlignment(TEXT_ALIGN_CENTER);

    if (!gps.location.isValid()) { // show GPS waiting message
        display->drawString(64 + x, 11 + y, buff[0]);
        display->drawString(64 + x, 22 + y, buff[1]);
    } else { // show GPS coordinates
        display->drawString(64 + x, 11 + y, buff[0]);
        display->drawString(64 + x, 22 + y, buff[1]);
        display->drawString(64 + x, 33 + y, buff[2]);
        display->drawString(64 + x, 44 + y, buff[3]);
    }
}

// Display Home position as recorded during startup
void drawFrame2(OLEDDisplay *display, OLEDDisplayUiState *state, int16_t x, int16_t y)
{
    frame_disp = 1;
    display->setFont(ArialMT_Plain_10);
    display->setTextAlignment(TEXT_ALIGN_CENTER);
    display->drawString(64 + x, y + 11, "HOME");
    display->drawString(64 + x, 22 + y, home_lg);
    display->drawString(64 + x, 33 + y, home_la);
    display->drawString(64 + x, 44 + y, home_al);
}

// Display Model selection
void drawFrame3(OLEDDisplay *display, OLEDDisplayUiState *state, int16_t x, int16_t y)
{
    frame_disp = 2;
    display->setFont(ArialMT_Plain_10);
    display->setTextAlignment(TEXT_ALIGN_CENTER);
    display->drawString(64 + x, 11 + y, "MODEL SELECT");
    display->drawString(64 + x, 22 + y, ssid);
    display->drawString(64 + x, 33 + y, drone_id[model]);
}

FrameCallback frames[] = {drawFrame1, drawFrame2, drawFrame3};
OverlayCallback overlays[] = { msOverlay };
#endif

void ssd1306_init()
{
    if (!ssd1306_found) {
        Serial.println("SSD1306 not found");
        return;
    }
    if (oled.init()) {
        oled_status = 1;
        oled.flipScreenVertically();
        oled.setFont(ArialMT_Plain_16);
        oled.setTextAlignment(TEXT_ALIGN_CENTER);
    } else {
        Serial.println("SSD1306 Begin FAIL");
    }
    Serial.println("SSD1306 Begin PASS");
    ui.setTargetFPS(20);
    ui.disableAutoTransition();
    ui.disableAllIndicators();
    ui.setFrameAnimation(SLIDE_LEFT);
    ui.setFrames(frames, ARRARY_SIZE(frames));
    // Add overlays
    ui.setOverlays(overlays, 1);
}


void scanI2Cdevice(void)
{
    byte err, addr;
    int nDevices = 0;
    for (addr = 1; addr < 127; addr++) {
        Wire.beginTransmission(addr);
        err = Wire.endTransmission();
        if (err == 0) {
            Serial.print("I2C device found at address 0x");
            if (addr < 16)
            Serial.print("0");
            Serial.print(addr, HEX);
            Serial.println(" !");
            nDevices++;

            if (addr == SSD1306_ADDRESS) {
                ssd1306_found = true;
                Serial.println("ssd1306 display found");
            }
        } else if (err == 4) {
            Serial.print("Unknow error at address 0x");
            if (addr < 16)
            Serial.print("0");
            Serial.println(addr, HEX);
        }
    }
    if (nDevices == 0)
    Serial.println("No I2C devices found\n");
    else
    Serial.println("done\n");
}



/**
* Phase de configuration
*/
void setup()
{
    status = IDLE;

    pinMode(LED_BUILTIN, OUTPUT); // configure LED pin
    digitalWrite(LED_BUILTIN, LOW); // turn ON builtin LED
    adcAttachPin(ADC_BAT_PIN); // configure battery monitoring pin

    // initialize EEPROM with predefined size (we only need 1 byte to store model number)
    EEPROM.begin(1);

    Serial.begin(115200);

    delay(1000);

    // read the last used model from flash memory (default to 0 if error)
    model = EEPROM.read(0);
    if (model >= NB_MODEL)
        model = 0;
    old_model = model;

    Wire.begin(I2C_SDA, I2C_SCL);

    scanI2Cdevice();

    button_init();

    ssd1306_init();

    Serial2.begin(GPS_BAUD_RATE, SERIAL_8N1, GPS_RX_PIN, GPS_TX_PIN);


    /********************************************************************************************************************
    * ICI ON INITIALISE LE WIFI
    */
    /**
    * Pour mon exemple, je crée un point d'accés. Il fait rien par defaut
    */
    Serial.println("Starting AP");
    WiFi.softAP(ssid, nullptr, wifi_channel);
    IPAddress myIP = WiFi.softAPIP();
    Serial.print("AP IP address: ");
    Serial.println(myIP);
    Serial.print("AP mac address: ");
    Serial.println(WiFi.macAddress());
    wifi_config_t conf_current;
    esp_wifi_get_config(WIFI_IF_AP, &conf_current);
    // Change WIFI AP default beacon interval sending to 1s
    conf_current.ap.beacon_interval = 1000;
    drone_idfr.set_drone_id(drone_id0);
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_AP, &conf_current));

    btnTick.attach_ms(20, button_loop);
}

/**
* Début du code principal, C'est une boucle infinie
*/
void loop()
{
    static uint64_t gpsMap = 0;
    static uint64_t adc_sample = 0;
    uint32_t battery_voltage;

    // on prend une mesure de la tension batterie toutes les 5 secondes
    if ((millis() - adc_sample) > 5000) {
        battery_voltage = ((analogRead(ADC_BAT_PIN) * 3300)/4095) * 2; // x2 to account for external 100K+100K divider
        snprintf(battery, sizeof(buff[0]), "B=%4d mV", battery_voltage);
        if (!ssd1306_found) {
            Serial.println(buff[0]);
        } else {
            ui.update();
        }
        adc_sample = millis();
    }

    // on retourne à l'écran d'accueil après 10s que la home position soit mémorisée (et que l'écran d'accueil n'est pas déjà affiché)
	if (frame_disp != 0) {
		if ((status == OPERATING) && ((millis() - button_press) > 10000)) {
			ui.switchToFrame(0); // display GPS screen
		}
    }

    // on éteint l'écran OLED après 30 secondes d'inactivité bouton (si le home position est défini et que l'écran est actif)
    if ((oled_status == 1) && (status == OPERATING)) {
        if ((millis() - button_press) > 30000) {
            Serial.println("Display Off");
            oled.displayOff();
            oled_status = 0;
        }
    }
	
    // Ici on lit les données qui arrive du GPS et on les passes à la librairie TinyGPS++ pour les traiter
    while (Serial2.available())
        gps.encode(Serial2.read());

    // On traite le cas ou le GPS a un probleme
    if ((millis() > 5000) && (gps.charsProcessed() < 10)) {
        snprintf(buff[0], sizeof(buff[0]), "External GPS");
        snprintf(buff[1], sizeof(buff[1]), "No GPS detected");
        status = GPS_FAIL;
        if (!ssd1306_found) {
            Serial.println(buff[1]);
        }
        return;
    }

    // On traite le cas si la position GPS n'est pas valide
    if (!gps.location.isValid()) {
        status = WAIT_GPS;
        if ((millis() - gpsMap) > 1000) {
            snprintf(buff[0], sizeof(buff[0]), "External GPS");
            snprintf(buff[1], sizeof(buff[1]), "Positioning");
            gpsSec++;
            if (!ssd1306_found) {
                Serial.println(buff[1]);
            }
            gpsMap = millis();
        }
    } else {  // On traite le cas si la position GPS est valide

        // On renseigne le point de démarrage quand la précision est satisfaisante
        if (!has_set_home && (gps.satellites.value() > 6) && (gps.hdop.hdop() < 2.0)) {
            Serial.println("Setting Home Position");
            drone_idfr.set_home_lat_lon(gps.location.lat(), gps.location.lng());
            home_lat = gps.location.lat();
            home_long = gps.location.lng();
            home_alt = gps.altitude.meters(); // PT: warning, the altitude measurement may not be accurate at this point (too early)
            has_set_home = true;
            // PT: latitude & longitude avec 5 chiffres après la virgule => environ 1m de résolution
            // PT: 0.00001 degré / 360 degrés * 40 000 000 m (périmètre de la terre) = 1.1m
            snprintf(home_lg, sizeof(home_lg), "LNG: %.5f", home_long);
            snprintf(home_la, sizeof(home_la), "LAT: %.5f", home_lat);
            snprintf(home_al, sizeof(home_al), "ALT: %4.0f m", home_alt);

            #if 1
            Serial.println(home_lg);
            Serial.println(home_la);
            Serial.println(home_al);
            #endif
            ui.transitionToFrame(1); // display home screen
            button_press = millis();
            status = OPERATING;
        }

        if (!has_set_home) {
            status = WAIT_HOME;
        }

        // On envoie les données a la librairie d'identification drone pour le formattage
        drone_idfr.set_lat_lon(gps.location.lat(), gps.location.lng());
        drone_idfr.set_altitude(gps.altitude.meters());
        drone_idfr.set_heading(gps.course.deg());
        drone_idfr.set_ground_speed(gps.speed.mps());
        drone_idfr.set_heigth(gps.altitude.meters() - home_alt);

        // Ici on ecrit sur le port USB les données GPS pour visualisation seulement
        if ((millis() - gpsMap) > 1000) {
            runSec++;
            digitalWrite(LED_BUILTIN, LOW); // turn ON builtin LED (to show we are receiving GPS frames)
            snprintf(buff[0], sizeof(buff[0]), "UTC: %d:%d:%d", gps.time.hour(), gps.time.minute(), gps.time.second());
            snprintf(buff[1], sizeof(buff[1]), "LNG: %.5f", gps.location.lng());
            snprintf(buff[2], sizeof(buff[2]), "LAT: %.5f", gps.location.lat());
            snprintf(buff[3], sizeof(buff[3]), "satellites: %u", gps.satellites.value());
            if (!ssd1306_found) {
                Serial.println(buff[0]);
                Serial.println(buff[1]);
                Serial.println(buff[2]);
                Serial.println(buff[3]);
            }
            gpsMap = millis();
        }

        // on éteint la led après 100ms
        if ((millis() - gpsMap) > 100) {
            digitalWrite(LED_BUILTIN, HIGH); // turn OFF builtin LED
        }
    }

    // On regarde s'il est temps d'envoyer la trame d'identification drone: soit toutes les 3s soit si le drones s'est déplacé de 30m en moins de 3s
    if (drone_idfr.time_to_send() && (status == OPERATING)) {
		// si on a parcouru au moins 3m (évite de se déclencher au repos dû à la fluctuation position GPS)
        if (drone_idfr.get_travelled_distance() > 2) {
			// on envoie une trace sur la console série
            char buffer[80];
            sprintf(buffer, "DIST : %8d", drone_idfr.get_travelled_distance());
            Serial.println(buffer);
			// remplacer le 0 ci-dessous pour afficher à la console série le suivi de trajectoire du drone
            #if 0
            sprintf(buffer, "OLAT: %8d", _old_latitude);
            Serial.println(buffer);
            sprintf(buffer, "LAT : %8d", _latitude);
            Serial.println(buffer);
            sprintf(buffer, "OLNG: %8d", _old_longitude);
            Serial.println(buffer);
            sprintf(buffer, "LNG : %8d", _longitude);
            Serial.println(buffer);
            #endif
        }

		// on envoie sur la console série la raison d'envoi du beacon
        if (drone_idfr.has_pass_distance()) {
            Serial.println(">Send beacon (distance)");
        } else {
            Serial.println(String(millis())+" ms >Send beacon (timeout)"); // le millis permettra de vérifier que l'on a bien un envoi toutes les 3 secondes même sans déplacement
        }
		
        /**
        * On commence par renseigner le ssid du wifi dans la trame
        */
        // write new SSID into beacon frame
        const size_t ssid_size = (sizeof(ssid)/sizeof(*ssid)) - 1; // remove trailling null termination
        beaconPacket[40] = ssid_size;  // set size
        memcpy(&beaconPacket[41], ssid, ssid_size); // set ssid
        const uint8_t header_size = 41 + ssid_size;  //TODO: remove 41 for a marker
        /**
        * On génère la trame wifi avec l'identfication
        */
        const uint8_t to_send = drone_idfr.generate_beacon_frame(beaconPacket, header_size);  // override the null termination
        // Décommenter ce block pour voir la trame entière sur le port usb
        /*Serial.println("beaconPacket : ");
        for (auto i=0; i<sizeof(beaconPacket);i++) {
            Serial.print(beaconPacket[i], HEX);
            Serial.print(" ");
        }
        Serial.println(" ");*/

        // On envoie la trame
        ESP_ERROR_CHECK(esp_wifi_80211_tx(WIFI_IF_AP, beaconPacket, to_send, true));

        // On reset la condition d'envoi
        drone_idfr.set_last_send();
    }

    if (ssd1306_found) {
        if (ui.update()) {
        }
    }
}
