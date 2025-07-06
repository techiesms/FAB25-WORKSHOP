/*
 * Simplified GNSS Geofencing Application
 * - Green LED: Inside geofence
 * - Blue LED: Outside geofence  
 * - Red LED: No GPS fix
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <nrf_modem_at.h>
#include <nrf_modem_gnss.h>
#include <modem/lte_lc.h>
#include <modem/nrf_modem_lib.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/devicetree.h>

LOG_MODULE_REGISTER(geofence_app, CONFIG_GNSS_SAMPLE_LOG_LEVEL);

#define PI 3.14159265358979323846
#define EARTH_RADIUS_METERS (6371.0 * 1000.0)

// Geofence configuration
#define GEOFENCE_RADIUS_METERS 1000.0  // 1 km radius
#define GEOFENCE_CENTER_LAT    23.014975
#define GEOFENCE_CENTER_LON    72.639570

// LED configuration
static const struct gpio_dt_spec red_led   = GPIO_DT_SPEC_GET(DT_ALIAS(led0), gpios);
static const struct gpio_dt_spec green_led = GPIO_DT_SPEC_GET(DT_ALIAS(led1), gpios);
static const struct gpio_dt_spec blue_led  = GPIO_DT_SPEC_GET(DT_ALIAS(led2), gpios);

enum color_state {
    RED,    // No fix
    GREEN,  // Inside geofence
    BLUE,   // Outside geofence
    OFF
};

// Global variables
static struct nrf_modem_gnss_pvt_data_frame last_pvt;
static K_SEM_DEFINE(pvt_data_sem, 0, 1);

// Convert degrees to radians
static double deg_to_rad(double deg) {
    return deg * PI / 180.0;
}

// Calculate distance using Haversine formula
static double haversine_distance(double lat1, double lon1, double lat2, double lon2) {
    double dlat = deg_to_rad(lat2 - lat1);
    double dlon = deg_to_rad(lon2 - lon1);

    lat1 = deg_to_rad(lat1);
    lat2 = deg_to_rad(lat2);

    double a = sin(dlat / 2.0) * sin(dlat / 2.0) +
               cos(lat1) * cos(lat2) *
               sin(dlon / 2.0) * sin(dlon / 2.0);

    double c = 2.0 * atan2(sqrt(a), sqrt(1.0 - a));

    return EARTH_RADIUS_METERS * c;
}

// Check if point is outside geofence
static bool is_outside_geofence(double lat, double lon) {
    double distance = haversine_distance(GEOFENCE_CENTER_LAT, GEOFENCE_CENTER_LON, lat, lon);
    LOG_INF("Distance from geofence center: %.2f meters", distance);
    return distance > GEOFENCE_RADIUS_METERS;
}

// Set LED color
void set_color(enum color_state color) {
    // Turn all LEDs off first (assuming active low)
    gpio_pin_set_dt(&red_led, 1);
    gpio_pin_set_dt(&green_led, 1);
    gpio_pin_set_dt(&blue_led, 1);

    switch (color) {
    case RED:
        gpio_pin_set_dt(&red_led, 0);
        break;
    case GREEN:
        gpio_pin_set_dt(&green_led, 0);
        break;
    case BLUE:
        gpio_pin_set_dt(&blue_led, 0);
        break;
    case OFF:
    default:
        break;
    }
}

// GNSS event handler
static void gnss_event_handler(int event) {
    int retval;

    switch (event) {
    case NRF_MODEM_GNSS_EVT_PVT:
        retval = nrf_modem_gnss_read(&last_pvt, sizeof(last_pvt), NRF_MODEM_GNSS_DATA_PVT);
        if (retval == 0) {
            k_sem_give(&pvt_data_sem);
        }
        break;
    default:
        break;
    }
}

// Initialize modem (without LTE connection)
static int modem_init(void) {
    // No LTE connection needed for basic GNSS operation
    LOG_INF("Modem initialized");
    return 0;
}

// Initialize and start GNSS
static int gnss_init_and_start(void) {
    // Enable GNSS functional mode (like in original code for ASSISTANCE_NONE)
    if (lte_lc_func_mode_set(LTE_LC_FUNC_MODE_ACTIVATE_GNSS) != 0) {
        LOG_ERR("Failed to activate GNSS functional mode");
        return -1;
    }

    // Set GNSS event handler
    if (nrf_modem_gnss_event_handler_set(gnss_event_handler) != 0) {
        LOG_ERR("Failed to set GNSS event handler");
        return -1;
    }

    // Configure GNSS use case (from original code)
    uint8_t use_case = NRF_MODEM_GNSS_USE_CASE_MULTIPLE_HOT_START;
    if (nrf_modem_gnss_use_case_set(use_case) != 0) {
        LOG_WRN("Failed to set GNSS use case");
    }

    // Configure for continuous tracking (from original code)
    if (nrf_modem_gnss_fix_retry_set(0) != 0) {
        LOG_ERR("Failed to set GNSS fix retry");
        return -1;
    }

    if (nrf_modem_gnss_fix_interval_set(1) != 0) {
        LOG_ERR("Failed to set GNSS fix interval");
        return -1;
    }

    // Start GNSS
    if (nrf_modem_gnss_start() != 0) {
        LOG_ERR("Failed to start GNSS");
        return -1;
    }

    LOG_INF("GNSS started successfully");
    return 0;
}

// Initialize LEDs
static int led_init(void) {
    if (!device_is_ready(red_led.port) || 
        !device_is_ready(green_led.port) || 
        !device_is_ready(blue_led.port)) {
        LOG_ERR("LED GPIO not ready");
        return -1;
    }

    gpio_pin_configure_dt(&red_led, GPIO_OUTPUT_INACTIVE);
    gpio_pin_configure_dt(&green_led, GPIO_OUTPUT_INACTIVE);
    gpio_pin_configure_dt(&blue_led, GPIO_OUTPUT_INACTIVE);
    
    set_color(OFF); // All LEDs off initially
    return 0;
}

// Process GPS fix and update geofence status
static void process_gps_fix(void) {
    if (last_pvt.flags & NRF_MODEM_GNSS_PVT_FLAG_FIX_VALID) {
        double lat = last_pvt.latitude;
        double lon = last_pvt.longitude;
        double alt = last_pvt.altitude;

        LOG_INF("GPS Fix: Lat=%.6f, Lon=%.6f, Alt=%.2f m", lat, lon, alt);

        if (is_outside_geofence(lat, lon)) {
            set_color(BLUE);  // Outside geofence
            LOG_INF("Status: OUTSIDE geofence");
        } else {
            set_color(GREEN); // Inside geofence
            LOG_INF("Status: INSIDE geofence");
        }
    } else {
        set_color(RED); // No fix
        LOG_INF("Status: NO GPS FIX");
    }
}

int main(void) {
    int err;

    LOG_INF("Starting GNSS Geofencing Application");
    LOG_INF("Geofence center: %.6f, %.6f", GEOFENCE_CENTER_LAT, GEOFENCE_CENTER_LON);
    LOG_INF("Geofence radius: %.0f meters", GEOFENCE_RADIUS_METERS);

    // Initialize LEDs
    if (led_init() != 0) {
        LOG_ERR("Failed to initialize LEDs");
        return -1;
    }

    // Initialize modem library
    err = nrf_modem_lib_init();
    if (err) {
        LOG_ERR("Modem library initialization failed, error: %d", err);
        return err;
    }

    // Initialize modem and GNSS (no LTE connection needed)
    if (modem_init() != 0) {
        LOG_ERR("Failed to initialize modem");
        return -1;
    }

    // Initialize and start GNSS
    if (gnss_init_and_start() != 0) {
        LOG_ERR("Failed to initialize and start GNSS");
        return -1;
    }

    LOG_INF("System initialized. Waiting for GPS fixes...");

    // Main loop - wait for GPS data and process geofencing
    for (;;) {
        if (k_sem_take(&pvt_data_sem, K_FOREVER) == 0) {
            process_gps_fix();
        }
    }

    return 0;
}