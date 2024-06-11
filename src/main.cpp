#if __has_include(<Arduino.h>)
#include <Arduino.h>
#include <WiFi.h>
#else
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <stdint.h>
#include <stddef.h>
#include <esp_wifi.h>
#define WIFI_CONNECTED_BIT BIT0
#define WIFI_FAIL_BIT      BIT1
uint32_t millis() {
    return pdTICKS_TO_MS(xTaskGetTickCount());
}
void loop();
#endif
#include <driver/gpio.h>
#include <driver/spi_master.h>
#include <esp_lcd_panel_io.h>
#include <esp_lcd_panel_ops.h>
#include <esp_lcd_panel_vendor.h>
#include <button.hpp>
#include <uix.hpp>
#include <gfx.hpp>
#include <wifi_manager.hpp>
#include <ip_loc.hpp>
#include <ntp_time.hpp>
// font is a TTF/OTF from downloaded from fontsquirrel.com
// converted to a header with https://honeythecodewitch.com/gfx/converter
#define OPENSANS_REGULAR_IMPLEMENTATION
#include <assets/OpenSans_Regular.hpp>
// faWifi icon generated using https://honeythecodewitch.com/gfx/iconPack
#define ICONS_IMPLEMENTATION
#include <assets/icons.hpp>
// include this after everything else except ui.hpp
#include <config.hpp>
#include <ui.hpp>

// namespace imports
#ifdef ARDUINO
using namespace arduino;
#else
using namespace esp_idf;
#endif
using namespace gfx;
using namespace uix;

wifi_manager wifi_man;

// use two 32KB buffers (DMA)
static uint8_t lcd_transfer_buffer1[32*1024];
static uint8_t lcd_transfer_buffer2[32*1024];
// this is the handle from the esp panel api
static esp_lcd_panel_handle_t lcd_handle;

using button_t = multi_button;
static basic_button button_a_raw(35, 10, true);
static basic_button button_b_raw(0, 10, true);
button_t button_a(button_a_raw);
button_t button_b(button_b_raw);

static time_t time_now = 0;
static char time_buffer[32];
static char time_date_buffer[32];
static long time_offset = 0;
static ntp_time time_server;
static char time_zone_buffer[64];

typedef enum {
    CS_IDLE = 0,
    CS_CONNECTING = 1,
    CS_CONNECTED = 2,
    CS_FETCHING = 3,
    CS_POLLING = 4
} connection_state_t;
static connection_state_t connection_state= CS_IDLE;

// the screen/control definitions
screen_t main_screen(
    {240,135},
    sizeof(lcd_transfer_buffer1),
    lcd_transfer_buffer1,
    lcd_transfer_buffer2);
svg_clock_t ana_clock(main_screen);
label_t dig_clock(main_screen);
label_t dig_date(main_screen);
label_t time_zone(main_screen);
canvas_t wifi_icon(main_screen);

// tell UIX the DMA transfer is complete
static bool lcd_flush_ready(esp_lcd_panel_io_handle_t panel_io, esp_lcd_panel_io_event_data_t *edata, void *user_ctx)
{
    main_screen.flush_complete();
    return true;
}
// tell the lcd panel api to transfer data via DMA
static void lcd_on_flush(const rect16 &bounds, const void *bmp, void *state)
{
    int x1 = bounds.x1, y1 = bounds.y1, x2 = bounds.x2 + 1, y2 = bounds.y2 + 1;
    esp_lcd_panel_draw_bitmap(lcd_handle, x1, y1, x2, y2, (void *)bmp);
}
// initialize the screen using the esp panel API
static void lcd_panel_init()
{
    // backlight
    gpio_set_direction((gpio_num_t)4, GPIO_MODE_OUTPUT);
    spi_bus_config_t buscfg;
    memset(&buscfg, 0, sizeof(buscfg));
    buscfg.sclk_io_num = 18;
    buscfg.mosi_io_num = 19;
    buscfg.miso_io_num = -1;
    buscfg.quadwp_io_num = -1;
    buscfg.quadhd_io_num = -1;
    buscfg.max_transfer_sz = sizeof(lcd_transfer_buffer1) + 8;

    // Initialize the SPI bus on VSPI (SPI3)
    spi_bus_initialize(SPI3_HOST, &buscfg, SPI_DMA_CH_AUTO);

    esp_lcd_panel_io_handle_t io_handle = NULL;
    esp_lcd_panel_io_spi_config_t io_config;
    memset(&io_config, 0, sizeof(io_config));
    io_config.dc_gpio_num = 16,
    io_config.cs_gpio_num = 5,
    io_config.pclk_hz = 40 * 1000 * 1000,
    io_config.lcd_cmd_bits = 8,
    io_config.lcd_param_bits = 8,
    io_config.spi_mode = 0,
    io_config.trans_queue_depth = 10,
    io_config.on_color_trans_done = lcd_flush_ready;
    // Attach the LCD to the SPI bus
    esp_lcd_new_panel_io_spi((esp_lcd_spi_bus_handle_t)SPI3_HOST, &io_config, &io_handle);

    lcd_handle = NULL;
    esp_lcd_panel_dev_config_t panel_config;
    memset(&panel_config, 0, sizeof(panel_config));
    panel_config.reset_gpio_num = 23;
#if ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(5, 0, 0)
    panel_config.rgb_endian = LCD_RGB_ENDIAN_RGB;
#else
    panel_config.color_space = ESP_LCD_COLOR_SPACE_RGB;
#endif
    panel_config.bits_per_pixel = 16;

    // Initialize the LCD configuration
    esp_lcd_new_panel_st7789(io_handle, &panel_config, &lcd_handle);

    // Turn off backlight to avoid unpredictable display on the LCD screen while initializing
    // the LCD panel driver. (Different LCD screens may need different levels)
    gpio_set_level((gpio_num_t)4, 0);
    // Reset the display
    esp_lcd_panel_reset(lcd_handle);

    // Initialize LCD panel
    esp_lcd_panel_init(lcd_handle);
    // esp_lcd_panel_io_tx_param(io_handle, LCD_CMD_SLPOUT, NULL, 0);
    //  Swap x and y axis (Different LCD screens may need different options)
    esp_lcd_panel_swap_xy(lcd_handle, true);
    esp_lcd_panel_set_gap(lcd_handle, 40, 52);
    esp_lcd_panel_mirror(lcd_handle, false, true);
    esp_lcd_panel_invert_color(lcd_handle, true);
    // Turn on the screen
#if ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(5, 0, 0)
    esp_lcd_panel_disp_on_off(lcd_handle, true);
#else
    esp_lcd_panel_disp_off(lcd_handle, false);
#endif
    // Turn on backlight (Different LCD screens may need different levels)
    gpio_set_level((gpio_num_t)4, 1);
}

// updates the time strings with the current time and date
static void update_time_buffer(time_t time) {
    tm tim = *localtime(&time);
    strftime(time_buffer, sizeof(time_buffer), "%I:%M %p", &tim);
    if(*time_buffer=='0') {
        *time_buffer=' ';
    }
    strftime(time_date_buffer, sizeof(time_date_buffer), "%D", &tim);
}

static void wifi_icon_paint(surface_t& destination, const srect16& clip, void* state) {
    // if we're using the radio, indicate it with the appropriate icon
    if(wifi_man.state()==wifi_manager_state::connected||wifi_man.state()==wifi_manager_state::connecting) {
        draw::icon(destination,point16::zero(),faWifi,color_t::light_gray);
    }
}
void button_pressed(bool pressed, void* state) {
    if(pressed) {
        if(connection_state==CS_IDLE) {
            connection_state = CS_CONNECTING;
        }
    }
}
#ifdef ARDUINO
void setup()
{
    Serial.begin(115200);
#else
extern "C" void app_main() {
#endif
    lcd_panel_init();
    button_a.initialize();
    button_b.initialize();
    puts("Clock booted");
    // init the screen and callbacks
    main_screen.background_color(color_t::black);
    main_screen.on_flush_callback(lcd_on_flush);
    
    // init the analog clock, 64x64
    ana_clock.bounds(srect16(0,0,63,63).center_vertical(main_screen.bounds()));
    ana_clock.face_color(color32_t::light_gray);
    // make the second hand semi-transparent
    auto px = ana_clock.second_color();
    // use pixel metadata to figure out what half of the max value is
    // and set the alpha channel (A) to that value
    px.template channel<channel_name::A>(
        decltype(px)::channel_by_name<channel_name::A>::max/2);
    ana_clock.second_color(px);
    // do similar with the minute hand as the second hand
    px = ana_clock.minute_color();
    // same as above, but it handles it for you, using a scaled float
    px.template channelr<channel_name::A>(0.5f);
    ana_clock.minute_color(px);
    // make the whole thing dark
    ana_clock.hour_border_color(color32_t::gray);
    ana_clock.minute_border_color(ana_clock.hour_border_color());
    ana_clock.face_color(color32_t::black);
    ana_clock.face_border_color(color32_t::black);
    //ana_clock.tick_color(color32_t::black);
    main_screen.register_control(ana_clock);

    // init the digital clock, 128x40, to the right of the analog clock
    dig_clock.bounds(
        srect16(0,0,127,39)
            .center_vertical(main_screen.bounds())
            .offset(70,0));
    *time_buffer = 0;
    dig_clock.text(time_buffer);
    dig_clock.text_open_font(&text_font);
    dig_clock.text_line_height(35);
    dig_clock.text_color(color32_t::white);
    dig_clock.text_justify(uix_justify::top_middle);
    main_screen.register_control(dig_clock);
    
    *time_date_buffer = 0;
    dig_date.bounds(dig_clock.bounds().offset(0,-dig_clock.dimensions().height-2));
    dig_date.text(time_date_buffer);
    dig_date.text_open_font(&text_font);
    dig_date.text_line_height(35);
    dig_date.text_color(color32_t::white);
    dig_date.text_justify(uix_justify::top_middle);
    main_screen.register_control(dig_date);
    
    *time_zone_buffer = 0;
    time_zone.bounds(srect16(0,main_screen.dimensions().height-40,main_screen.dimensions().width-1,main_screen.dimensions().height-1));
    time_zone.text_open_font(&text_font);
    time_zone.text_line_height(30);
    time_zone.text_color(color32_t::light_sky_blue);
    time_zone.text_justify(uix_justify::top_middle);
    time_zone.text(time_zone_buffer);
    main_screen.register_control(time_zone);

    // set up a custom canvas for displaying our wifi icon
    wifi_icon.bounds(((srect16)faWifi.bounds()).offset(main_screen.dimensions().width-faWifi.dimensions().width,0));
    wifi_icon.on_paint_callback(wifi_icon_paint);
    main_screen.register_control(wifi_icon);
    button_a.on_pressed_changed(button_pressed);
    button_b.on_pressed_changed(button_pressed);

#ifndef ARDUINO
    while(1) {
        loop();
        //static int count = 0;
        //if(count>6) {
            vTaskDelay(5);
          //  count = 0;    
        //}
    }
#endif
}

void loop()
{
    ///////////////////////////////////
    // manage connection and fetching
    ///////////////////////////////////
    static uint32_t connection_refresh_ts = 0;
    static uint32_t time_ts = 0;
    switch(connection_state) { 
        case CS_IDLE:
        if(connection_refresh_ts==0 || millis() > (connection_refresh_ts+(time_refresh_interval*1000))) {
            connection_refresh_ts = millis();
            connection_state = CS_CONNECTING;
        }
        break;
        case CS_CONNECTING:
            time_ts = 0;
            wifi_icon.invalidate();
            if(wifi_man.state()!=wifi_manager_state::connected && wifi_man.state()!=wifi_manager_state::connecting) {
                puts("Connecting to network...");
                wifi_man.connect(wifi_ssid,wifi_pass);
                connection_state =CS_CONNECTED;
            } else if(wifi_man.state()==wifi_manager_state::connected) {
                connection_state = CS_CONNECTED;
            }
            break;
        case CS_CONNECTED:
            if(wifi_man.state()==wifi_manager_state::connected) {
                puts("Connected.");
                connection_state = CS_FETCHING;
            } else {
                connection_refresh_ts = 0; // immediately try to connect again
                connection_state = CS_IDLE;

            }
            break;
        case CS_FETCHING:
            puts("Retrieving time info...");
            connection_refresh_ts = millis();
            // grabs the timezone and tz offset based on IP
            ip_loc::fetch(nullptr,nullptr,&time_offset,nullptr,0,nullptr,0,time_zone_buffer,sizeof(time_zone_buffer));
            connection_state = CS_POLLING;
            time_ts = millis(); // we're going to correct for latency
            time_server.begin_request();
            break;
        case CS_POLLING:
            if(time_server.request_received()) {
                const int latency_offset = (millis()-time_ts)/1000;
                time_now=(time_t)(time_server.request_result()+time_offset+latency_offset);
                puts("Clock set.");
                // set the digital clock - otherwise it only updates once a minute
                update_time_buffer(time_now);
                dig_clock.invalidate();
                dig_date.invalidate();
                time_zone.invalidate();
                connection_state = CS_IDLE;
                puts("Turning WiFi off.");
                wifi_man.disconnect(true);
                wifi_icon.invalidate();
            } else if(millis()>time_ts+(wifi_fetch_timeout*1000)) {
                puts("Retrieval timed out. Retrying.");
                connection_state = CS_FETCHING;
            }
            break;
    }
    ///////////////////
    // Track time
    //////////////////
    static uint32_t tick_ts = millis();
    if(millis()>=tick_ts+1000) {
        tick_ts = millis();
        ++time_now;
    }
    ///////////////////
    // update the UI
    //////////////////
    ana_clock.time(time_now);
    // only update every minute (efficient)
    if(0==(time_now%60)) {
        update_time_buffer(time_now);
        // tell the labels the text changed
        dig_clock.invalidate();
        dig_date.invalidate();
    }
        
    //////////////////////////
    // pump various objects
    /////////////////////////
    time_server.update();
    main_screen.update();    
    
    button_a.update();
    button_b.update();
}
