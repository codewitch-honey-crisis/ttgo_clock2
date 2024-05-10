#include <Arduino.h>
#include <ttgo.hpp>
#include <uix.hpp>
#include <gfx.hpp>
#include <WiFi.h>
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
using namespace arduino;
using namespace gfx;
using namespace uix;

// use two 32KB buffers (DMA)
static uint8_t lcd_transfer_buffer1[32*1024];
static uint8_t lcd_transfer_buffer2[32*1024];
static time_t time_now = 0;
static char time_buffer[32];
static long time_offset = 0;
static ntp_time time_server;
static char time_zone_buffer[64];
static bool time_fetching=false;

// the screen/control definitions
screen_t main_screen(
    {240,135},
    sizeof(lcd_transfer_buffer1),
    lcd_transfer_buffer1,
    lcd_transfer_buffer2);
svg_clock_t ana_clock(main_screen);
label_t dig_clock(main_screen);
label_t time_zone(main_screen);
canvas_t wifi_icon(main_screen);

// for dumping to the display (UIX)
static void lcd_flush(const rect16& bounds,const void* bmp,void* state) {
    // wrap the void* bitmap buffer with a read only (const) bitmap object
    // this is a light and fast op
    const const_bitmap<decltype(lcd)::pixel_type> cbmp(bounds.dimensions(),bmp);
    // send what we just created to the display
    draw::bitmap_async(lcd,bounds,cbmp,cbmp.bounds());
}
// for display DMA (UIX/GFX)
static void lcd_wait_flush(void* state) {
    // wait for any async transfers to complete
    lcd.wait_all_async();
}

// updates the time string with the current time
static void update_time_buffer(time_t time) {
    tm tim = *localtime(&time);
    strftime(time_buffer, sizeof(time_buffer), "%I:%M %p", &tim);
    if(*time_buffer=='0') {
        *time_buffer=' ';
    }
}

static void wifi_icon_paint(surface_t& destination, const srect16& clip, void* state) {
    // if we're using the radio, indicate it with the appropriate icon
    if(time_fetching) {
        draw::icon(destination,point16::zero(),faWifi,color_t::light_gray);
    }
}

void setup()
{
    Serial.begin(115200);
    ttgo_initialize();
    lcd.rotation(3);
    Serial.println("Clock booted");
    // init the screen and callbacks
    main_screen.background_color(color_t::black);
    main_screen.on_flush_callback(lcd_flush);
    main_screen.wait_flush_callback(lcd_wait_flush);
    
    // init the analog clock, 64x64
    ana_clock.bounds(srect16(0,0,63,63).center_vertical((srect16)lcd.bounds()));
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

    if(wifi_ssid!=nullptr) {
        Serial.print("Using fixed SSID and credentials: ");
        Serial.println(wifi_ssid);
    } else {
        Serial.println("Using stored SSID and credentials.");
    }
    
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

    time_zone.bounds(srect16(0,main_screen.dimensions().height-40,main_screen.dimensions().width-1,main_screen.dimensions().height-1));
    time_zone.text_open_font(&text_font);
    time_zone.text_line_height(30);
    time_zone.text_color(color32_t::light_sky_blue);
    time_zone.text_justify(uix_justify::top_middle);
    main_screen.register_control(time_zone);

    // set up a custom canvas for displaying our wifi icon
    wifi_icon.bounds(
        srect16(spoint16(0,0),(ssize16)wifi_icon.dimensions())
            .offset(main_screen.dimensions().width-
                wifi_icon.dimensions().width,0));
    wifi_icon.on_paint_callback(wifi_icon_paint);
    main_screen.register_control(wifi_icon);
    
}

void loop()
{
    ///////////////////////////////////
    // manage connection and fetching
    ///////////////////////////////////
    static int connection_state=0;
    static uint32_t connection_refresh_ts = 0;
    static uint32_t time_ts = 0;
    IPAddress time_server_ip;
    switch(connection_state) { 
        case 0: // idle
        if(connection_refresh_ts==0 || millis() > (connection_refresh_ts+(time_refresh_interval*1000))) {
            connection_refresh_ts = millis();
            connection_state = 1;
            time_ts = 0;
        }
        break;
        case 1: // connecting
            time_fetching = true;
            wifi_icon.invalidate();
            if(WiFi.status()!=WL_CONNECTED) {
                Serial.println("Connecting to network...");
                if(wifi_ssid==nullptr) {
                    WiFi.begin();
                } else {
                    WiFi.begin(wifi_ssid,wifi_pass);
                }
                connection_state =2;
            } else if(WiFi.status()==WL_CONNECTED) {
                connection_state = 2;
            }
            break;
        case 2: // connected
            if(WiFi.status()==WL_CONNECTED) {
                Serial.println("Connected.");
                connection_state = 3;
            } else if(WiFi.status()==WL_CONNECT_FAILED) {
                connection_refresh_ts = 0; // immediately try to connect again
                connection_state = 0;
                time_fetching = false;
            }
            break;
        case 3: // fetch
            Serial.println("Retrieving time info...");
            connection_refresh_ts = millis();
            // grabs the timezone offset based on IP
            ip_loc::fetch(nullptr,nullptr,&time_offset,nullptr,0,nullptr,0,time_zone_buffer,sizeof(time_zone_buffer));
            WiFi.hostByName(time_server_domain,time_server_ip);
            connection_state = 4;
            time_ts = millis(); // we're going to correct for latency
            time_server.begin_request(time_server_ip);
            break;
        case 4: // polling for response
            if(time_server.request_received()) {
                const int latency_offset = (millis()-time_ts)/1000;
                time_now=(time_t)(time_server.request_result()+time_offset+latency_offset);
                Serial.println("Clock set.");
                // set the digital clock - otherwise it only updates once a minute
                update_time_buffer(time_now);
                dig_clock.invalidate();
                time_zone.text(time_zone_buffer);
                connection_state = 0;
                Serial.println("Turning WiFi off.");
                WiFi.disconnect(true,false);
                time_fetching = false;
                wifi_icon.invalidate();
            } else if(millis()>time_ts+(wifi_fetch_timeout*1000)) {
                Serial.println("Retrieval timed out. Retrying.");
                connection_state = 3;
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
    time_t time = time_now;
    ana_clock.time(time);
    // only update every minute (efficient)
    if(0==(time%60)) {
        update_time_buffer(time);
        // tell the label the text changed
        dig_clock.invalidate();
    }
        
    //////////////////////////
    // pump various objects
    /////////////////////////
    time_server.update();
    main_screen.update();    
    ttgo_update();
    dimmer.wake(); // don't dim
}