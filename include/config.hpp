#pragma once
#include <gfx.hpp>
#include "wifi_creds.h"
static constexpr const char* wifi_ssid = WIFI_SSID;
static constexpr const char* wifi_pass = WIFI_PASS;
static constexpr const unsigned int wifi_fetch_timeout = 30;
static constexpr const unsigned int time_refresh_interval = 10*60;
static const gfx::open_font& text_font = OpenSans_Regular;
