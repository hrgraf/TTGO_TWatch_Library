/*
Copyright (c) 2019 lewis he
This is just a demonstration. Most of the functions are not implemented.
The main implementation is low-power standby.
The off-screen standby (not deep sleep) current is about 4mA.
Select standard motherboard and standard backplane for testing.
Created by Lewis he on October 10, 2019.
*/

// Copyright (c) 2020 by H.R.Graf
// for improvements and extensions done by H.R.Graf in 2020

// Please select the model you want to use in config.h
#include "config.h"
#include "lvgl/lvgl.h"
#include "gui.h"

#include <WiFi.h>
#include <HTTPClient.h>         //Remove Audio Lib error
#include <time.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/timers.h"
#include "freertos/queue.h"

#include <soc/rtc.h>

#include "AudioFileSourcePROGMEM.h"
#include "AudioGeneratorWAV.h"
#include "AudioOutputI2S.h"

#include "hallo_tina_hihi.h" // wav
#define WAVE_FILE hallo_tina_hihi

static const char *TIME_ZONE  = "CET-1CEST-2,M3.5.0/02:00:00,M10.5.0/03:00:00";
static const char *NTP_SERVER = "pool.ntp.org";

static const char *SSID = "YOUR_SSID";
static const char *PASS = "YOUR_PASSWD";

#define G_EVENT_VBUS_PLUGIN         _BV(0)
#define G_EVENT_VBUS_REMOVE         _BV(1)
#define G_EVENT_CHARGE_DONE         _BV(2)

#define G_EVENT_WIFI_SCAN_START     _BV(3)
#define G_EVENT_WIFI_SCAN_DONE      _BV(4)
#define G_EVENT_WIFI_CONNECTED      _BV(5)
#define G_EVENT_WIFI_BEGIN          _BV(6)
#define G_EVENT_WIFI_OFF            _BV(7)

enum {
    Q_EVENT_WIFI_SCAN_DONE,
    Q_EVENT_WIFI_CONNECT,
    Q_EVENT_BMA_INT,
    Q_EVENT_AXP_INT,
} ;

#define DEFAULT_SCREEN_TIMEOUT  10*1000

#define WATCH_FLAG_SLEEP_MODE   _BV(1)
#define WATCH_FLAG_SLEEP_EXIT   _BV(2)
#define WATCH_FLAG_BMA_IRQ      _BV(3)
#define WATCH_FLAG_AXP_IRQ      _BV(4)

static QueueHandle_t g_event_queue_handle = NULL;
static EventGroupHandle_t g_event_group = NULL;
static EventGroupHandle_t isr_group = NULL;
static bool lenergy = false;
static uint32_t num_tasks_handled = 0;

static TTGOClass *ttgo;

static AudioGeneratorWAV *wav;
static AudioFileSourcePROGMEM *file;
static AudioOutputI2S *out;

static bool run_audio()
{
    if (! wav)
        return false;

    if (!(wav->isRunning()))
        return false;

    if (wav->loop())
    {
        //lv_disp_trig_activity(NULL);
        return true;
    }

    wav->stop();
    MY_LOG("WAV done");
    ttgo->power->setPowerOutPut(AXP202_LDO3, AXP202_OFF);
    delete file;
    lv_disp_trig_activity(NULL);
    return false;
}

void play_sound()
{
    if (! wav)
        return;

    if (!(wav->isRunning())) 
    {
        MY_LOG("PLAY WAV");
        ttgo->enableLDO3(); // audio power
        file = new AudioFileSourcePROGMEM(WAVE_FILE, sizeof(WAVE_FILE));
        wav->begin(file, out);
        while (run_audio())
            ;
    }
}

uint16_t get_bat_level()
{
    run_audio();
    return ttgo->power->getBattPercentage();
}

uint16_t get_bat_charging()
{
    run_audio();
    return ttgo->power->isChargeing();
}

uint32_t get_free_mem(void)
{
    //if (!(ttgo->motor))
    //    ttgo->motor_begin();
    //ttgo->shake();

    MY_LOG("MEM info");
    heap_caps_print_heap_info(MALLOC_CAP_8BIT);

    size_t sz = heap_caps_get_free_size(MALLOC_CAP_8BIT);

    //delay(100);
    //delete ttgo->motor;
    //ttgo->motor = NULL;

    return sz;
}

void get_accel(lv_point_t *dir)
{
    Accel acc;
    ttgo->bma->getAccel(acc);
    //MY_LOG("ACC %d / %d / %d", acc.x, acc.y, acc.z);

    dir->x = acc.x;
    dir->y = acc.y;
}

void setup_wifi()
{
    WiFi.mode(WIFI_STA);
    WiFi.onEvent([](WiFiEvent_t event, WiFiEventInfo_t info) {
        MY_LOG("WIFI DISCONNECTED");
        xEventGroupClearBits(g_event_group, G_EVENT_WIFI_CONNECTED);
    }, WiFiEvent_t::SYSTEM_EVENT_STA_DISCONNECTED);

    WiFi.onEvent([](WiFiEvent_t event, WiFiEventInfo_t info) {
        MY_LOG("WIFI SCAN DONE");
        uint8_t data = Q_EVENT_WIFI_SCAN_DONE;
        xQueueSend(g_event_queue_handle, &data, portMAX_DELAY);
    }, WiFiEvent_t::SYSTEM_EVENT_SCAN_DONE);

    WiFi.onEvent([](WiFiEvent_t event, WiFiEventInfo_t info) {
        MY_LOG("WIFI CONNECTED");
        xEventGroupSetBits(g_event_group, G_EVENT_WIFI_CONNECTED);
    }, WiFiEvent_t::SYSTEM_EVENT_STA_CONNECTED);

    WiFi.onEvent([](WiFiEvent_t event, WiFiEventInfo_t info) {
        MY_LOG("WIFI GOT IP");
        wifi_connect_status(true);
    }, WiFiEvent_t::SYSTEM_EVENT_STA_GOT_IP);
}

void ntp_sync_time()
{
    MY_LOG(WiFi.SSID().c_str());
    MY_LOG(WiFi.localIP().toString().c_str());

    delay(100);

    configTzTime(TIME_ZONE, NTP_SERVER);
    MY_LOG("NTP Tz");

    delay(100);

    struct tm info;
    if (getLocalTime(&info))
    {
        //Serial.println(&info, "%A, %B %d %Y %H:%M:%S");
        MY_LOG("NTP: %d %d %d - %d %d %d \n", info.tm_year+1900, info.tm_mon+1, info.tm_mday, info.tm_hour, info.tm_min, info.tm_sec);

        ttgo->rtc->syncToRtc();
    }

    WiFi.disconnect(true);
    WiFi.mode(WIFI_OFF);
}

void update_time(bool update_date)
{
    run_audio();

    uint32_t num = num_tasks_handled;
    num_tasks_handled = 0;
    MY_LOG("%d tasks", num);
    run_audio();

    time_t now;
    time(&now);
    run_audio();

    struct tm  info;
    localtime_r(&now, &info);
    run_audio();
    
//  static char buf[64];
//  strftime(buf, sizeof(buf), "%H:%M:%S", &info);
//  MY_LOG(buf);
//  run_audio();

    updateTime(info.tm_hour, info.tm_min, info.tm_sec);
    run_audio();
    if (update_date)
    {
        updateDate(info.tm_year+1900, info.tm_mon+1, info.tm_mday, info.tm_wday);
        run_audio();
    }
}

static void low_energy()
{
    MY_LOG("LOW ENERGY");
    if (ttgo->bl->isOn()) {
        xEventGroupSetBits(isr_group, WATCH_FLAG_SLEEP_MODE);
        ttgo->closeBL();

        showHome(); // close current app
        lv_task_handler();

        ttgo->stopLvglTick();
        ttgo->bma->enableStepCountInterrupt(false);
        ttgo->displaySleep();
        if (!WiFi.isConnected()) {
            MY_LOG("ENTER SLEEP");
            lenergy = true;
            WiFi.mode(WIFI_OFF);
            // rtc_clk_cpu_freq_set(RTC_CPU_FREQ_2M);
            setCpuFrequencyMhz(20);

            gpio_wakeup_enable((gpio_num_t)AXP202_INT, GPIO_INTR_LOW_LEVEL);
            gpio_wakeup_enable((gpio_num_t)BMA423_INT1, GPIO_INTR_HIGH_LEVEL);
            esp_sleep_enable_gpio_wakeup();
            esp_light_sleep_start();
        }
    } else {
        ttgo->startLvglTick();
        esp_sleep_source_t src = esp_sleep_get_wakeup_cause();
        MY_LOG("WAKE UP %d", src);
        ttgo->displayWakeup();
        ttgo->rtc->syncToSystem();

        wifi_connect_status(WiFi.isConnected());

        updateStepCounter(ttgo->bma->getCounter());
        updateBatteryLevel();
        update_time(true);

        lv_disp_trig_activity(NULL);
        showHome(); // close current app
        lv_task_handler();

        ttgo->openBL();
        ttgo->setBrightness(128);
        ttgo->bma->enableStepCountInterrupt();
    }
}

static void my_lv_log_print(lv_log_level_t level, const char *file, uint32_t line, const char *func, const char *buf)
{
    static const char * lvl_prefix[] = {"Trace", "Info", "Warn", "Error", "User"};
    /*Use only the file name not the path*/
    size_t p;
    for(p = strlen(file); p > 0; p--) {
        if(file[p] == '/' || file[p] == '\\') {
            p++;    /*Skip the slash*/
            break;
        }
    }

    MY_LOG("%s: %s \t(%s #%d %s())", lvl_prefix[level], buf, &file[p], line, func);
}

void setup()
{
    Serial.begin(115200);
    Serial.println("SETUP SIMPLEWATCH");
    lv_log_register_print_cb(my_lv_log_print);

    //Create a program that allows the required message objects and group flags
    g_event_queue_handle = xQueueCreate(20, sizeof(uint8_t));
    g_event_group = xEventGroupCreate();
    isr_group = xEventGroupCreate();

    //Initialize TWatch
    ttgo = TTGOClass::getWatch();
    ttgo->begin();

    //Initialize lvgl
    ttgo->lvgl_begin();

    // Turn on the IRQ used
    ttgo->power->adc1Enable(AXP202_BATT_VOL_ADC1 | AXP202_BATT_CUR_ADC1 | AXP202_VBUS_VOL_ADC1 | AXP202_VBUS_CUR_ADC1, AXP202_ON);
    ttgo->power->enableIRQ(AXP202_VBUS_REMOVED_IRQ | AXP202_VBUS_CONNECT_IRQ | AXP202_CHARGING_FINISHED_IRQ, AXP202_ON);
    ttgo->power->clearIRQ();

    // Turn off unused power
    ttgo->power->setPowerOutPut(AXP202_EXTEN, AXP202_OFF);
    ttgo->power->setPowerOutPut(AXP202_DCDC2, AXP202_OFF);
    ttgo->power->setPowerOutPut(AXP202_LDO3, AXP202_OFF);
    ttgo->power->setPowerOutPut(AXP202_LDO4, AXP202_OFF);

    // Enable BMA423 interrupt ，
    // The default interrupt configuration,
    // you need to set the acceleration parameters, please refer to the BMA423_Accel example
    ttgo->bma->attachInterrupt();

    //Connection interrupted to the specified pin
    pinMode(BMA423_INT1, INPUT);
    attachInterrupt(BMA423_INT1, [] {
        BaseType_t xHigherPriorityTaskWoken = pdFALSE;
        EventBits_t  bits = xEventGroupGetBitsFromISR(isr_group);
        if (bits & WATCH_FLAG_SLEEP_MODE)
        {
            //! For quick wake up, use the group flag
            xEventGroupSetBitsFromISR(isr_group, WATCH_FLAG_SLEEP_EXIT | WATCH_FLAG_BMA_IRQ, &xHigherPriorityTaskWoken);
        } else
        {
            uint8_t data = Q_EVENT_BMA_INT;
            xQueueSendFromISR(g_event_queue_handle, &data, &xHigherPriorityTaskWoken);
        }

        if (xHigherPriorityTaskWoken)
        {
            portYIELD_FROM_ISR ();
        }
    }, RISING);

    // Connection interrupted to the specified pin
    pinMode(AXP202_INT, INPUT);
    attachInterrupt(AXP202_INT, [] {
        BaseType_t xHigherPriorityTaskWoken = pdFALSE;
        EventBits_t  bits = xEventGroupGetBitsFromISR(isr_group);
        if (bits & WATCH_FLAG_SLEEP_MODE)
        {
            //! For quick wake up, use the group flag
            xEventGroupSetBitsFromISR(isr_group, WATCH_FLAG_SLEEP_EXIT | WATCH_FLAG_AXP_IRQ, &xHigherPriorityTaskWoken);
        } else
        {
            uint8_t data = Q_EVENT_AXP_INT;
            xQueueSendFromISR(g_event_queue_handle, &data, &xHigherPriorityTaskWoken);
        }
        if (xHigherPriorityTaskWoken)
        {
            portYIELD_FROM_ISR ();
        }
    }, FALLING);

    //Check if the RTC clock matches, if not, use compile time
    ttgo->rtc->check();

    //Synchronize time to system time
    ttgo->rtc->syncToSystem();

#ifdef LILYGO_WATCH_HAS_BUTTON
    /*
        ttgo->button->setClickHandler([]() {
            MY_LOG("Button2 Pressed");
        });
    */

    //Set the user button long press to restart
    ttgo->button->setLongClickHandler([]() {
        MY_LOG("Pressed Restart Button,Restart now ...");
        delay(1000);
        esp_restart();
    });
#endif

    //Setting up the network
    MY_LOG("SETUP WIFI");
    setup_wifi();

    //Execute your own GUI interface
    MY_LOG("SETUP GUI");
    setupGui();

    updateBatteryLevel();
    updateStepCounter(ttgo->bma->getCounter());
    update_time(true);

    lv_disp_trig_activity(NULL);

#ifdef LILYGO_WATCH_HAS_BUTTON
    //In lvgl we call the button processing regularly
    lv_task_create([](lv_task_t *args) {
        ttgo->button->loop();
    }, 30, 1, nullptr);
#endif

    // prepare audio
#if defined(STANDARD_BACKPLANE)
    out = new AudioOutputI2S(0, 1);
#elif defined(EXTERNAL_DAC_BACKPLANE)
    out = new AudioOutputI2S();
    //External DAC decoding
    out->SetPinout(TWATCH_DAC_IIS_BCK, TWATCH_DAC_IIS_WS, TWATCH_DAC_IIS_DOUT);
#endif
    wav = new AudioGeneratorWAV();
    //wav->SetBufferSize(16*1024);

    //When the initialization is complete, turn on the backlight
    ttgo->openBL();
    ttgo->setBrightness(128);

    // connect to WiFi
    WiFi.begin(SSID, PASS);
}

void loop()
{
    bool  rlst;
    uint8_t data;
    //! Fast response wake-up interrupt
    EventBits_t  bits = xEventGroupGetBits(isr_group);
    if (bits & WATCH_FLAG_SLEEP_EXIT) {
        if (lenergy) {
            lenergy = false;
            // rtc_clk_cpu_freq_set(RTC_CPU_FREQ_160M);
            setCpuFrequencyMhz(160);
            MY_LOG("EXIT SLEEP");
        }

        low_energy();

        if (bits & WATCH_FLAG_BMA_IRQ) {
            do {
                rlst =  ttgo->bma->readInterrupt();
            } while (!rlst);
            xEventGroupClearBits(isr_group, WATCH_FLAG_BMA_IRQ);
        }
        if (bits & WATCH_FLAG_AXP_IRQ) {
            ttgo->power->readIRQ();
            ttgo->power->clearIRQ();
            //TODO: Only accept axp power pek key short press
            xEventGroupClearBits(isr_group, WATCH_FLAG_AXP_IRQ);
        }
        xEventGroupClearBits(isr_group, WATCH_FLAG_SLEEP_EXIT);
        xEventGroupClearBits(isr_group, WATCH_FLAG_SLEEP_MODE);
    }
    if ((bits & WATCH_FLAG_SLEEP_MODE)) {
        //! No event processing after entering the information screen
        return;
    }

    //! Normal polling
    if (xQueueReceive(g_event_queue_handle, &data, 5 / portTICK_RATE_MS) == pdPASS) {
        switch (data) {
        case Q_EVENT_BMA_INT:
            do {
                rlst =  ttgo->bma->readInterrupt();
            } while (!rlst);

            //! setp counter
            if (ttgo->bma->isStepCounter()) {
                updateStepCounter(ttgo->bma->getCounter());
            }
            break;
        case Q_EVENT_AXP_INT:
            ttgo->power->readIRQ();
            if (ttgo->power->isVbusPlugInIRQ()) {
                updateBatteryCharge();
            }
            if (ttgo->power->isVbusRemoveIRQ()) {
                updateBatteryCalc();
            }
            if (ttgo->power->isChargingDoneIRQ()) {
                updateBatteryCalc();
            }
            if (ttgo->power->isPEKShortPressIRQ()) {
                ttgo->power->clearIRQ();
                low_energy();
                return;
            }
            ttgo->power->clearIRQ();
            break;
        case Q_EVENT_WIFI_SCAN_DONE: {
            int16_t len =  WiFi.scanComplete();
            for (int i = 0; i < len; ++i) {
                wifi_list_add(WiFi.SSID(i).c_str());
            }
            break;
        }
        default:
            break;
        }

    }

//  while (run_audio())
//  {
//      num_tasks_handled++;
//      if (! (num_tasks_handled & 0xFF))
//          lv_task_handler();
//  }

    if (lv_disp_get_inactive_time(NULL) < DEFAULT_SCREEN_TIMEOUT) 
    {
        num_tasks_handled++;
        lv_task_handler();
        delay(10);
    } 
    else 
    {
        low_energy();
    }
}

