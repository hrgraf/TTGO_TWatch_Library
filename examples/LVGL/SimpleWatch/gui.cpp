// ------------------------------------------------------------------------
// Watch GUI - hardware independent
// Copyright (c) 2020 by H.R.Graf
// ------------------------------------------------------------------------

#include "lvgl/lvgl.h"
#include "gui.h"

// display size
#define WIDTH  240
#define HEIGHT 240

extern "C" LV_IMG_DECLARE(step);
extern "C" LV_IMG_DECLARE(white_face);
extern "C" LV_IMG_DECLARE(mickey);
extern "C" LV_IMG_DECLARE(hand_hour);
extern "C" LV_IMG_DECLARE(hand_min);
extern "C" LV_IMG_DECLARE(hand_sec);

static const char *day_names[7] = { "So", "Mo", "Di", "Mi", "Do", "Fr", "Sa" };
static const char *month_names[12] = {
	"Januar", "Februar", "Marz",
	"April", "Mai", "Juni",
	"Juli", "August", "September",
	"Oktober", "November", "Dezember"
};

// ------------------------------------------------------------------------
// Helpers
// ------------------------------------------------------------------------

static inline int32_t sq(int16_t x)
{
    int32_t y = x;
    return y * y;
}

static uint16_t tile_corner(const lv_point_t pos)
{
    int32_t R2 = sq((WIDTH + HEIGHT) / 6);

    if ((pos.x <= 0) && (pos.y <= 0))
        return 0;

    if (sq(WIDTH/2 - pos.x) + sq(HEIGHT/2 - pos.y) <= R2)
        return 5; // center

    if (sq(pos.x) + sq(pos.y) <= R2)
        return 1; // upper-left
    if (sq(WIDTH - pos.x) + sq(pos.y) <= R2)
        return 2; // upper-right
    if (sq(pos.x) + sq(HEIGHT - pos.y) <= R2)
        return 3; // lower-left
    if (sq(WIDTH - pos.x) + sq(HEIGHT - pos.y) <= R2)
        return 4; // lower-right

    return 0;
}

// ------------------------------------------------------------------------
// App Base Class
// ------------------------------------------------------------------------

class App
{
public:
    virtual ~App()
    {
        if (screen)
            lv_obj_del(screen);
    }

    void create(lv_obj_t *parent)
    {
        if (! parent)
            parent = lv_obj_create(NULL, NULL); // new screen

        lv_obj_t *tile = lv_cont_create(parent, NULL);
        lv_obj_set_style_local_border_side(tile, LV_CONT_PART_MAIN, LV_STATE_DEFAULT, LV_BORDER_SIDE_NONE);
        lv_obj_set_size(tile, WIDTH, HEIGHT);

        populate(tile);

        lv_obj_set_user_data(tile, this);
        lv_obj_set_event_cb(tile, tile_cb);

        screen = parent;
    }

    virtual void populate(lv_obj_t *parent)
    {
        lv_cont_set_layout(parent, LV_LAYOUT_CENTER);

        // label
        lv_obj_t *label = lv_label_create(parent, NULL);
        lv_obj_set_style_local_text_font(label, LV_LABEL_PART_MAIN, LV_STATE_DEFAULT, &lv_font_montserrat_24);
        lv_label_set_static_text(label, "Hallo Tina\n\nAlles Gute zum\n10. Geburtstag!!!");
        lv_label_set_align(label, LV_LABEL_ALIGN_CENTER);
    }

    static void tile_cb(lv_obj_t * obj, lv_event_t event)
    {
        static uint16_t ignore_click = 0;
        App *inst = (App *)lv_obj_get_user_data(obj);

        switch (event)
        {
        case LV_EVENT_PRESSED:
            ignore_click = 0; // reset at new touch
            break;

        case LV_EVENT_LONG_PRESSED:
            ignore_click = 1; // avoid immediate restart on clicked
            if (inst)
                inst->tile_long_press_cb();
            break;

        case LV_EVENT_CLICKED:
            if (inst && (!ignore_click))
                inst->tile_clicked_cb();
            break;

        }
    }

    virtual void tile_long_press_cb()
    {
        lv_point_t pos;
        lv_indev_get_point(lv_indev_get_act(), &pos);
        MY_LOG("Tile long press at %d / %d", pos.x, pos.y);
    }

    virtual void tile_clicked_cb()
    {
        lv_point_t pos;
        lv_indev_get_point(lv_indev_get_act(), &pos);
        MY_LOG("Tile clicked at %d / %d", pos.x, pos.y);

        showHome();
    }

    virtual void show()
    {
        if (screen)
            lv_scr_load(screen);
    }

    virtual void anim()
    {
        MY_LOG("ANIM");
    }

protected:
    lv_obj_t *screen;
};

// ------------------------------------------------------------------------
// forward declaration of local functions

//static bool showApp();
//static void createApp(uint16_t id);

static void startAnim(App *app);

// ------------------------------------------------------------------------
// Calendar App
// ------------------------------------------------------------------------

class CalendarApp : public App
{
public:
    virtual void populate(lv_obj_t *parent)
    {
        cal = lv_calendar_create(parent, NULL);
        lv_obj_set_style_local_border_side(cal, LV_CALENDAR_PART_BG, LV_STATE_DEFAULT, LV_BORDER_SIDE_NONE);
        lv_obj_set_style_local_text_color(cal, LV_CALENDAR_PART_DATE, LV_STATE_DISABLED, LV_COLOR_MAKE(0x40, 0x40, 0x40));
        lv_calendar_set_day_names(cal, day_names);
        lv_calendar_set_month_names(cal, month_names);
        lv_obj_set_size(cal, WIDTH, HEIGHT);
        lv_page_glue_obj(cal, true);

        lv_obj_set_user_data(cal, this);
        lv_obj_set_event_cb(cal, cal_cb);

        // Make the date number smaller to be sure they fit into their area
        //lv_obj_set_style_local_text_font(cal, LV_CALENDAR_PART_DATE, LV_STATE_DEFAULT, lv_theme_get_font_small());

        // Set date
        //update(2020, 9, 22);
    }

    static void cal_cb(lv_obj_t * obj, lv_event_t event)
    {
        static uint16_t year = 0, month = 0, day = 0;
        lv_calendar_date_t *date = NULL;

        switch (event)
        {
        case LV_EVENT_VALUE_CHANGED:
            date = lv_calendar_get_pressed_date(obj);
            if (date)
            {
                year = date->year;
                month = date->month;
                day = date->day;
                break;
            }
            // fall-thru
        case LV_EVENT_RELEASED:
            year = month = day = 0;
            break;

        case LV_EVENT_CLICKED:
            if (year)
            {
                MY_LOG("Clicked %d.%d.%d", day, month, year);
                showHome();
            }
            else
            {
                MY_LOG("Clicked no date");
            }
            break;
        }
    }

    void update(uint16_t year, uint16_t month, uint16_t day)
    {
        // set today's date
        lv_calendar_date_t today;
        today.year = year;
        today.month = (int8_t)month;
        today.day = (int8_t)day;
        lv_calendar_set_today_date(cal, &today);

        // show
        lv_calendar_set_showed_date(cal, &today);
    }

    virtual void show()
    {
        // show today's date
        lv_calendar_date_t *today = lv_calendar_get_today_date(cal);
        lv_calendar_set_showed_date(cal, today);

        // show app
        App::show();
    }

private:
    // GUI
    lv_obj_t *cal;
};

// ------------------------------------------------------------------------
// Level App
// ------------------------------------------------------------------------

class LevelApp : public App
{
public:
    virtual void populate(lv_obj_t *parent)
    {
        lv_cont_set_layout(parent, LV_LAYOUT_OFF);

        // label
        lv_obj_t * label = lv_label_create(parent, NULL);
        lv_label_set_static_text(label, "Wasserwaage");
        lv_obj_align(label, NULL, LV_ALIGN_IN_TOP_MID, 0, 20);

        // crosshair
        static lv_point_t horiz[2] = { {-WIDTH / 2, 0 }, {WIDTH / 2, 0} };
        static lv_point_t vert[2] = { {0, -HEIGHT / 2}, {0, HEIGHT / 2} };

        static lv_style_t style_line;
        lv_style_init(&style_line);
        lv_style_set_line_width(&style_line, LV_STATE_DEFAULT, 1);
        lv_style_set_line_color(&style_line, LV_STATE_DEFAULT, LV_COLOR_GRAY);

        lv_obj_t * line = lv_line_create(parent, NULL);
        lv_obj_add_style(line, LV_LINE_PART_MAIN, &style_line);
        lv_line_set_points(line, horiz, 2);
        lv_obj_align(line, NULL, LV_ALIGN_CENTER, 0, 0);

        line = lv_line_create(parent, NULL);
        lv_obj_add_style(line, LV_LINE_PART_MAIN, &style_line);
        lv_line_set_points(line, vert, 2);
        lv_obj_align(line, NULL, LV_ALIGN_CENTER, 0, 0);

        // bubble
        lv_obj_t * led = lv_led_create(parent, NULL);
        //lv_obj_set_style_local_outline_width(led, LV_LED_PART_MAIN, LV_STATE_DEFAULT, 0);
        //lv_obj_set_style_local_shadow_width(led, LV_LED_PART_MAIN, LV_STATE_DEFAULT, 0);
        lv_obj_set_style_local_shadow_spread(led, LV_LED_PART_MAIN, LV_STATE_DEFAULT, 3);
        //lv_obj_set_size(led, 15, 15);
        lv_obj_align(led, NULL, LV_ALIGN_CENTER, 0, 0);
        bubble = led;
    }

    void update(lv_coord_t x, lv_coord_t y)
    {
        if (bubble)
        {
            if (x >  WIDTH/2)
                x =  WIDTH/2;
            if (x < -WIDTH/2)
                x = -WIDTH/2;

            if (y >  HEIGHT/2)
                y =  HEIGHT/2;
            if (y < -HEIGHT/2)
                y = -HEIGHT/2;

            lv_obj_align(bubble, NULL, LV_ALIGN_CENTER, x, y);
        }
    }

    virtual void anim()
    {
        //MY_LOG("ANIM LEVEL");
        lv_disp_trig_activity(NULL);

        lv_point_t dir;
        get_accel(&dir);
        update(-dir.x, -dir.y);
    }

    virtual void show()
    {
        update(20, 20);

        // show app
        App::show();

        startAnim(this);
    }

private:
    // GUI
    lv_obj_t *bubble;
};

// ------------------------------------------------------------------------
// Battery App
// ------------------------------------------------------------------------

class BatApp : public App
{
public:
    virtual void populate(lv_obj_t *parent)
    {
        lv_obj_t *label;
        lv_cont_set_layout(parent, LV_LAYOUT_CENTER);

//      // label
//      label = lv_label_create(parent, NULL);
//      lv_label_set_static_text(label, "Status");
//      //lv_obj_align(label, NULL, LV_ALIGN_IN_TOP_MID, 0, 20);

        // label
        label = lv_label_create(parent, NULL);
        lv_label_set_static_text(label, "");
        lv_label_set_align(label, LV_LABEL_ALIGN_CENTER);
        //lv_obj_set_auto_realign(label, true);
        //lv_obj_align(label, NULL, LV_ALIGN_CENTER, 0, 0);
        lab_level = label;

        // label
        label = lv_label_create(parent, NULL);
        lv_label_set_static_text(label, "");
        lv_label_set_align(label, LV_LABEL_ALIGN_CENTER);
        //lv_obj_set_auto_realign(label, true);
        //lv_obj_align(label, NULL, LV_ALIGN_IN_BOTTOM_MID, 0, -20);
        lab_mem = label;
    }

    virtual void show()
    {
        // static content
        if (lab_level)
        {
            uint16_t level = get_bat_level();
            lv_label_set_text_fmt(lab_level, "Batterie:\n%d %%", level);
        }
        if (lab_mem)
        {
            uint32_t mem = get_free_mem();
            lv_label_set_text_fmt(lab_mem, "Freier Speicher:\n%d bytes", mem);
        }

        // show app
        App::show();
    }

private:
    // GUI
    lv_obj_t *lab_level, *lab_mem;
};

// ------------------------------------------------------------------------
// Watch App
// ------------------------------------------------------------------------

class WatchApp : public App
{
public:
    virtual void populate(lv_obj_t *parent)
    {
        lv_cont_set_layout(parent, LV_LAYOUT_OFF);

        img_bg = lv_img_create(parent, NULL);
        lv_img_set_src(img_bg, &white_face);
        lv_obj_align(img_bg, NULL, LV_ALIGN_CENTER, 0, 0);

        lab_tl = lv_label_create(parent, NULL);
        lv_label_set_static_text(lab_tl, LV_SYMBOL_WIFI);
        lv_obj_set_auto_realign(lab_tl, true);
        lv_obj_align(lab_tl, NULL, LV_ALIGN_IN_TOP_LEFT, 2, 2);

        lab_tr = lv_label_create(parent, NULL);
        lv_label_set_static_text(lab_tr, LV_SYMBOL_BATTERY_FULL);
        lv_label_set_align(lab_tr, LV_LABEL_ALIGN_RIGHT);
        lv_obj_set_auto_realign(lab_tr, true);
        lv_obj_align(lab_tr, NULL, LV_ALIGN_IN_TOP_RIGHT, -2, 2);

        lab_bl = lv_label_create(parent, NULL);
        lv_label_set_static_text(lab_bl, "Di\n22.9.");
        lv_obj_set_auto_realign(lab_bl, true);
        lv_obj_align(lab_bl, NULL, LV_ALIGN_IN_BOTTOM_LEFT, 2, -2);

        lv_obj_t *icon = lv_img_create(parent, NULL);
        lv_img_set_src(icon, &step);
        lv_obj_align(icon, NULL, LV_ALIGN_IN_BOTTOM_RIGHT, -2, -22);

        lab_br = lv_label_create(parent, NULL);
        lv_label_set_static_text(lab_br, "1001");
        lv_label_set_align(lab_br, LV_LABEL_ALIGN_RIGHT);
        lv_obj_set_auto_realign(lab_br, true);
        lv_obj_align(lab_br, NULL, LV_ALIGN_IN_BOTTOM_RIGHT, -2, -2);

        img_fig = lv_img_create(parent, NULL);
        lv_img_set_src(img_fig, &mickey);
        lv_obj_align(img_fig, NULL, LV_ALIGN_CENTER, 0, 10);

        img_hour = lv_img_create(parent, NULL);
        lv_img_set_src(img_hour, &hand_hour);
        lv_obj_align(img_hour, NULL, LV_ALIGN_CENTER, 0, 0);

        img_min = lv_img_create(parent, NULL);
        lv_img_set_src(img_min, &hand_min);
        lv_obj_align(img_min, NULL, LV_ALIGN_CENTER, 0, 0);

        img_sec = lv_img_create(parent, NULL);
        lv_img_set_src(img_sec, &hand_sec);
        lv_obj_align(img_sec, NULL, LV_ALIGN_CENTER, 0, 0);
    }

    virtual void tile_clicked_cb()
    {
        lv_point_t pos;
        lv_indev_get_point(lv_indev_get_act(), &pos);
        uint16_t corner = tile_corner(pos);
        MY_LOG("Tile clicked at %d / %d => corner %d", pos.x, pos.y, corner);

        if (!corner)
            return;

        if (corner == 5) // center
        {
            play_sound();
            return;
        }

//      if (!showApp())
//          createApp(corner);

        showApp(corner);
    }

    void setup(lv_task_t *anim_task, CalendarApp *my_cal)
    {
        anim  = anim_task;
        cal   = my_cal;
    }

    void updateTime(uint16_t hour, uint16_t min, uint16_t sec)
    {
        lv_img_set_angle(img_fig, (sec & 1) ? -25 : 25);
        lv_img_set_angle(img_hour, (hour % 12) * 300 + min * 5);
        lv_img_set_angle(img_min, min * 60 + sec);
        lv_img_set_angle(img_sec, sec * 60);
    }

    void updateWiFi(bool connected)
    {
        lv_label_set_static_text(lab_tl, (connected ? LV_SYMBOL_WIFI : ""));
    }

    void updateBattery(uint16_t level, uint16_t charging)
    {
        const char *txt_charge = (charging ? LV_SYMBOL_CHARGE : "");
        const char *txt_level = LV_SYMBOL_BATTERY_EMPTY;
        if (level >= 90)
            txt_level = LV_SYMBOL_BATTERY_FULL;
        else if (level >= 70)
            txt_level = LV_SYMBOL_BATTERY_3;
        else if (level >= 40)
            txt_level = LV_SYMBOL_BATTERY_2;
        else if (level >= 10)
            txt_level = LV_SYMBOL_BATTERY_1;
        lv_label_set_text_fmt(lab_tr, "%s %s ", txt_charge, txt_level);
    }

    void updateDate(uint16_t year, uint16_t month, uint16_t day, uint16_t weekday)
    {
        const char *name = "";
        if (weekday < 7)
            name = day_names[weekday];
        lv_label_set_text_fmt(lab_bl, "%s\n%d.%d.", name, day, month);

        if (cal)
            cal->update(year, month, day);
    }

    void updateSteps(uint32_t count)
    {
        lv_label_set_text_fmt(lab_br, "%d", count);
    }

    void startAnim(App *app)
    {
        if (anim)
        {
            anim->user_data = app;
            lv_task_set_prio(anim, LV_TASK_PRIO_LOWEST);
        }
    }

    void stopAnim()
    {
        if (anim)
            lv_task_set_prio(anim, LV_TASK_PRIO_OFF);
    }

    virtual void show()
    {
        stopAnim();

        // show app
        App::show();
    }

protected:
    lv_task_t *anim;
    CalendarApp *cal;

    // GUI
    lv_obj_t *img_bg, *img_fig, *img_hour, *img_min, *img_sec;
    lv_obj_t *lab_tl, *lab_tr, *lab_bl, *lab_br;
};

// ------------------------------------------------------------------------
// Call backs
// ------------------------------------------------------------------------

static WatchApp home;
static BatApp bat;
static CalendarApp cal;
static LevelApp level;
static App dummy;

void wifi_list_add(const char *ssid)
{
    MY_LOG("ssid: %s", ssid);
}

void wifi_connect_status(bool result)
{
    MY_LOG("wifi connect %d", result);
    home.updateWiFi(result);

    if (result)
        ntp_sync_time();
}

void updateBatteryCharge()
{
    MY_LOG("bat charging");
    uint16_t level = get_bat_level();
    home.updateBattery(level, 1);
}

void updateBatteryCalc()
{
    MY_LOG("bat calc");
    uint16_t level = get_bat_level();
    home.updateBattery(level, 0);
}

void updateBatteryLevel()
{
    uint16_t level = get_bat_level();
    uint16_t charging = get_bat_charging();
    MY_LOG("bat level %d%% %s", level, (charging ? "charging" : ""));
    home.updateBattery(level, charging);
}

void updateStepCounter(uint32_t count)
{
    MY_LOG("step count %d", count);
    home.updateSteps(count);
}

void updateDate(uint16_t year, uint16_t month, uint16_t day, uint16_t weekday)
{
    MY_LOG("Date %04d-%02d-%02d (%d)", year, month, day, weekday);
    home.updateDate(year, month, day, weekday);
}

void updateTime(uint16_t hour, uint16_t min, uint16_t sec)
{
    home.updateTime(hour, min, sec);
}

static void updateTask(lv_task_t *data)
{
    update_time(false);
}

static void batteryTask(lv_task_t *data)
{
    updateBatteryLevel();
}

static void animTask(lv_task_t *task)
{
    App *app = (App *)(task->user_data);
    if (!app)
    {
        MY_LOG("ANIM OFF");
        lv_task_set_prio(task, LV_TASK_PRIO_OFF);
        return;
    }

    app->anim();
}

static void startAnim(App *app)
{
    home.startAnim(app);
}


// ------------------------------------------------------------------------
// GUI launcher
// ------------------------------------------------------------------------

//  static bool showApp()
//  {
//      if (!app)
//          return false;
//  
//      MY_LOG("Back to app");
//      app->show();
//  }
//  
//  static void createApp(uint16_t id)
//  {
//      MY_LOG("New app");
//      lv_obj_t *screen = lv_obj_create(NULL, NULL);
//  
//      if (id == 3)
//          app = new CalendarApp();
//      else
//          app = new App();
//  
//      if (app)
//      {
//          app->create(screen);
//          app->show();
//      }
//      else
//          lv_obj_del(screen);
//  }
//
//  static void killApp(void *data)
//  {
//      MY_LOG("Kill app");
//      delete app;
//      app = NULL;
//  }

void showApp(uint16_t corner)
{
    MY_LOG("Show app %d", corner);

    switch (corner)
    {
    case 2:  bat.show(); break;
    case 3:  cal.show(); break;
    case 4:  level.show(); break;
    default: dummy.show(); break;
    }
}

void showHome()
{
    MY_LOG("Back to home");
    home.show();

//  if (app)
//      lv_async_call(killApp, NULL);
}

void setupGui()
{
    lv_task_t *anim = lv_task_create(animTask, 100, LV_TASK_PRIO_OFF, NULL);

    home.create(lv_scr_act());
    bat.create(NULL);
    cal.create(NULL);
    level.create(NULL);
    dummy.create(NULL);

    home.setup(anim, &cal);

    lv_task_create(updateTask,   1000, LV_TASK_PRIO_MID, NULL);
    lv_task_create(batteryTask, 30000, LV_TASK_PRIO_LOW, NULL);
}

// ------------------------------------------------------------------------
