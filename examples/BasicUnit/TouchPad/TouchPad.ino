#include "config.h"

TTGOClass *ttgo;

char buf[128];

void setup()
{
    ttgo = TTGOClass::getWatch();
    ttgo->begin();
    ttgo->openBL();

    TFT_eSPI *tft = ttgo->tft;
    int32_t w = tft->width();
    int32_t h = tft->height();
    int32_t r = (w > h ? h : w) / 2 - 1;    

    tft->fillScreen(TFT_BLACK);

    tft->drawLine(0,h/2,w,h/2,TFT_GREEN);
    tft->drawLine(w/2,0,w/2,h,TFT_GREEN);

    tft->drawCircle(w/2, h/2, r, TFT_RED);
    tft->drawCircle(1*w/4, 1*h/4, 6, TFT_BLUE);
    tft->drawCircle(3*w/4, 1*h/4, 6, TFT_BLUE);
    tft->drawCircle(1*w/4, 3*h/4, 6, TFT_BLUE);
    tft->drawCircle(3*w/4, 3*h/4, 6, TFT_BLUE);

    tft->setTextFont(2);
    tft->setTextColor(TFT_WHITE, TFT_BLACK);
    tft->drawString("T-Watch Touch Test", 60, 20);

    FT5206_Class *touch = ttgo->touch;
    if (touch) 
    {
        //touch->enterMonitorMode();
        //touch->adjustTheshold(200);
    }
}

void loop()
{
    // int16_t x, y;
    // if (ttgo->getTouch(x, y)) 
    // {
    //     sprintf(buf, "x:%03d  y:%03d", x, y);
    //     ttgo->tft->drawString(buf, 80, 200);
    // }

    FT5206_Class *touch = ttgo->touch;
    if (touch) 
    {
        int num = touch->touched();
        if (num > 0)
        {
            TP_Point p = touch->getPoint(0);

            int gesture = touch->gesture();
            const char *s = nullptr;
            switch (gesture)
            {
            case GESTURE_MOVE_UP:    s="move up     "; break;
            case GESTURE_MOVE_DOWN:  s="move down   "; break;
            case GESTURE_MOVE_LEFT:  s="move left   "; break;
            case GESTURE_MOVE_RIGHT: s="move right  "; break;
            case GESTURE_ZOOM_IN:    s="zoom in     "; break;
            case GESTURE_ZOOM_OUT:   s="zoom out    "; break;
//          default:                s="                 "; break;
            }
            if (s)
                ttgo->tft->drawString(s, 60, 40);

            ttgo->tft->drawNumber(num, 20, 200);

            sprintf(buf, "x:%03d  y:%03d  ", p.x, p.y);
            ttgo->tft->drawString(buf, 80, 200);

            if (num > 1)
            {
                p = touch->getPoint(1);
                sprintf(buf, "x:%03d  y:%03d  ", p.x, p.y);
            }
            ttgo->tft->drawString(num > 1 ? buf : "x:      y:    ", 80, 220);
        }
    }
    delay(5);
}
