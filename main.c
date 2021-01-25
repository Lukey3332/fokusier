#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <pthread.h>
#include <math.h>
#include <limits.h>
#include <time.h>
#include <stdbool.h>

#include <bluetooth/bluetooth.h>
#include "cwiid.h"

static struct {
        unsigned num_src;
        uint8_t battery;
        uint16_t buttons;
        bool recorded;
        struct cwiid_ir_src src;
} mydata;

static pthread_mutex_t mtx = PTHREAD_MUTEX_INITIALIZER;

#define do_abort(err, func) do { \
	fprintf(stderr, "%s: %s\n", func, strerror(err)); \
	abort(); \
} while(0)

static void mutex_lock(pthread_mutex_t *mtx)
{
	int err;

	err = pthread_mutex_lock(mtx);
	if (err)
		do_abort(err, __func__);
}

static void mutex_unlock(pthread_mutex_t *mtx)
{
	int err;

	err = pthread_mutex_unlock(mtx);
	if (err)
		do_abort(err, __func__);
}

float distance_u16(uint16_t *a, uint16_t *b)
{
        float x, y;

        x = (float)a[CWIID_X] - b[CWIID_X];
        y = (float)a[CWIID_Y] - b[CWIID_Y];
        return sqrtf(x*x+y*y);
}

float distance(unsigned *a, unsigned *b)
{
        float x, y;

        x = (float)a[CWIID_X] - b[CWIID_X];
        y = (float)a[CWIID_Y] - b[CWIID_Y];
        return sqrtf(x*x+y*y);
}

struct cwiid_ir_src *lowest_dist(struct cwiid_ir_src *src, uint16_t *last_pos)
{
        unsigned n;
        struct cwiid_ir_src *ptr;
        float dist, lowest_dist;

        lowest_dist = UINT_MAX;
        ptr = NULL;
        for (n=0; n < CWIID_IR_SRC_COUNT; n++) {
                if (src[n].valid && src[n].size <= 3) {
                        dist = distance_u16(last_pos, src[n].pos);
                        if (dist < lowest_dist) {
                                lowest_dist = dist;
                                ptr = &src[n];
                        }
                }
        }

        return ptr;
}

void cwiid_ir(cwiid_wiimote_t *wiimote, struct cwiid_ir_mesg *mesg)
{
        unsigned n, num_src;
        struct cwiid_ir_src *src = mesg->src;
        struct cwiid_ir_src *ptr = NULL;

        num_src = 0;
        for (n=0; n < CWIID_IR_SRC_COUNT; n++) {
                if (src[n].valid && src[n].size <= 3) {
                        num_src++;
                        ptr = &src[n];
                }
        }

        if (num_src > 1) {
                ptr = lowest_dist(src, mydata.src.pos);
        }

        if (!mydata.recorded) {
                mydata.num_src = num_src;
        }

        if (ptr) {
                memcpy(&mydata.src, ptr, sizeof(mydata.src));
                mydata.recorded = true;
                cwiid_set_rpt_mode(wiimote, CWIID_RPT_STATUS | CWIID_RPT_BTN);
        }
}

void cwiid_callback(cwiid_wiimote_t *wiimote, int mesg_count,
                    union cwiid_mesg *mesg_array, struct timespec *timestamp)
{
        unsigned n;
        struct cwiid_ir_mesg *ir_data;

        mutex_lock(&mtx);
        for (n=0; n < mesg_count; n++) {
                switch (mesg_array[n].type) {
                        case CWIID_MESG_STATUS:
                                mydata.battery = mesg_array[n].status_mesg.battery;
                                break;
                        case CWIID_MESG_IR:
                                cwiid_ir(wiimote, &mesg_array[n].ir_mesg);
                                break;
                        case CWIID_MESG_BTN:
                                mydata.buttons |= mesg_array[n].btn_mesg.buttons;
                                break;
                }
        }
        mutex_unlock(&mtx);
}

void mainloop(cwiid_wiimote_t *wiimote, unsigned ms_sample, unsigned ms_long,
              float tresh_short, float tresh_long,
              unsigned warn_sec, unsigned err_sec,
              unsigned override_sec, bool calibrate)
{
        uint16_t last_pos_short[2];
        unsigned avg_pos_long[2], last_pos_long[2];
        unsigned long_size, long_count;
        bool long_is_moving, is_moving, warned, recorded_last;
        float dist, max_dist;
        unsigned max_dotsize;
        time_t time_sec, last_move_time, override_time;
        struct tm time_fmt;

        recorded_last = false;
        avg_pos_long[0] = 0;
        avg_pos_long[1] = 0;
        last_pos_long[0] = 0;
        last_pos_long[1] = 0;
        long_size = ms_long / ms_sample;
        long_count = 0;
        long_is_moving = false;
        warned = false;
        max_dotsize = 0;
        dist = 0.0;
        max_dist = 0.0;
        last_move_time = time(NULL);
        override_time = 0;

        while (1) {
                is_moving = false;

                // 20ms timeout to detect something, preserve battery
                cwiid_set_rpt_mode(wiimote, CWIID_RPT_STATUS | CWIID_RPT_BTN | CWIID_RPT_IR);
                usleep(20*1000);
                cwiid_set_rpt_mode(wiimote, CWIID_RPT_STATUS | CWIID_RPT_BTN);
                usleep((ms_sample-20)*1000);
                mutex_lock(&mtx);

                if (mydata.recorded) {
                        mydata.recorded = false;

                        if (recorded_last) {
                                dist = distance_u16(mydata.src.pos, last_pos_short);
                                if (dist > tresh_short) {
                                        is_moving = true;
                                }

                                if (!long_is_moving && dist > tresh_long) {
                                        is_moving = false;
                                        long_is_moving = true;
                                        avg_pos_long[CWIID_X] = 0;
                                        avg_pos_long[CWIID_Y] = 0;
                                        long_count = 0;
                                }
                        }

                        avg_pos_long[CWIID_X] += mydata.src.pos[CWIID_X];
                        avg_pos_long[CWIID_Y] += mydata.src.pos[CWIID_Y];
                        long_count++;

                        memcpy(last_pos_short, mydata.src.pos, sizeof(last_pos_short));
                        recorded_last = true;
                }

                if (long_count >= long_size) {
                        avg_pos_long[CWIID_X] /= long_count;
                        avg_pos_long[CWIID_Y] /= long_count;
                        if (distance(avg_pos_long, last_pos_long) > tresh_long) {
                                long_is_moving = true;
                        } else {
                                long_is_moving = false;
                        }
                        memcpy(last_pos_long, avg_pos_long, sizeof(last_pos_long));

                        avg_pos_long[CWIID_X] = 0;
                        avg_pos_long[CWIID_Y] = 0;
                        long_count = 0;
                }

                time_sec = time(NULL);
                localtime_r(&time_sec, &time_fmt);

                if (mydata.buttons & CWIID_BTN_A) {
                        mydata.buttons = 0;
                        override_time = time_sec;
                }

                if ((is_moving && long_is_moving) || time_sec - override_time < override_sec) {
                        last_move_time = time_sec;
                        warned = false;
                }

                if (!warned && time_sec - last_move_time > warn_sec) {
                        warned = true;
                        mutex_unlock(&mtx);
                        system("aplay --quiet warn.wav");
                        mutex_lock(&mtx);
                }

                if (time_sec - last_move_time > err_sec) {
                        mutex_unlock(&mtx);
                        system("aplay --quiet warn.wav");
                        mutex_lock(&mtx);
                }

#define NOTHING "                 "
#define OVERRID "override         "
#define WARNING "warning          "
#define ERROR   "get back to work!"
                if (calibrate) {
                        if (dist > max_dist) {
                                max_dist = dist;
                        }
                        if (mydata.src.size > max_dotsize) {
                                max_dotsize = mydata.src.size;
                        }
                        printf("\r[%02u:%02u] %3d%% %u %c X:%4u Y:%4u D:%3.0f MD:%3.0f S:%d MS:%d",
                               time_fmt.tm_hour, time_fmt.tm_min,
                               (int) (100.0 * mydata.battery / CWIID_BATTERY_MAX),
                               mydata.num_src, (is_moving && long_is_moving ? 'M' : ' '),
                               mydata.src.pos[CWIID_X], mydata.src.pos[CWIID_Y],
                               dist, max_dist,
                               mydata.src.size, max_dotsize);
                } else {
                        printf("\r[%02u:%02u] %3d%% %u %s",
                               time_fmt.tm_hour, time_fmt.tm_min,
                               (int) (100.0 * mydata.battery / CWIID_BATTERY_MAX),
                               mydata.num_src,
                               (time_sec - override_time < override_sec ? OVERRID :
                                (time_sec - last_move_time > err_sec ? ERROR :
                                 (warned ? WARNING : NOTHING))));
                }
                fflush(stdout);
                mutex_unlock(&mtx);
        }
}

int main (int argc, char **argv)
{
        bdaddr_t *bdaddr = BDADDR_ANY;
        cwiid_wiimote_t *wiimote = NULL;

        mydata.recorded = false;

        puts("Put Wiimote in discoverable mode (press 1+2) before starting");
        if ((wiimote = cwiid_open(bdaddr, CWIID_FLAG_MESG_IFC)) == NULL) {
                puts("Unable to connect");
                exit(1);
        }
        if (cwiid_set_mesg_callback(wiimote, &cwiid_callback)) {
                puts("Error setting callback");
                exit(1);
        }
        cwiid_set_rpt_mode(wiimote, CWIID_RPT_STATUS | CWIID_RPT_BTN | CWIID_RPT_IR);

        mainloop(wiimote, 100, 2*1000, 5, 30, 60, 120, 300, false);
}
