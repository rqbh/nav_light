
#include "driver/ledc.h"
#include "driver/rmt.h"
#include "driver/timer.h"
#include "esp_log.h"
#include "iot_button.h"
#include "iot_led.h"
#include "led_strip.h"
#include "light_driver.h"
#include "light_handle.h"
#include "math.h"
#include "mconfig_blufi.h"
#include "mconfig_chain.h"
#include "mdebug_console.h"
#include "mdebug_log.h"
#include "mdf_common.h"
#include "mesh_utils.h"
#include "mespnow.h"
#include "mlink.h"
#include "mupgrade.h"
#include "mwifi.h"
#include "soc/ledc_reg.h"
#include "soc/ledc_struct.h"
#include "soc/timer_group_struct.h"

#define TAG "NAV LIGHT"

#define RMT_DEFAULT_CONFIG_TX( gpio, channel_id )                                                                           \
    {                                                                                                                       \
        .rmt_mode = RMT_MODE_TX, .channel = channel_id, .gpio_num = gpio, .clk_div = 80, .mem_block_num = 1, .tx_config = { \
            .carrier_freq_hz      = 38000,                                                                                  \
            .carrier_level        = RMT_CARRIER_LEVEL_HIGH,                                                                 \
            .idle_level           = RMT_IDLE_LEVEL_LOW,                                                                     \
            .carrier_duty_percent = 33,                                                                                     \
            .carrier_en           = false,                                                                                  \
            .loop_en              = false,                                                                                  \
            .idle_output_en       = true,                                                                                   \
        }                                                                                                                   \
    }

#define CONFIG_WHITE_LED_STRIP_NUM 60
#define CONFIG_WHITE_LED_STRIP_GPIO 12
#define CONFIG_WHITE_LED_STRIP_CHANNEL RMT_CHANNEL_0

#define CONFIG_LEFT_LED_STRIP_NUM 60
#define CONFIG_LEFT_LED_STRIP_GPIO 13
#define CONFIG_LEFT_LED_STRIP_CHANNEL RMT_CHANNEL_1

#define CONFIG_RIGHT_LED_STRIP_NUM 60
#define CONFIG_RIGHT_LED_STRIP_GPIO 15
#define CONFIG_RIGHT_LED_STRIP_CHANNEL RMT_CHANNEL_2

#define NAV_RUN_TIME ( 60 * 5 * 1000 / portTICK_PERIOD_MS )  // 30s

#define BUTTON_REL_BIT BIT0
#define BUTTON_REL_BIT_0 BIT1

static sys_mutex_t g_led_strip_left_mutex  = NULL;
static sys_mutex_t g_led_strip_right_mutex = NULL;

static EventGroupHandle_t g_led_event_group = NULL;

static ws2812_t *strip_white, *strip_left, *strip_right;

typedef struct strip_nav {
    char*             nav_id;
    uint8_t           r;
    uint8_t           g;
    uint8_t           b;
    bool              dir;
    portTickType      start;
    struct strip_nav* pre;
    struct strip_nav* next;
} strip_nav_t;

static strip_nav_t nav_head = { .start = 0, .r = 0, .g = 0, .b = 0, .next = &nav_head, .pre = &nav_head }, *p_nav_head = &nav_head;

static void add_nav( strip_nav_t* nav )
{
    strip_nav_t* p = p_nav_head;
    for ( ;; ) {
        if ( p->next == p_nav_head && p->pre == p_nav_head && p->start == 0 ) {
            p->dir   = nav->dir;
            p->r     = nav->r;
            p->g     = nav->g;
            p->b     = nav->b;
            p->start = xTaskGetTickCount();
            free( nav );
            break;
        }
        else if ( p->next == p_nav_head ) {
            nav->pre  = p;
            nav->next = p->next;

            p->next->pre = nav;
            p->next      = nav;

            nav->start = xTaskGetTickCount();
            ESP_LOGI( TAG, "new nav added" );
            break;
        }
        else {
            p = p->next;
        }
    }
}

static void del_nav( strip_nav_t* nav )
{
    strip_nav_t* pre_nav  = nav->pre;
    strip_nav_t* next_nav = nav->next;

    if ( nav != p_nav_head ) {
        xSemaphoreTakeRecursive( g_led_strip_left_mutex, portMAX_DELAY );
        pre_nav->next = next_nav;
        next_nav->pre = pre_nav;
        xSemaphoreGiveRecursive( g_led_strip_left_mutex );
        ESP_LOGI( TAG, "nav del" );

        free( nav );
    }
    else {  // p_nav_head
        nav->start = 0;
        nav->r     = 0xff;
        nav->g     = 0xff;
        nav->b     = 0xff;
    }
}

static int get_nav_count( bool dir )
{
    strip_nav_t* nav = &nav_head;
    int          i   = 0;

    for ( ;; ) {
        if ( nav->dir == dir && nav->start != 0 )
            i++;
        if ( nav->next == &nav_head )  // last one
            break;
        nav = nav->next;
    }
    return i;
}

static strip_nav_t* next_nav( bool from_start, bool dir )
{
    static strip_nav_t* current_nav;
    strip_nav_t *       p    = NULL, *tmp;
    portTickType        tick = xTaskGetTickCount();

    int cnt = get_nav_count( dir );
    if ( cnt == 0 )
        return NULL;

    if ( !current_nav || from_start )
        current_nav = p_nav_head;

    for ( int i = 0; i < cnt + get_nav_count( !dir ); i++ ) {
        if ( current_nav->dir == dir && current_nav->start != 0 && tick < current_nav->start + NAV_RUN_TIME ) {
            p = current_nav;
            break;
        }
        else if ( tick > current_nav->start + NAV_RUN_TIME )  // check if should delete
        {
            tmp         = current_nav;
            current_nav = current_nav->next;
            del_nav( tmp );
        }
        else
            current_nav = current_nav->next;
    }

    // for next time
    current_nav = current_nav->next;

    return p;
}

static bool look_nav_by_id( char* id, strip_nav_t** nav )
{
    int          i;
    strip_nav_t* pnav = p_nav_head;

    if ( pnav->next == pnav ) {
        if ( pnav->nav_id ) {
            if ( !strcmp( pnav->nav_id, id ) )
                *nav = pnav;
            return true;
        }
        if ( pnav->start == 0 ) {
            pnav->nav_id = id;
            *nav         = pnav;
            return true;
        }
    }
    for ( ; pnav->next != p_nav_head; pnav = pnav->next ) {
        if ( pnav->nav_id ) {
            if ( !strcmp( pnav->nav_id, id ) ) {
                *nav = pnav;
                return true;
                break;
            }
        }
    }
    return false;
}

static void btn_rel_cb( void* arg )
{
    xEventGroupSetBits( g_led_event_group, BUTTON_REL_BIT );
}

static void btn_rel_cb_io0( void* arg )
{
    xEventGroupSetBits( g_led_event_group, BUTTON_REL_BIT_0 );
}

static void register_btn( void* arg )
{
    button_handle_t btn;
    button_config_t config = {
        .type = BUTTON_TYPE_GPIO,
    };
    config.gpio_button_config.gpio_num     = 35;
    config.gpio_button_config.active_level = 0;

    btn = iot_button_create( &config );
    iot_button_register_cb( btn, BUTTON_PRESS_UP, btn_rel_cb );

    config.gpio_button_config.gpio_num = 0;
    btn                                = iot_button_create( &config );
    iot_button_register_cb( btn, BUTTON_PRESS_UP, btn_rel_cb_io0 );
}

static void check_btn_event()
{
    uint32_t bits = xEventGroupGetBits( g_led_event_group );
    xEventGroupClearBits( g_led_event_group, BUTTON_REL_BIT | BUTTON_REL_BIT_0 );
    if ( bits & BUTTON_REL_BIT ) {
        strip_nav_t* nav = malloc( sizeof( strip_nav_t ) );
        if ( nav == NULL ) {
            ESP_LOGW( TAG, "failed malloc mem for nav" );
            return;
        }
        nav->r = esp_random() & 0x01 ? 0xFF : 0;
        nav->g = esp_random() & 0x01 ? 0xFF : 0;
        nav->b = esp_random() & 0x01 ? 0xFF : 0;
        if ( nav->r == 0 && nav->g == 0 && nav->b == 0 ) {
            nav->g = 0xff;
            nav->r = 0xff;
        }

        nav->dir = true;

        add_nav( nav );
    }
    if ( bits & BUTTON_REL_BIT_0 ) {
        strip_nav_t* nav = malloc( sizeof( strip_nav_t ) );
        if ( nav == NULL ) {
            ESP_LOGW( TAG, "failed malloc mem for nav" );
            return;
        }
        nav->r = esp_random() & 0x01 ? 0xFF : 0;
        nav->g = esp_random() & 0x01 ? 0xFF : 0;
        nav->b = esp_random() & 0x01 ? 0xFF : 0;
        if ( nav->r == 0 && nav->g == 0 && nav->b == 0 ) {
            nav->g = 0xff;
            nav->r = 0xff;
        }

        nav->dir = false;

        add_nav( nav );
    }
}

void led_strip_task( void* arg )
{
    bool strip_left_need_update, strip_right_need_update;

    g_led_strip_left_mutex  = xSemaphoreCreateMutex();
    g_led_strip_right_mutex = xSemaphoreCreateMutex();
    g_led_event_group       = xEventGroupCreate();

    register_btn( NULL );

    // create strip white
    rmt_config_t config = RMT_DEFAULT_CONFIG_TX( CONFIG_WHITE_LED_STRIP_GPIO, CONFIG_WHITE_LED_STRIP_CHANNEL );
    // set counter clock to 40MHz
    config.clk_div = 2;
    ESP_ERROR_CHECK( rmt_config( &config ) );
    ESP_ERROR_CHECK( rmt_driver_install( config.channel, 0, 0 ) );
    led_strip_config_t strip_config = LED_STRIP_DEFAULT_CONFIG( CONFIG_WHITE_LED_STRIP_NUM, ( led_strip_dev_t )config.channel );
    strip_white                     = ( ws2812_t* )led_strip_new_rmt_ws2812( &strip_config );
    MDF_ERROR_ASSERT( strip_white != NULL ? MDF_OK : MDF_FAIL );
    strip_white->parent.clear( &( strip_white->parent ), 100 );
    for ( int i = 0; i < strip_white->strip_len; i++ ) {
        strip_white->parent.set_pixel( &( strip_white->parent ), i, 0xFF, 0xFF, 0xFF );
    }
    ESP_LOGI( TAG, "strip set_pixel" );

    // create left color strip
    rmt_config_t config_left = RMT_DEFAULT_CONFIG_TX( CONFIG_LEFT_LED_STRIP_GPIO, CONFIG_LEFT_LED_STRIP_CHANNEL );
    config_left.clk_div      = 2;
    ESP_ERROR_CHECK( rmt_config( &config_left ) );
    ESP_ERROR_CHECK( rmt_driver_install( config_left.channel, 0, 0 ) );
    led_strip_config_t strip_config_left = LED_STRIP_DEFAULT_CONFIG( CONFIG_LEFT_LED_STRIP_NUM, ( led_strip_dev_t )config_left.channel );
    strip_left                           = ( ws2812_t* )led_strip_new_rmt_ws2812( &strip_config_left );
    MDF_ERROR_ASSERT( strip_left != NULL ? MDF_OK : MDF_FAIL );
    strip_left->parent.clear( &( strip_left->parent ), 100 );
    ESP_LOGI( TAG, "strip left created" );

    // create right color strip
    rmt_config_t config_right = RMT_DEFAULT_CONFIG_TX( CONFIG_RIGHT_LED_STRIP_GPIO, CONFIG_RIGHT_LED_STRIP_CHANNEL );
    config_right.clk_div      = 2;
    ESP_ERROR_CHECK( rmt_config( &config_right ) );
    ESP_ERROR_CHECK( rmt_driver_install( config_right.channel, 0, 0 ) );
    led_strip_config_t strip_config_right = LED_STRIP_DEFAULT_CONFIG( CONFIG_RIGHT_LED_STRIP_NUM, ( led_strip_dev_t )config_right.channel );
    strip_right                           = ( ws2812_t* )led_strip_new_rmt_ws2812( &strip_config_right );
    MDF_ERROR_ASSERT( strip_right != NULL ? MDF_OK : MDF_FAIL );
    strip_right->parent.clear( &( strip_right->parent ), 100 );
    ESP_LOGI( TAG, "strip right created" );

    strip_left->parent.refresh( &( strip_left->parent ), portMAX_DELAY );
    strip_right->parent.refresh( &( strip_right->parent ), portMAX_DELAY );

    while ( true ) {
        static int   start_led_left = 0, start_led_right = 0;
        strip_nav_t* nav;

        // each time ,move one led
        start_led_left++;
        if ( start_led_left == strip_left->strip_len ) {
            start_led_left = 0;  // next circle
        }

        start_led_right--;
        if ( start_led_right < 0 ) {
            start_led_right = strip_right->strip_len - 1;  // next circle
        }

        // left led strip, forward
        xSemaphoreTakeRecursive( g_led_strip_left_mutex, portMAX_DELAY );
        nav = next_nav( true, true );

        int        left_nav_cnt = get_nav_count( true ), left_nav_led_cnt, left_nav_led_space_cnt;
        static int left_clr_cnt = 0;
        if ( left_nav_cnt == 0 ) {
            if ( left_clr_cnt < 1 ) {
                strip_left->parent.clear( &( strip_left->parent ), portMAX_DELAY );
                strip_left->parent.refresh( &( strip_left->parent ), portMAX_DELAY );
                left_clr_cnt++;
                if ( left_clr_cnt > 10 )
                    left_clr_cnt = 10;
            }
        }
        else {
            int tmp;
            tmp                    = strip_left->strip_len / left_nav_cnt;
            left_nav_led_cnt       = tmp * 3 / 4;
            left_nav_led_space_cnt = tmp / 4;

            if ( left_nav_led_cnt < 1 )
                left_nav_led_cnt = 1;
            if ( left_nav_led_cnt > 8 )
                left_nav_led_cnt = 8;
            if ( left_nav_led_space_cnt < 1 )
                left_nav_led_space_cnt = 1;
            if ( left_nav_led_space_cnt > 8 )
                left_nav_led_space_cnt = 8;

            for ( int i = 0, index_led = start_led_left; i < strip_left->strip_len; ) {
                // from start_led
                if ( nav ) {
                    for ( int j = 0; j < left_nav_led_cnt; j++ ) {
                        if ( i == strip_left->strip_len )
                            break;
                        strip_left->parent.set_pixel( &( strip_left->parent ), ( index_led++ ) % strip_left->strip_len, nav->r, nav->g, nav->b );
                        i++;
                    }
                }
                for ( int j = 0; j < left_nav_led_space_cnt; j++ ) {
                    if ( i == strip_left->strip_len )
                        break;
                    strip_left->parent.set_pixel( &( strip_left->parent ), ( index_led++ ) % strip_left->strip_len, 0, 0, 0 );
                    i++;
                }

                nav = next_nav( false, true );
            }
            left_clr_cnt = 0;
        }
        xSemaphoreGiveRecursive( g_led_strip_left_mutex );
        strip_left->parent.refresh( &( strip_left->parent ), portMAX_DELAY );

        // right strip, backward
        xSemaphoreTakeRecursive( g_led_strip_right_mutex, portMAX_DELAY );
        nav = next_nav( true, false );

        int        right_nav_cnt = get_nav_count( false ), right_nav_led_cnt, right_nav_led_space_cnt;
        static int right_clr_cnt = 0;
        if ( right_nav_cnt == 0 ) {
            if ( right_clr_cnt < 1 ) {
                strip_right->parent.clear( &( strip_right->parent ), portMAX_DELAY );
                strip_right->parent.refresh( &( strip_right->parent ), portMAX_DELAY );
                right_clr_cnt++;
                if ( right_clr_cnt > 10 )
                    right_clr_cnt = 10;
            }
        }
        else {
            int tmp;
            tmp                     = strip_right->strip_len / right_nav_cnt;
            right_nav_led_cnt       = tmp * 3 / 4;
            right_nav_led_space_cnt = tmp / 4;

            if ( right_nav_led_cnt < 1 )
                right_nav_led_cnt = 1;
            if ( right_nav_led_cnt > 8 )
                right_nav_led_cnt = 8;
            if ( right_nav_led_space_cnt < 1 )
                right_nav_led_space_cnt = 1;
            if ( right_nav_led_space_cnt > 8 )
                right_nav_led_space_cnt = 8;
            for ( int i = 0, index_led = start_led_right; i < strip_right->strip_len; ) {
                // from start_led
                if ( nav ) {
                    for ( int j = 0; j < right_nav_led_cnt; j++ ) {
                        if ( i == strip_right->strip_len )
                            break;
                        strip_right->parent.set_pixel( &( strip_right->parent ), ( ( index_led-- ) % strip_right->strip_len ), nav->r, nav->g, nav->b );
                        i++;
                        if ( index_led < 0 )
                            index_led = strip_right->strip_len - 1;
                    }
                }
                for ( int j = 0; j < right_nav_led_space_cnt; j++ ) {
                    if ( i == strip_right->strip_len )
                        break;
                    strip_right->parent.set_pixel( &( strip_right->parent ), ( ( index_led-- ) % strip_right->strip_len ), 0, 0, 0 );
                    i++;
                    if ( index_led < 0 )
                        index_led = strip_right->strip_len - 1;
                }
                nav = next_nav( false, false );
            }
            xSemaphoreGiveRecursive( g_led_strip_right_mutex );
            strip_right->parent.refresh( &( strip_right->parent ), portMAX_DELAY );
            right_clr_cnt = 0;
        }
        vTaskDelay( 30 / portTICK_PERIOD_MS );

        check_btn_event();
    }
}

static mdf_err_t led_strip_add_nav( char* str )
{
    strip_nav_t* nav = malloc( sizeof( strip_nav_t ) );
    if ( !nav ) {
        ESP_LOGW( TAG, "failed alloc mem for new nav" );
        return MDF_ERR_NO_MEM;
    }

    char* p = str;
    if ( *p != '0' && *p != '1' ) {
        ESP_LOGW( TAG, "recv wrong data format for nav data" );
        return MDF_ERR_INVALID_ARG;
    }
    if ( *p == '0' )
        nav->dir = 0;
    else
        nav->dir = 1;
    p++;
    if ( *p == ',' ) {
        uint8_t r, g, b;
        if ( 1 < scanf( p, ",%d,%d,%d", &r, &g, &b ) ) {
            nav->r = r;
            nav->g = g;
            nav->b = b;

            add_nav( nav );
            return MDF_OK;
        }
    }
    return MDF_FAIL;
}

#define MAX_NAV_STRING_LEN 128
static mdf_err_t led_strip_get_nav( char** str )
{
    static char buff[ MAX_NAV_STRING_LEN ];

    memset( buff, 0, sizeof( MAX_NAV_STRING_LEN ) );

    *str = buff;

    return MDF_OK;
}

#define LEDC_FADE_MARGIN ( 10 )
#define LEDC_VALUE_TO_DUTY( value ) ( value * ( ( 1 << LEDC_TIMER_13_BIT ) - 1 ) / UINT16_MAX )
#define LEDC_DUTY_TO_VALUE( value ) ( value * UINT16_MAX / ( ( 1 << LEDC_TIMER_13_BIT ) - 1 ) )
#define LEDC_FIXED_Q ( 8 )
#define FLOATINT_2_FIXED( X, Q ) ( ( int )( ( X ) * ( 0x1U << Q ) ) )
#define FIXED_2_FLOATING( X, Q ) ( ( int )( ( X ) / ( 0x1U << Q ) ) )
#define GET_FIXED_INTEGER_PART( X, Q ) ( X >> Q )
#define GET_FIXED_DECIMAL_PART( X, Q ) ( X & ( ( 0x1U << Q ) - 1 ) )

typedef struct {
    int    cur;
    int    final;
    int    step;
    int    cycle;
    size_t num;
} ledc_fade_data_t;

typedef struct {
    timer_group_t timer_group;
    timer_idx_t   timer_id;
} hw_timer_idx_t;

typedef struct {
    ledc_fade_data_t fade_data[ LEDC_CHANNEL_MAX ];
    ledc_mode_t      speed_mode;
    ledc_timer_t     timer_num;
    hw_timer_idx_t   timer_id;
} iot_light_t;

static DRAM_ATTR iot_light_t* g_light_config  = NULL;
static DRAM_ATTR uint16_t* g_gamma_table      = NULL;
static DRAM_ATTR bool      g_hw_timer_started = false;
static DRAM_ATTR timg_dev_t* TG[ 2 ]          = { &TIMERG0, &TIMERG1 };
#if 0
static IRAM_ATTR esp_err_t _timer_pause(timer_group_t group_num, timer_idx_t timer_num)
{
    TG[group_num]->hw_timer[timer_num].config.enable = 0;
    return ESP_OK;
}

static void iot_timer_create(hw_timer_idx_t *timer_id, bool auto_reload,
                             uint32_t timer_interval_ms, void *isr_handle)
{
    /* Select and initialize basic parameters of the timer */
    timer_config_t config;
    config.divider = HW_TIMER_DIVIDER;
    config.counter_dir = TIMER_COUNT_UP;
    config.counter_en = TIMER_PAUSE;
    config.alarm_en = TIMER_ALARM_EN;
    config.intr_type = TIMER_INTR_LEVEL;
    config.auto_reload = auto_reload;
    timer_init(timer_id->timer_group, timer_id->timer_id, &config);

    /* Timer's counter will initially start from value below.
       Also, if auto_reload is set, this value will be automatically reload on alarm */
    timer_set_counter_value(timer_id->timer_group, timer_id->timer_id, 0x00000000ULL);

    /* Configure the alarm value and the interrupt on alarm. */
    timer_set_alarm_value(timer_id->timer_group, timer_id->timer_id, timer_interval_ms * HW_TIMER_SCALE / 1000);
    timer_enable_intr(timer_id->timer_group, timer_id->timer_id);
    timer_isr_register(timer_id->timer_group, timer_id->timer_id, isr_handle,
                       (void *)timer_id->timer_id, ESP_INTR_FLAG_IRAM, NULL);
}

static void iot_timer_start(hw_timer_idx_t *timer_id)
{
    timer_start(timer_id->timer_group, timer_id->timer_id);
    g_hw_timer_started = true;
}

static IRAM_ATTR void iot_timer_stop(hw_timer_idx_t *timer_id)
{
    _timer_pause(timer_id->timer_group, timer_id->timer_id);
    g_hw_timer_started = false;
}

static IRAM_ATTR esp_err_t iot_ledc_duty_config(ledc_mode_t speed_mode, ledc_channel_t channel, int hpoint_val, int duty_val,
                                                uint32_t duty_direction, uint32_t duty_num, uint32_t duty_cycle, uint32_t duty_scale)
{
    if (hpoint_val >= 0)
    {
        LEDC.channel_group[speed_mode].channel[channel].hpoint.hpoint = hpoint_val & LEDC_HPOINT_HSCH1_V;
    }

    if (duty_val >= 0)
    {
        LEDC.channel_group[speed_mode].channel[channel].duty.duty = duty_val;
    }

    LEDC.channel_group[speed_mode].channel[channel].conf1.val = ((duty_direction & LEDC_DUTY_INC_HSCH0_V) << LEDC_DUTY_INC_HSCH0_S) |
                                                                ((duty_num & LEDC_DUTY_NUM_HSCH0_V) << LEDC_DUTY_NUM_HSCH0_S) |
                                                                ((duty_cycle & LEDC_DUTY_CYCLE_HSCH0_V) << LEDC_DUTY_CYCLE_HSCH0_S) |
                                                                ((duty_scale & LEDC_DUTY_SCALE_HSCH0_V) << LEDC_DUTY_SCALE_HSCH0_S);

    LEDC.channel_group[speed_mode].channel[channel].conf0.sig_out_en = 1;
    LEDC.channel_group[speed_mode].channel[channel].conf1.duty_start = 1;

    if (speed_mode == LEDC_LOW_SPEED_MODE)
    {
        LEDC.channel_group[speed_mode].channel[channel].conf0.low_speed_update = 1;
    }

    return ESP_OK;
}

static IRAM_ATTR esp_err_t _iot_set_fade_with_step(ledc_mode_t speed_mode, ledc_channel_t channel, uint32_t target_duty, int scale, int cycle_num)
{
    uint32_t duty_cur = LEDC.channel_group[speed_mode].channel[channel].duty_rd.duty_read >> 4;
    int step_num = 0;
    int dir = LEDC_DUTY_DIR_DECREASE;

    if (scale > 0)
    {
        if (duty_cur > target_duty)
        {
            step_num = (duty_cur - target_duty) / scale;
            step_num = step_num > 1023 ? 1023 : step_num;
            scale = (step_num == 1023) ? (duty_cur - target_duty) / step_num : scale;
        }
        else
        {
            dir = LEDC_DUTY_DIR_INCREASE;
            step_num = (target_duty - duty_cur) / scale;
            step_num = step_num > 1023 ? 1023 : step_num;
            scale = (step_num == 1023) ? (target_duty - duty_cur) / step_num : scale;
        }
    }

    if (scale > 0 && step_num > 0)
    {
        iot_ledc_duty_config(speed_mode, channel, -1, duty_cur << 4, dir, step_num, cycle_num, scale);
    }
    else
    {
        iot_ledc_duty_config(speed_mode, channel, -1, target_duty << 4, dir, 0, 1, 0);
    }

    return ESP_OK;
}

static IRAM_ATTR esp_err_t _iot_set_fade_with_time(ledc_mode_t speed_mode, ledc_channel_t channel, uint32_t target_duty, int max_fade_time_ms)
{
    uint32_t freq = 0;
    uint32_t duty_cur = LEDC.channel_group[speed_mode].channel[channel].duty_rd.duty_read >> 4;
    uint32_t duty_delta = target_duty > duty_cur ? target_duty - duty_cur : duty_cur - target_duty;

    uint32_t timer_source_clk = LEDC.timer_group[speed_mode].timer[g_light_config->timer_num].conf.tick_sel;
    uint32_t duty_resolution = LEDC.timer_group[speed_mode].timer[g_light_config->timer_num].conf.duty_resolution;
    uint32_t clock_divider = LEDC.timer_group[speed_mode].timer[g_light_config->timer_num].conf.clock_divider;
    uint32_t precision = (0x1U << duty_resolution);

    if (timer_source_clk == LEDC_APB_CLK)
    {
        freq = ((uint64_t)LEDC_APB_CLK_HZ << 8) / precision / clock_divider;
    }
    else
    {
        freq = ((uint64_t)LEDC_REF_CLK_HZ << 8) / precision / clock_divider;
    }

    if (duty_delta == 0)
    {
        return _iot_set_fade_with_step(speed_mode, channel, target_duty, 0, 0);
    }

    int total_cycles = max_fade_time_ms * freq / 1000;

    if (total_cycles == 0)
    {
        return _iot_set_fade_with_step(speed_mode, channel, target_duty, 0, 0);
    }

    int scale, cycle_num;

    if (total_cycles > duty_delta)
    {
        scale = 1;
        cycle_num = total_cycles / duty_delta;

        if (cycle_num > LEDC_DUTY_NUM_HSCH0_V)
        {
            cycle_num = LEDC_DUTY_NUM_HSCH0_V;
        }
    }
    else
    {
        cycle_num = 1;
        scale = duty_delta / total_cycles;

        if (scale > LEDC_DUTY_SCALE_HSCH0_V)
        {
            scale = LEDC_DUTY_SCALE_HSCH0_V;
        }
    }

    return _iot_set_fade_with_step(speed_mode, channel, target_duty, scale, cycle_num);
}

static IRAM_ATTR esp_err_t _iot_update_duty(ledc_mode_t speed_mode, ledc_channel_t channel)
{
    LEDC.channel_group[speed_mode].channel[channel].conf0.sig_out_en = 1;
    LEDC.channel_group[speed_mode].channel[channel].conf1.duty_start = 1;

    if (speed_mode == LEDC_LOW_SPEED_MODE)
    {
        LEDC.channel_group[speed_mode].channel[channel].conf0.low_speed_update = 1;
    }

    return ESP_OK;
}

static IRAM_ATTR esp_err_t iot_ledc_set_duty(ledc_mode_t speed_mode, ledc_channel_t channel, uint32_t duty)
{
    return iot_ledc_duty_config(speed_mode,
                                channel, // uint32_t chan_num,
                                -1,
                                duty << 4, // uint32_t duty_val,the least 4 bits are decimal part
                                1,         // uint32_t increase,
                                1,         // uint32_t duty_num,
                                1,         // uint32_t duty_cycle,
                                0          // uint32_t duty_scale
    );
}

static void gamma_table_create(uint16_t *gamma_table, float correction)
{
    float value_tmp = 0;

    /**
     * @brief gamma curve formula: y=a*x^(1/gm)
     * x ∈ (0,(GAMMA_TABLE_SIZE-1)/GAMMA_TABLE_SIZE)
     * a = GAMMA_TABLE_SIZE
     */
    for (int i = 0; i < GAMMA_TABLE_SIZE; i++)
    {
        value_tmp = (float)(i) / GAMMA_TABLE_SIZE;
        value_tmp = powf(value_tmp, 1.0f / correction);
        gamma_table[i] = (uint16_t)FLOATINT_2_FIXED((value_tmp * GAMMA_TABLE_SIZE), LEDC_FIXED_Q);
    }

    if (gamma_table[255] == 0)
    {
        gamma_table[255] = __UINT16_MAX__;
    }
}

static IRAM_ATTR uint32_t gamma_value_to_duty(int value)
{
    uint32_t tmp_q = GET_FIXED_INTEGER_PART(value, LEDC_FIXED_Q);
    uint32_t tmp_r = GET_FIXED_DECIMAL_PART(value, LEDC_FIXED_Q);

    uint16_t cur = LEDC_VALUE_TO_DUTY(g_gamma_table[tmp_q]);
    uint16_t next = LEDC_VALUE_TO_DUTY(g_gamma_table[tmp_q + 1]);
    return (cur + (next - cur) * tmp_r / (0x1U << LEDC_FIXED_Q));
}

static IRAM_ATTR void fade_timercb(void *para)
{
    int timer_idx = (int)para;
    int idle_channel_num = 0;

    if (HW_TIMER_GROUP == TIMER_GROUP_0)
    {
        /* Retrieve the interrupt status */
        uint32_t intr_status = TIMERG0.int_st_timers.val;
        TIMERG0.hw_timer[timer_idx].update = 1;

        /* Clear the interrupt */
        if ((intr_status & BIT(timer_idx)) && timer_idx == TIMER_0)
        {
            TIMERG0.int_clr_timers.t0 = 1;
        }
        else if ((intr_status & BIT(timer_idx)) && timer_idx == TIMER_1)
        {
            TIMERG0.int_clr_timers.t1 = 1;
        }

        /* After the alarm has been triggered
          we need enable it again, so it is triggered the next time */
        TIMERG0.hw_timer[timer_idx].config.alarm_en = TIMER_ALARM_EN;
    }
    else if (HW_TIMER_GROUP == TIMER_GROUP_1)
    {
        uint32_t intr_status = TIMERG1.int_st_timers.val;
        TIMERG1.hw_timer[timer_idx].update = 1;

        if ((intr_status & BIT(timer_idx)) && timer_idx == TIMER_0)
        {
            TIMERG1.int_clr_timers.t0 = 1;
        }
        else if ((intr_status & BIT(timer_idx)) && timer_idx == TIMER_1)
        {
            TIMERG1.int_clr_timers.t1 = 1;
        }

        TIMERG1.hw_timer[timer_idx].config.alarm_en = TIMER_ALARM_EN;
    }

    for (int channel = 0; channel < LEDC_CHANNEL_MAX; channel++)
    {
        ledc_fade_data_t *fade_data = g_light_config->fade_data + channel;

        if (fade_data->num > 0)
        {
            fade_data->num--;

            if (fade_data->step)
            {
                fade_data->cur += fade_data->step;

                if (fade_data->num != 0)
                {
                    _iot_set_fade_with_time(g_light_config->speed_mode, channel,
                                            gamma_value_to_duty(fade_data->cur),
                                            DUTY_SET_CYCLE - LEDC_FADE_MARGIN);
                }
                else
                {
                    fade_data->cur = fade_data->cycle && fade_data->step < 0 ? 0 : fade_data->final;
                    iot_ledc_set_duty(g_light_config->speed_mode, channel, gamma_value_to_duty(fade_data->cur));
                }

                _iot_update_duty(g_light_config->speed_mode, channel);
            }
            else
            {
                iot_ledc_set_duty(g_light_config->speed_mode, channel, gamma_value_to_duty(fade_data->cur));
                _iot_update_duty(g_light_config->speed_mode, channel);
            }
        }
        else if (fade_data->cycle)
        {
            fade_data->num = fade_data->cycle - 1;

            if (fade_data->step)
            {
                fade_data->step *= -1;
                fade_data->cur += fade_data->step;
            }
            else
            {
                fade_data->cur = (fade_data->cur == fade_data->final) ? 0 : fade_data->final;
            }

            _iot_set_fade_with_time(g_light_config->speed_mode, channel,
                                    gamma_value_to_duty(fade_data->cur),
                                    DUTY_SET_CYCLE - LEDC_FADE_MARGIN);
            _iot_update_duty(g_light_config->speed_mode, channel);
        }
        else
        {
            idle_channel_num++;
        }
    }

    if (idle_channel_num >= LEDC_CHANNEL_MAX)
    {
        iot_timer_stop(&g_light_config->timer_id);
    }
}
#endif

static strip_nav_t rgb_nav, *nav = &rgb_nav;
#if 0
mdf_err_t iot_led_init(ledc_timer_t timer_num, ledc_mode_t speed_mode, uint32_t freq_hz)
{

    mdf_err_t ret = MDF_OK;
    const ledc_timer_config_t ledc_time_config = {
        .speed_mode = speed_mode,
        .timer_num = timer_num,
        .freq_hz = freq_hz,
        .duty_resolution = LEDC_TIMER_13_BIT,
    };

    //ret = ledc_timer_config(&ledc_time_config);
    //MDF_ERROR_CHECK(ret != MDF_OK, ret, "LEDC timer configuration");

    if (g_gamma_table == NULL)
    {
        /* g_gamma_table[GAMMA_TABLE_SIZE] must be 0 */
        //    g_gamma_table = MDF_CALLOC(GAMMA_TABLE_SIZE + 1, sizeof(uint16_t));
        //    gamma_table_create(g_gamma_table, GAMMA_CORRECTION);
    }
    else
    {
        MDF_LOGE("gamma_table has been initialized");
    }

    if (g_light_config == NULL)
    {
        g_light_config = MDF_CALLOC(1, sizeof(iot_light_t));
        g_light_config->timer_num = timer_num;
        g_light_config->speed_mode = speed_mode;

        hw_timer_idx_t hw_timer = {
            .timer_group = HW_TIMER_GROUP,
            .timer_id = HW_TIMER_ID,
        };
        g_light_config->timer_id = hw_timer;
        //iot_timer_create(&hw_timer, 1, DUTY_SET_CYCLE, fade_timercb);
    }
    else
    {
        MDF_LOGE("g_light_config has been initialized");
    }

    return ESP_OK;
}

mdf_err_t iot_led_deinit()
{
    if (g_gamma_table)
    {
        MDF_FREE(g_gamma_table);
    }

    if (g_light_config)
    {
        MDF_FREE(g_light_config);
    }

    //timer_disable_intr(g_light_config->timer_id.timer_group, g_light_config->timer_id.timer_id);

    return ESP_OK;
}

mdf_err_t iot_led_regist_channel(ledc_channel_t channel, gpio_num_t gpio_num)
{
    if (channel > 2)
        return MDF_OK;

    mdf_err_t ret = MDF_OK;
    MDF_ERROR_CHECK(g_light_config == NULL, MDF_ERR_NOT_INIT, "iot_led_init() must be called first");
#ifdef CONFIG_SPIRAM_SUPPORT
    MDF_ERROR_CHECK(gpio_num != GPIO_NUM_16 || gpio_num != GPIO_NUM_17, MDF_ERR_INVALID_ARG,
                    "gpio_num must not conflict to PSRAM(IO16 && IO17)");
#endif
    const ledc_channel_config_t ledc_ch_config = {
        .gpio_num = gpio_num,
        .channel = channel,
        .intr_type = LEDC_INTR_DISABLE,
        .speed_mode = g_light_config->speed_mode,
        .timer_sel = g_light_config->timer_num,
    };

    //ret = ledc_channel_config(&ledc_ch_config);
    //MDF_ERROR_CHECK(ret != MDF_OK, ret, "LEDC channel configuration");

    return MDF_OK;
}

mdf_err_t iot_led_get_channel(ledc_channel_t channel, uint8_t *dst)
{
    if (channel > 2)
    {
        *dst = 0;
        return MDF_OK;
    }
    uint8_t *p = &rgb_nav.r;
    *dst = *(p + channel);
    return MDF_OK;
#if 0
    MDF_ERROR_CHECK(g_light_config == NULL, MDF_ERR_NOT_INIT, "iot_led_init() must be called first");
    MDF_ERROR_CHECK(dst == NULL, MDF_ERR_INVALID_ARG, "dst should not be NULL");
    int cur = g_light_config->fade_data[channel].cur;
    *dst = FIXED_2_FLOATING(cur, LEDC_FIXED_Q);
    return MDF_OK;
#endif
}

esp_err_t iot_led_set_channel(ledc_channel_t channel, uint8_t value, uint32_t fade_ms)
{
    if (channel > 2)
        return MDF_OK;

    uint8_t *p = &rgb_nav.r;
    *(p + channel) = value;

    strip_nav_t *nav = NULL;
    look_nav_by_id("Light_nav", &nav);

    if (nav)
    {
        nav->nav_id = "Light_nav";
        nav->r = rgb_nav.r;
        nav->g = rgb_nav.g;
        nav->b = rgb_nav.b;
        nav->dir = true;
        nav->start = xTaskGetTickCount();
        return MDF_OK;
    }

    nav = malloc(sizeof(strip_nav_t));
    if (!nav)
    {
        ESP_LOGE(TAG, "failed malloc mem for nav");
        return MDF_OK;
    }
    nav->nav_id = "Light_nav";
    nav->r = rgb_nav.r;
    nav->g = rgb_nav.g;
    nav->b = rgb_nav.b;
    nav->dir = true;

    add_nav(nav);

    //add or modify nav

    return MDF_OK;

#if 0

    MDF_ERROR_CHECK(g_light_config == NULL, MDF_ERR_NOT_INIT, "iot_led_init() must be called first");
    ledc_fade_data_t *fade_data = g_light_config->fade_data + channel;

    fade_data->final = FLOATINT_2_FIXED(value, LEDC_FIXED_Q);

    if (fade_ms < DUTY_SET_CYCLE)
    {
        fade_data->num = 1;
    }
    else
    {
        fade_data->num = fade_ms / DUTY_SET_CYCLE;
    }

    fade_data->step = abs(fade_data->cur - fade_data->final) / fade_data->num;

    if (fade_data->cur > fade_data->final)
    {
        fade_data->step *= -1;
    }

    if (fade_data->cycle != 0)
    {
        fade_data->cycle = 0;
    }

    if (g_hw_timer_started != true)
    {
        //iot_timer_start(&g_light_config->timer_id);
    }

    return MDF_OK;
#endif
}

esp_err_t iot_led_start_blink(ledc_channel_t channel, uint8_t value, uint32_t period_ms, bool fade_flag)
{
    if (channel > 2)
        return MDF_OK;

    MDF_ERROR_CHECK(g_light_config == NULL, MDF_ERR_NOT_INIT, "iot_led_init() must be called first");
    ledc_fade_data_t *fade_data = g_light_config->fade_data + channel;

    fade_data->final = fade_data->cur = FLOATINT_2_FIXED(value, LEDC_FIXED_Q);
    fade_data->cycle = period_ms / 2 / DUTY_SET_CYCLE;
    fade_data->num = (fade_flag) ? period_ms / 2 / DUTY_SET_CYCLE : 0;
    fade_data->step = (fade_flag) ? fade_data->cur / fade_data->num * -1 : 0;

    if (g_hw_timer_started != true)
    {
        //iot_timer_start(&g_light_config->timer_id);
    }

    return MDF_OK;
}

esp_err_t iot_led_stop_blink(ledc_channel_t channel)
{
    if (channel > 2)
        return MDF_OK;

    MDF_ERROR_CHECK(g_light_config == NULL, MDF_ERR_NOT_INIT, "iot_led_init() must be called first");
    ledc_fade_data_t *fade_data = g_light_config->fade_data + channel;
    fade_data->cycle = fade_data->num = 0;

    return MDF_OK;
}


mdf_err_t iot_led_set_gamma_table(const uint16_t gamma_table[GAMMA_TABLE_SIZE])
{
    MDF_ERROR_CHECK(g_gamma_table == NULL, MDF_ERR_NOT_INIT, "iot_led_init() must be called first");
    memcpy(g_gamma_table, gamma_table, GAMMA_TABLE_SIZE * sizeof(uint16_t));
    return MDF_OK;
}
#endif

mdf_err_t light_show_layer( mlink_handle_data_t* handle_data )
{
    #if 0
    switch ( esp_mesh_get_layer() ) {
    case 1:
        light_driver_set_rgb( 255, 0, 0 ); /**< red */
        break;

    case 2:
        light_driver_set_rgb( 255, 128, 0 ); /**< orange */
        break;

    case 3:
        light_driver_set_rgb( 255, 255, 0 ); /**< yellow */
        break;

    case 4:
        light_driver_set_rgb( 0, 255, 0 ); /**< green */
        break;

    case 5:
        light_driver_set_rgb( 0, 255, 255 ); /**< cyan */
        break;

    case 6:
        light_driver_set_rgb( 0, 0, 255 ); /**< blue */
        break;

    case 7:
        light_driver_set_rgb( 128, 0, 255 ); /**< purple */
        break;

    default:
        light_driver_set_rgb( 255, 255, 255 ); /**< white */
        break;
    }
#endif
    return MDF_OK;
}

mdf_err_t light_get_tsf_time( mlink_handle_data_t* handle_data )
{
    char tsf_time_str[ 16 ] = { 0x0 };

    sprintf( tsf_time_str, "%lld", esp_mesh_get_tsf_time() );
    mlink_json_pack( &handle_data->resp_data, "tsf_time", tsf_time_str );

    handle_data->resp_size = strlen( handle_data->resp_data );

    return MDF_OK;
}

mdf_err_t mlink_set_value( uint16_t cid, void* arg )
{
#if 0
    int value = *( ( int* )arg );

    switch ( cid ) {
    case LIGHT_CID_STATUS:
        switch ( value ) {
        case LIGHT_STATUS_ON:
        case LIGHT_STATUS_OFF:
            light_driver_set_switch( value );
            break;

        case LIGHT_STATUS_SWITCH:
            light_driver_set_switch( !light_driver_get_switch() );
            break;

        case LIGHT_STATUS_HUE: {
            uint16_t hue = light_driver_get_hue();
            hue          = ( hue + 60 ) % 360;

            light_driver_set_saturation( 100 );
            light_driver_set_hue( hue );
            break;
        }

        case LIGHT_STATUS_BRIGHTNESS: {
            if ( light_driver_get_mode() == MODE_HSV ) {
                uint8_t value = ( light_driver_get_value() + 20 ) % 100;
                light_driver_set_value( value );
            }
            else {
                uint8_t brightness = ( light_driver_get_brightness() + 20 ) % 100;
                light_driver_set_brightness( brightness );
            }

            break;
        }

        case LIGHT_STATUS_COLOR_TEMPERATURE: {
            uint8_t color_temperature = ( light_driver_get_color_temperature() + 20 ) % 100;

            if ( !light_driver_get_brightness() ) {
                light_driver_set_brightness( 30 );
            }

            light_driver_set_color_temperature( color_temperature );

            break;
        }

        default:
            break;
        }

        break;

    case LIGHT_CID_MODE:
        switch ( value ) {
        case MODE_BRIGHTNESS_INCREASE:
            light_driver_fade_brightness( 100 );
            break;

        case MODE_BRIGHTNESS_DECREASE:
            light_driver_fade_brightness( 0 );
            break;

        case MODE_HUE_INCREASE:
            light_driver_set_saturation( 100 );
            light_driver_fade_hue( 360 );
            break;

        case MODE_HUE_DECREASE:
            light_driver_set_saturation( 100 );
            light_driver_fade_hue( 0 );
            break;

        case MODE_WARM_INCREASE:
            if ( !light_driver_get_brightness() ) {
                light_driver_set_brightness( 30 );
            }

            light_driver_fade_warm( 100 );
            break;

        case MODE_WARM_DECREASE:
            if ( !light_driver_get_brightness() ) {
                light_driver_set_brightness( 30 );
            }

            light_driver_fade_warm( 0 );
            break;

        case MODE_NONE:
            light_driver_fade_stop();
            break;

        default:
            break;
        }

        break;

    case LIGHT_CID_HUE:
        light_driver_set_hue( value );
        break;

    case LIGHT_CID_SATURATION:
        light_driver_set_saturation( value );
        break;

    case LIGHT_CID_VALUE:
        light_driver_set_value( value );
        break;

    case LIGHT_CID_COLOR_TEMPERATURE:
        light_driver_set_color_temperature( value );
        break;

    case LIGHT_CID_BRIGHTNESS:
        light_driver_set_brightness( value );
        break;

    case LIGHT_CID_ADD_NAV:
        led_strip_add_nav( ( char* )arg );
        break;

    default:
        MDF_LOGE( "No support cid: %d", cid );
        return MDF_FAIL;
    }

    MDF_LOGD( "cid: %d, value: %d", cid, value );
#endif
    return MDF_OK;
}

mdf_err_t mlink_get_value( uint16_t cid, void* arg )
{
#if 0
    int* value = ( int* )arg;

    switch ( cid ) {
    case LIGHT_CID_STATUS:
        *value = light_driver_get_switch();
        break;

    case LIGHT_CID_HUE:
        *value = light_driver_get_hue();
        break;

    case LIGHT_CID_SATURATION:
        *value = light_driver_get_saturation();
        break;

    case LIGHT_CID_VALUE:
        *value = light_driver_get_value();
        break;

    case LIGHT_CID_COLOR_TEMPERATURE:
        *value = light_driver_get_color_temperature();
        break;

    case LIGHT_CID_BRIGHTNESS:
        *value = light_driver_get_brightness();
        break;

    case LIGHT_CID_MODE:
        *value = light_driver_get_mode();
        break;
        // case LIGHT_CID_ADD_NAV:
        //    led_strip_get_nav((char **)arg);
        //    break;

    default:
        MDF_LOGE( "No support cid: %d", cid );
        return MDF_FAIL;
    }

    MDF_LOGV( "cid: %d, value: %d", cid, *value );
#endif
    return MDF_OK;
}