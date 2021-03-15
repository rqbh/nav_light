#define EASYPR
#undef EPS  // specreg.h defines EPS which interfere with opencv
#ifdef EASYPR
#include "easypr.h"
//#include "easypr/util/switch.hpp"
//#include "accuracy.hpp"
//#include "chars.hpp"
#else
#include "opencv2/core.hpp"
#include "opencv2/imgcodecs.hpp"
#include "opencv2/imgproc.hpp"
#endif
#define EPS 192

#include "freertos/FreeRTOS.h"
#include <esp_err.h>
#include <esp_freertos_hooks.h>
#include <esp_log.h>
#include <esp_timer.h>

#include <freertos/task.h>
#include <iostream>
#include <map>

#define HAVE_ABS

#include "app_camera.h"
#include "app_screen.h"
#include "iot_lvgl.h"
#include "system.h"

using namespace cv;
using namespace std;
#ifdef EASYPR
using namespace easypr;
//using namespace demo;
#endif
extern "C" {
void opencv_task( void );
}

#define TAG "main"

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunused-variable"

    extern CEspLcd*    tft;

static lv_obj_t* lvCameraImage;  // Camera image object

/****************** vlpr start ***********************************/
static void fillHole( const Mat srcBw, Mat& dstBw );  //填补算法
static Mat  cutEdge( Mat cutImage );                  //边框切割算法
static void cutChar( Mat srcImage );                  //单个字符切割算法
static Mat  Location( Mat srcImage );                 //图像识别算法
static void singleCharCut( Mat doubleImage, int k1, int k2 );
static void showChar();
static void matchProvince();
static void matchNumber();
static void readProvince();
static void readNumber();
static void VideoShow( Mat videoImage );
static void GetStringSize( /*HDC hDC,*/ const char* str, int* w, int* h );
// static void putTextZH( Mat& dst, const char* str, Point org, Scalar color, int fontSize, const char* fn, bool italic, bool underline );
static void demo_task( void* p );

//__attribute__( ( section( ".ext_ram.bss"
//                         "." _COUNTER_STRINGIFY( __COUNTER__ ) ) ) )
int           tilt_flag;                 //判断是否倾斜，需不需要二次定位车牌
bool          locate_flag;               //判断提取是否成功
bool          specialFlag      = false;  //针对嵌套车牌
int           captureRead      = 0;
int           videoFlag        = 0;
string        carPlateProvince = " ";
string        carPlate         = " ";
char          test[ 10 ];
vector< Mat > singleChar;  //字符图片容器
/**************** vlpr end ***********************************/


void gui_boot_screen()
{
    static lv_style_t style;
    lv_style_init( &style );

    lv_style_set_radius( &style, LV_STATE_DEFAULT, 2 );
    lv_style_set_bg_opa( &style, LV_STATE_DEFAULT, LV_OPA_COVER );
    lv_style_set_bg_color( &style, LV_STATE_DEFAULT, LV_COLOR_MAKE( 190, 190, 190 ) );
    lv_style_set_border_width( &style, LV_STATE_DEFAULT, 2 );
    lv_style_set_border_color( &style, LV_STATE_DEFAULT, LV_COLOR_MAKE( 142, 142, 142 ) );

    lv_style_set_pad_top( &style, LV_STATE_DEFAULT, 60 );
    lv_style_set_pad_bottom( &style, LV_STATE_DEFAULT, 60 );
    lv_style_set_pad_left( &style, LV_STATE_DEFAULT, 60 );
    lv_style_set_pad_right( &style, LV_STATE_DEFAULT, 60 );

    lv_style_set_text_color( &style, LV_STATE_DEFAULT, LV_COLOR_MAKE( 102, 102, 102 ) );
    lv_style_set_text_letter_space( &style, LV_STATE_DEFAULT, 5 );
    lv_style_set_text_line_space( &style, LV_STATE_DEFAULT, 20 );

    /*Create an object with the new style*/
    lv_obj_t* obj = lv_label_create( lv_scr_act(), NULL );
    lv_obj_add_style( obj, LV_LABEL_PART_MAIN, &style );
    lv_label_set_text( obj, "TTGO\n"
                            "demo!" );
    lv_obj_align( obj, NULL, LV_ALIGN_CENTER, 0, 0 );
    wait_msec( 3000 );
}

void gui_init()
{
    // Create screen
    lv_obj_t* scr = lv_obj_create( nullptr, nullptr );
    lv_scr_load( scr );
}

esp_err_t updateCameraImage( const cv::Mat& img )
{
    // static variables because they must still be available when lv_task_handler() is called
    static Mat          imgCopy;
    static lv_img_dsc_t my_img_dsc;

    if ( img.empty() ) {
        ESP_LOGW( TAG, "Can't display empty image" );
        return ESP_ERR_INVALID_ARG;
    }

    // convert image to bgr565 if needed
    if ( img.type() == CV_8UC1 ) {  // grayscale image
        cvtColor( img, imgCopy, COLOR_GRAY2BGR565, 1 );
    }
    else if ( img.type() == CV_8UC3 ) {  // BGR888 image
        cvtColor( img, imgCopy, COLOR_BGR2BGR565, 1 );
    }
    else if ( img.type() == CV_8UC2 ) {  // BGR565 image
        img.copyTo( imgCopy );
    }

    my_img_dsc.header.always_zero = 0;
    my_img_dsc.header.w           = imgCopy.cols;
    my_img_dsc.header.h           = imgCopy.rows;
    my_img_dsc.header.cf          = LV_IMG_CF_TRUE_COLOR;
    my_img_dsc.data_size          = imgCopy.size().width * imgCopy.size().height;
    my_img_dsc.data               = imgCopy.ptr< uchar >( 0 );

    lv_img_set_src( lvCameraImage, &my_img_dsc ); /* Set the created file as image */
    lv_obj_set_pos( lvCameraImage, -40, 0 );

    return ESP_OK;
}

// mode of the demo
enum class DisplayMode : uint8_t { RGB, GRAYSCALE, BINARIZED, EDGES, NUM_OF_MODES };
static DisplayMode currentDisplayMode;

static const std::string displayModeToString( DisplayMode dispMode )
{
    const std::map< DisplayMode, const std::string > DisplayModeStrings{
        { DisplayMode::RGB, "RGB" },
        { DisplayMode::GRAYSCALE, "GRAYSCALE" },
        { DisplayMode::BINARIZED, "BINARIZED" },
        { DisplayMode::EDGES, "EDGES" },
    };
    auto it = DisplayModeStrings.find( dispMode );
    return ( it == DisplayModeStrings.end() ) ? "Out of range" : it->second;
}

#ifndef EASYPR
/**
 * Task doing the demo: Getting image from camera, processing it with opencv depending on the displayMode and
 * displaying it on the lcd
 */
static void demo_task( void* arg )
{
    ESP_LOGI( TAG, "Starting demo_task" );

    // Display memory infos
    disp_infos();

    tft->setRotation( 2 );  // rotation needed if camera is on the back of the device
    sensor_t* s = esp_camera_sensor_get();

    // Init camera image Lvgl object
    lvCameraImage = lv_img_create( lv_disp_get_scr_act( nullptr ), nullptr );
    lv_obj_move_foreground( lvCameraImage );

    while ( true ) {
        auto start = esp_timer_get_time();

        camera_fb_t* fb = esp_camera_fb_get();

        if ( !fb ) {
            ESP_LOGE( TAG, "Camera capture failed" );
        }
        else {
            if ( s->pixformat == PIXFORMAT_JPEG ) {
                TFT_jpg_image( CENTER, CENTER, 0, -1, NULL, fb->buf, fb->len );
                esp_camera_fb_return( fb );
                fb = NULL;
            }
            else {                                                          // RGB565 pixformat
                Mat inputImage( fb->height, fb->width, CV_8UC2, fb->buf );  // rgb565 is 2 channels of 8-bit unsigned

                if ( currentDisplayMode == DisplayMode::RGB ) {
                }
                else if ( currentDisplayMode == DisplayMode::GRAYSCALE ) {
                    cvtColor( inputImage, inputImage, COLOR_BGR5652GRAY );
                }
                else if ( currentDisplayMode == DisplayMode::BINARIZED ) {
                    cvtColor( inputImage, inputImage, COLOR_BGR5652GRAY );
                    threshold( inputImage, inputImage, 128, 255, THRESH_BINARY );
                }
                else if ( currentDisplayMode == DisplayMode::EDGES ) {
                    cvtColor( inputImage, inputImage, COLOR_BGR5652GRAY );
                    // Reduce noise with a kernel 3x3
                    blur( inputImage, inputImage, Size( 3, 3 ) );
                    /** Apply the canny edges detector with:
                     * - low threshold = 50
                     * - high threshold = 4x low
                     * - sobel kernel size = 3x3
                     */
                    int lowThresh = 40;
                    int kernSize  = 3;
                    Canny( inputImage, inputImage, lowThresh, 4 * lowThresh, kernSize );
                }
                else {
                    ESP_LOGE( TAG, "Wrong display mode: %d", ( int )currentDisplayMode );
                }

                // display image on lcd
                updateCameraImage( inputImage );
            }
        }
        // vTaskDelay(500/portTICK_PERIOD_MS);
        ESP_LOGI( TAG, "%s mode: around %f fps", displayModeToString( currentDisplayMode ).c_str(), 1.0f / ( ( esp_timer_get_time() - start ) / 1000000.0f ) );
    }
}
#endif

/**
 * Task changing the current displayMode at regular interval
 */
static void timer_task( void* arg )
{
    while ( true ) {
        wait_msec( 3000 );
        currentDisplayMode = static_cast< DisplayMode >( ( static_cast< int >( currentDisplayMode ) + 1 ) % static_cast< int >( DisplayMode::NUM_OF_MODES ) );
    }
}

void opencv_task()
{
    ESP_LOGI( TAG, "Starting main" );

    /* initializations */
    app_camera_init();
    lvgl_init();
    gui_init();

    /* display boot screen */
    gui_boot_screen();

    /* Display memory infos */
    disp_infos();

    ESP_LOGI( TAG, "Display width = %d, height = %d", tft->width(), tft->height() );

    /* Start the tasks */
    xTaskCreatePinnedToCore( demo_task, "opencv", 1024 * 9, nullptr, 24, nullptr, 1 );
    xTaskCreatePinnedToCore( timer_task, "timer", 1024 * 1, nullptr, 24, nullptr, 1 );
}
#if defined( EASYPR )
/************** vlpr start ************************************/
static void demo_task( void* arg )
{
    ESP_LOGI( TAG, "Starting demo_task" );

    // Display memory infos
    disp_infos();

    tft->setRotation( 2 );  // rotation needed if camera is on the back of the device
    sensor_t* s = esp_camera_sensor_get();

    // Init camera image Lvgl object
    lvCameraImage = lv_img_create( lv_disp_get_scr_act( nullptr ), nullptr );
    lv_obj_move_foreground( lvCameraImage );

    Mat srcImage;
    Mat theFirst;
    int singleCharLength;
    //读取匹配用的省份和字符图片
    //readProvince();
    //readNumber();

    while ( true ) {
        auto start = esp_timer_get_time();

        camera_fb_t* fb = esp_camera_fb_get();

        if ( !fb ) {
            ESP_LOGE( TAG, "Camera capture failed" );
        }
        else {
            if ( s->pixformat == PIXFORMAT_JPEG ) {
                TFT_jpg_image( CENTER, CENTER, 0, -1, NULL, fb->buf, fb->len );
                esp_camera_fb_return( fb );
                fb = NULL;
            }
#if 0
            else {                                                          // RGB565 pixformat
                Mat inputImage( fb->height, fb->width, CV_8UC2, fb->buf );  // rgb565 is 2 channels of 8-bit unsigned

                if ( currentDisplayMode == DisplayMode::RGB ) {
                }
                else if ( currentDisplayMode == DisplayMode::GRAYSCALE ) {
                    cvtColor( inputImage, inputImage, COLOR_BGR5652GRAY );
                }
                else if ( currentDisplayMode == DisplayMode::BINARIZED ) {
                    cvtColor( inputImage, inputImage, COLOR_BGR5652GRAY );
                    threshold( inputImage, inputImage, 128, 255, THRESH_BINARY );
                }
                else if ( currentDisplayMode == DisplayMode::EDGES ) {
                    cvtColor( inputImage, inputImage, COLOR_BGR5652GRAY );
                    // Reduce noise with a kernel 3x3
                    blur( inputImage, inputImage, Size( 3, 3 ) );
                    /** Apply the canny edges detector with:
                     * - low threshold = 50
                     * - high threshold = 4x low
                     * - sobel kernel size = 3x3
                     */
                    int lowThresh = 40;
                    int kernSize  = 3;
                    Canny( inputImage, inputImage, lowThresh, 4 * lowThresh, kernSize );
                }
                else {
                    ESP_LOGE( TAG, "Wrong display mode: %d", ( int )currentDisplayMode );
                }
            }
#endif
            else {
                // try {
                Mat srcImage( fb->height, fb->width, CV_8UC2, fb->buf );
#if 0

                resize( srcImage, srcImage, Size( 400 * srcImage.cols / srcImage.rows, 400 ) );

                //定位车牌
                theFirst = Location( srcImage );

                if ( locate_flag ) {
                    if ( tilt_flag == 1 ) {  //是否需要旋转重新定位
                        theFirst  = Location( theFirst );
                        tilt_flag = false;
                    }
                }

                if ( locate_flag ) {
                    theFirst = cutEdge( theFirst );  //上下去杂边
                    cutChar( theFirst );             //单个字符切割
                    singleCharLength = singleChar.size();
                    ESP_LOGI( TAG, "采集到字符轮廓数: %d", singleCharLength );
                    showChar();  //显示图框
                    if ( singleCharLength >= 7 ) {
                        matchProvince();
                        matchNumber();
                    }

                    singleChar.clear();
                }
                //}
                // catch ( Exception e ) {
                //    ESP_LOGE( TAG, "Standard exception: %s", e.what() );
                //}
#else
                CPlateRecognize pr;
                pr.setLifemode( true );
                pr.setDebug( false );
                pr.setMaxPlates( 1 );
                // pr.setDetectType(PR_DETECT_COLOR | PR_DETECT_SOBEL);
                pr.setDetectType( easypr::PR_DETECT_CMSER );

                // vector<string> plateVec;
                vector< CPlate > plateVec;

                int result = pr.plateRecognize( srcImage, plateVec );
                // int result = pr.plateRecognizeAsText(src, plateVec);
                if ( result == 0 ) {
                    size_t num = plateVec.size();
                    for ( size_t j = 0; j < num; j++ ) {
                        ESP_LOGI( TAG, "plateRecognize: %s", plateVec[ j ].getPlateStr().c_str() );
                    }
                }

                if ( result != 0 )
                    cout << "result:" << result << endl;
#endif
            }
            // display image on lcd
            // updateCameraImage( srcImage );
        }

        ESP_LOGI( TAG, "%s mode: around %f fps", displayModeToString( currentDisplayMode ).c_str(), 1.0f / ( ( esp_timer_get_time() - start ) / 1000000.0f ) );
    }
}

#if 0
static void vlpr_task( void* p ) {

    //视频操作
    //  VideoCapture capture("1.mp4");
    Mat          srcImage;
    Mat          theFirst;
    int          singleCharLength;
    camera_fb_t* fb;
    sensor_t*    s = esp_camera_sensor_get();

    //读取字符图片
    readProvince();
    readNumber();

    while ( 1 ) {
        // TODO: wait for scan
        // TODO: get image framebuffer fro camera
        camera_fb_t* fb = esp_camera_fb_get();
        if ( fb ) {
            if ( s->pixformat == PIXFORMAT_JPEG ) {
                TFT_jpg_image( CENTER, CENTER, 0, -1, NULL, fb->buf, fb->len );
            }
        }
        capture >> srcImage;
        try {
            if ( !srcImage.data ) {
                printf( "视频识别结束    \n" );
                continue;
            }

            // if (srcImage.rows >= srcImage.cols)
            //{
            //    resize(srcImage, srcImage, Size(640, 640 * srcImage.rows / srcImage.cols));
            //}
            // else
            //{
            resize( srcImage, srcImage, Size( 400 * srcImage.cols / srcImage.rows, 400 ) );
            //}

            //车牌定位
            theFirst = Location( srcImage );

            if ( locate_flag ) {
                if ( tilt_flag == 1 )  //旋转后要再次定位去上下杂边
                {
                    theFirst  = Location( theFirst );
                    tilt_flag = 0;
                }
            }
            if ( locate_flag ) {
                //车牌切割(切割上下边，去除干扰)
                theFirst = cutOne( theFirst );
                //单个字符切割
                CharCut( theFirst );
                singleCharLength = singleChar.size();
                printf( "采取字符轮廓数                               %d\n", singleCharLength );
                ShowChar();
                if ( singleCharLength >= 7 ) {

                    matchProvince();
                    matchNumber();
                }

                singleChar.clear();
            }
        }
        catch ( Exception e ) {

            cout << "Standard ecxeption : " << e.what() << " \n" << endl;
        }

        // if (waitKey(30) >= 0)
        //    break;
        //延时30ms
    }
    // imwrite("match\\xxxxxx.bmp", singleChar[2]);
}
#endif

static void fillHole( const Mat srcBw, Mat& dstBw )
{
    Size imageSize = srcBw.size();
    Mat  Temp      = Mat::zeros( imageSize.height + 2, imageSize.width + 2, srcBw.type() );  //延展图像
    srcBw.copyTo( Temp( Range( 1, imageSize.height + 1 ), Range( 1, imageSize.width + 1 ) ) );

    cv::floodFill( Temp, Point( 0, 0 ), Scalar( 255 ) );

    Mat cutImg;  //裁剪延展的图像
    Temp( Range( 1, imageSize.height + 1 ), Range( 1, imageSize.width + 1 ) ).copyTo( cutImg );

    dstBw = srcBw | ( ~cutImg );
}

//定位车牌位置
static Mat Location( Mat srcImage )
{
    //判断变量重赋值
    locate_flag = false;

    //用于旋转车牌
    int imageWidth, imageHeight;  //输入图像的长和宽
    imageWidth  = srcImage.rows;  //获取图片的宽
    imageHeight = srcImage.cols;  //获取图像的长

    Mat blueROI = srcImage.clone();  //复制原图像
    cvtColor( blueROI, blueROI, COLOR_BGR2HSV );
    // namedWindow("hsv图");
    updateCameraImage( blueROI );

    //中值滤波操作
    medianBlur( blueROI, blueROI, 3 );  //图像模糊处理
    // namedWindow("medianBlur图");
    updateCameraImage( blueROI );

    //将蓝色区域二值化
    inRange( blueROI, Scalar( 100, 130, 50 ), Scalar( 124, 255, 255 ), blueROI );
    // namedWindow("blue图");
    updateCameraImage( blueROI );

    Mat element1 = getStructuringElement( MORPH_RECT, Size( 2, 2 ) );  // size（）对速度有影响
    morphologyEx( blueROI, blueROI, MORPH_OPEN, element1 );
    // namedWindow("0次K运算后图像");
    updateCameraImage( blueROI );

    Mat element0 = getStructuringElement( MORPH_ELLIPSE, Size( 10, 10 ) );  // size（）对速度有影响
    morphologyEx( blueROI, blueROI, MORPH_CLOSE, element0 );
    // namedWindow("0次闭运算后图像");
    updateCameraImage( blueROI );

    vector< vector< Point > > contours;

    findContours( blueROI, contours, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE );

    int contours_cnt = contours.size();
    ESP_LOGI( TAG, "采集到蓝色区域数: %d", contours_cnt );

    if ( contours_cnt == 0 ) {
        if ( !locate_flag ) {  //在视频中显示
            return srcImage;
        }
    }

    double      contour_area;
    double      contour_longside, contour_shortside, long2short;
    float       contour_angle = 0;
    Rect        contour_rect;
    RotatedRect box;          //可旋转的矩形盒子
    Point2f     vertex[ 4 ];  //四个顶点

    Mat image = srcImage.clone();  //为后来显示做准备
    Mat rgbCutImg;                 //车牌裁剪图

    for ( int i = 0; i < contours_cnt; i++ ) {
        contour_area = contourArea( contours[ i ] );       //获取轮廓面积
        if ( contour_area > 600 && contour_area < 15000 )  //矩形区域面积大小判断
        {
            contour_rect = boundingRect( contours[ i ] );  //计算矩形边界
            box          = minAreaRect( contours[ i ] );   //获取轮廓的矩形
            box.points( vertex );                          //获取矩形四个顶点坐标
            contour_angle = box.angle;                     //得到车牌倾斜角度

            contour_longside  = sqrt( pow( vertex[ 1 ].x - vertex[ 0 ].x, 2 ) + pow( vertex[ 1 ].y - vertex[ 0 ].y, 2 ) );
            contour_shortside = sqrt( pow( vertex[ 2 ].x - vertex[ 1 ].x, 2 ) + pow( vertex[ 2 ].y - vertex[ 1 ].y, 2 ) );
            if ( contour_shortside > contour_longside )  //短轴大于长轴，交换数据
            {
                double temp;
                temp              = contour_longside;
                contour_longside  = contour_shortside;
                contour_shortside = temp;
                ESP_LOGI( TAG, "长短边交换" );
            }
            else
                contour_angle += 90;  // ???

            long2short = contour_longside / contour_shortside;
            if ( long2short > 1.5 && long2short < 4.5 ) {
                locate_flag = true;            //得到了合适的区域
                for ( int i = 0; i < 4; ++i )  //划线框出车牌区域
                    line( image, vertex[ i ], vertex[ ( ( i + 1 ) % 4 ) ? ( i + 1 ) : 0 ], Scalar( 0, 255, 0 ), 1, LINE_AA );

                if ( !tilt_flag )  //在视频中显示
                {
                    ESP_LOGI( TAG, "采集成功" );
                    // TODO: show image on screen
                }

                rgbCutImg = srcImage( contour_rect );
                // namedWindow("车牌图");
                updateCameraImage( rgbCutImg );  //裁剪出车牌

                break;  //退出循环，以免容器中变量变换
            }
        }
    }

    ESP_LOGI( TAG, "倾斜角度: %f", contour_angle );
    if ( locate_flag && fabs( contour_angle ) > 0.8 )  //车牌过偏，转一下,偏移角度小时可不调用，后续找到合适范围再改进
    {
        tilt_flag = 1;
        Mat     rotractImg( imageWidth, imageHeight, CV_8UC2, Scalar( 0, 0, 0 ) );   //倾斜矫正图片
        Point2f center = box.center;                                                 //获取车牌中心坐标
        Mat     M2     = getRotationMatrix2D( center, contour_angle, 1 );            //计算旋转加缩放的变换矩阵
        warpAffine( srcImage, rotractImg, M2, srcImage.size(), 1, 0, Scalar( 0 ) );  //进行倾斜矫正
        // namedWindow("倾斜矫正后图片",0);
        updateCameraImage( rotractImg );

        rgbCutImg = rotractImg( contour_rect );  //截取车牌彩色照片
        // namedWindow("矫正后车牌照");
        updateCameraImage( rgbCutImg );
        ESP_LOGI( TAG, "矩形中心: %f  %f", box.center.x, box.center.y );

        return rgbCutImg;
    }

    if ( locate_flag == false ) {
        ESP_LOGI( TAG, "提取失败\n" );  //后期加边缘检测法识别
        if ( !tilt_flag )               //在视频中显示
        {
            /*namedWindow("提取车牌结果图");*/
            updateCameraImage( image );
        }
    }
    return rgbCutImg;
}

static Mat cutEdge( Mat cutImage )
{
    //打印车牌长宽
    //    try
    //    {
    ESP_LOGI( TAG, "cutEdge: rows %d, cols %d", cutImage.rows, cutImage.cols );

    if ( cutImage.rows >= cutImage.cols )
        resize( cutImage, cutImage, Size( 320, 320 * cutImage.rows / cutImage.cols ) );
    //    }
    //    catch (Exception e)
    //    {
    //        resize(cutImage, cutImage, Size(320, 100));
    //    }
    /*namedWindow("Resize车牌图");*/
    updateCameraImage( cutImage );

    int height = cutImage.rows;
    //    cout << "\tHeight:" << height << "\tWidth:" << 320 << endl;
    if ( height < 86 ) {
        //!!!!!!!!!!!!!!!!!!!!!!!!!!!!处理新型嵌套车牌!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
        ESP_LOGI( TAG, "嵌套车牌" );
        specialFlag = true;
    }

    Mat whiteROI = cutImage.clone();

    if ( specialFlag ) {
        cvtColor( whiteROI, whiteROI, COLOR_BGR2HSV );
        //将白色区域二值化
        // inRange(whiteROI, Scalar(0, 0, 0), Scalar(130, 50, 245), whiteROI);      //增大 S 即饱和度可以使hsv白色检测范围更大

        inRange( whiteROI, Scalar( 0, 0, 0 ), Scalar( 180, 100, 245 ), whiteROI );
        // namedWindow("specialFlagwhiteROI图");
        updateCameraImage( whiteROI );
    }
    else {
        GaussianBlur( whiteROI, whiteROI, Size( 3, 3 ), 0, 0 );
        /*namedWindow("GaussianBlur车牌图");*/
        updateCameraImage( whiteROI );

        cvtColor( whiteROI, whiteROI, COLOR_BGR2HSV );

        // medianBlur(whiteROI, whiteROI, 3);
        // namedWindow("Src_medianBlur图");
        updateCameraImage( whiteROI );

        //将白色区域二值化
        // inRange(whiteROI, Scalar(0, 0, 10), Scalar(180, 30, 255), whiteROI);      //增大 S 即饱和度可以使hsv白色检测范围更大
        inRange( whiteROI, Scalar( 0, 0, 10 ), Scalar( 180, 120, 255 ), whiteROI );
        // namedWindow("whiteROI图");
        updateCameraImage( whiteROI );
    }

    /*
    Mat element0 = getStructuringElement(MORPH_ELLIPSE, Size(4, 4));     //size（）对速度有影响
    morphologyEx(whiteROI, whiteROI, MORPH_OPEN, element0);
    namedWindow("OPEN图");
    imshow("OPEN图", whiteROI);
    */
    Mat                       dstImage = cutImage.clone();
    vector< vector< Point > > contours;

    findContours( whiteROI, contours, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE );
    drawContours( dstImage, contours, -1, Scalar( 0, 0, 255 ), 1 );
    // namedWindow("疑似字符轮廓识别图");
    updateCameraImage( dstImage );

    inRange( dstImage, Scalar( 0, 0, 255 ), Scalar( 0, 0, 255 ), dstImage );
    // namedWindow("字符大轮廓图");
    updateCameraImage( dstImage );
    /*fillHole(dstImage, dstImage);
    namedWindow("填补轮廓后图");
    updateCameraImage(dstImage);*/

    int row1      = 2;
    int row2      = dstImage.rows;
    int rowMax    = dstImage.rows - 1;  //开区间，防止越界
    int colMax    = dstImage.cols - 1;  //开区间，防止越界
    int addFirst  = 10;
    int addFirst0 = 0;
    int addFirst1 = 0;
    int addFirst2 = 0;
    //测中间像素
    // dstImage.at<uchar>(rowMax-1, colMax-1);
    // cout << "Width:" << j << endl;

    int addFirstTemp = addFirst;  //第一次用时已经改变数值，容易忽略！！！！！

    uchar* data;

    //裁剪上下边
    //上边
    for ( int i = 2; i < rowMax / 3; i++, addFirst1 = 0 )  //   6     刚刚好
    {
        data = dstImage.ptr< uchar >( i );
        for ( int j = 2; j < colMax; j++ ) {
            if ( data[ j ] == 255 ) {
                addFirst1++;
            }
        }
        if ( addFirst1 < addFirst )  //筛选最小值所在行
        {
            row1     = i;
            addFirst = addFirst1 + 3;
            // cout << "行头" << row1 << endl;
            // flag_x = 1;
        }
    }
    //下边
    for ( int i = rowMax - 2; i > rowMax - rowMax / 4; i--, addFirst2 = 0 )  //   6     刚刚好
    {
        data = dstImage.ptr< uchar >( i );
        for ( int j = 2; j < colMax; j++ ) {
            if ( data[ j ] == 255 ) {
                addFirst2++;
            }
        }
        if ( addFirst2 < addFirstTemp )  //筛选最小值所在行
        {
            row2         = i;
            addFirstTemp = addFirst2 + 3;
            // cout << "行底" << row2 << endl;
            // flag_y = 1;
        }
    }

    int orow;
    orow = row2 - row1;
    Mat w_image;
    Mat rgb_w_image;

    w_image     = dstImage( Rect( 0, row1, colMax, orow ) );
    rgb_w_image = cutImage( Rect( 0, row1, colMax, orow ) );
    // namedWindow("裁剪上下图");
    updateCameraImage( w_image );

    int rowMax_ALT = w_image.rows - 1;  //开区间，防止越界(注意，裁剪完上下后要重新写行和宽，因为行和宽已经改变)
    int colMax_ALT = w_image.cols - 1;  //开区间，防止越界（注意，裁剪完上下后要重新写行和宽，因为行和宽已经改变）
    int col_1      = 2;
    int col_2      = w_image.cols;
    int add        = 2;
    int add1       = 0;
    int add2       = 0;

    int addTemp = add;  //第一次用时已经改变数值，容易忽略！！！！！

    //裁剪左右边。。。。。。。。。。。。。。。。。。。。。。。。。。。。。。。。。。。。。。
    //左边
    // for (int i = 0; i < colMax_ALT / 18; i++, add1 = 0)                           //       刚刚好
    //{
    //	for (int j = 2; j < rowMax_ALT; j++)
    //	{
    //		data = dstImage.ptr<uchar>(j);
    //		if (data[i] == 255)
    //		{
    //			add1++;
    //		}
    //	}
    //	if (add1 < add)                       //筛选最小值所在列
    //	{
    //		col_1 = i;
    //		add = add1 + 1;
    //	}
    //}
    //右边
    if ( specialFlag ) {
        for ( int i = colMax_ALT; i > colMax_ALT - colMax_ALT / 18; i--, add2 = 0 )  //        刚刚好
        {
            for ( int j = 2; j < rowMax_ALT; j++ ) {
                data = dstImage.ptr< uchar >( j );
                if ( data[ i ] == 255 ) {
                    add2++;
                }
            }
            if ( add2 < addTemp )  //筛选最小值所在列
            {
                col_2   = i;
                addTemp = add2 + 1;
                // cout << "行底" << row2 << endl;
            }
        }
    }
    int o_col;
    o_col = col_2 - col_1;

    Mat H_image;
    H_image     = w_image( Rect( col_1, 0, o_col, rowMax_ALT ) );
    rgb_w_image = rgb_w_image( Rect( col_1, 0, o_col, rowMax_ALT ) );
    // namedWindow("再裁剪左右图");
    updateCameraImage( H_image );
    // namedWindow("裁剪后彩图");
    updateCameraImage( rgb_w_image );

    return rgb_w_image;
}

static void cutChar( Mat srcImage )
{
    resize( srcImage, srcImage, Size( 320, 320 * srcImage.rows / srcImage.cols ) );
    // namedWindow("Resize车牌图");
    updateCameraImage( srcImage );

    GaussianBlur( srcImage, srcImage, Size( 3, 3 ), 0, 0 );
    /*namedWindow("GaussianBlur车牌图");*/
    updateCameraImage( srcImage );

    medianBlur( srcImage, srcImage, 3 );
    // namedWindow("Src_medianBlur图");
    updateCameraImage( srcImage );

    cvtColor( srcImage, srcImage, COLOR_BGR2HSV );

    //将白色区域二值化

    Mat doubleImage;
    // inRange(srcImage, Scalar(0, 0, 10), Scalar(180, 75, 255), doubleImage);      //增大 S 即饱和度可以使hsv白色检测范围更大
    inRange( srcImage, Scalar( 0, 0, 0 ), Scalar( 180, 125, 245 ), doubleImage );
    // namedWindow("doubleImage图");
    updateCameraImage( doubleImage );

    int    colTemp  = 0;
    int    rowMax   = doubleImage.rows;
    int    colMax   = doubleImage.cols;
    int    addFirst = 0;
    int    add      = 0;
    int    k1       = 0;
    int    k2;
    int    kTemp     = 0;
    int    times     = 0;
    int    oneCutEnd = 0;
    float  t         = 1.0;
    uchar* data;

    //针对嵌套车牌处理
    if ( specialFlag ) {
        for ( int i = 2; i < colMax; i++, addFirst = 0, add = 0 ) {
            for ( int j = rowMax / 10.8; j < rowMax - rowMax / ( 10.8 * t ); j++ ) {
                data = doubleImage.ptr< uchar >( j );
                if ( data[ i - 1 ] == 255 ) {
                    addFirst++;  //统计前一列
                }
            }

            for ( int j = rowMax / 10.8; j < rowMax - rowMax / ( 10.8 * t ); j++ ) {
                data = doubleImage.ptr< uchar >( j );
                if ( data[ i ] == 255 ) {
                    add++;  //统计后一列
                }
            }
            //省份字符分开切割
            if ( !times ) {
                if ( !oneCutEnd && ( !addFirst && add ) )
                    k1 = i - 1;
                if ( addFirst && !add ) {
                    k2        = i;
                    oneCutEnd = 1;
                    if ( k2 - k1 > colMax / 11.25 ) {
                        times = 1;
                        if ( k2 - k1 < colMax / 5.625 )
                            singleCharCut( doubleImage, k1, k2 );
                        else
                            i = 2;
                    }
                }
            }  //切割其他字符
            else {

                if ( !addFirst && add )
                    k1 = i - 1;
                if ( addFirst && !add ) {
                    k2 = i;
                    if ( k2 - k1 > colMax / 32 ) {
                        if ( k2 - k1 < colMax / 5.625 )
                            singleCharCut( doubleImage, k1, k2 );
                        else  //针对嵌套车牌下部连接过靠上
                        {
                            i = k1;
                            t -= 0.1;
                        }
                    }
                    else {  //!!!!!!!!!!!!!!!!!!!!!!处理中间分割点与‘ 1 ’!!!!!!!!!!!!!!!!!!!!!!!!
                        for ( int a = k1; a <= k2; a++ ) {
                            data = doubleImage.ptr< uchar >( rowMax / 5 );
                            if ( data[ a ] == 255 )
                                kTemp++;
                        }
                        if ( kTemp > 0 )
                            singleCharCut( doubleImage, k1, k2 );
                        kTemp = 0;
                    }
                }
            }
        }

        k2 = colMax;
        if ( k2 - k1 > colMax / 32 )
            singleCharCut( doubleImage, k1, k2 );

        specialFlag = false;
    }
    else {
        for ( int i = 2; i < colMax; i++, addFirst = 0, add = 0 ) {
            for ( int j = rowMax / 12.8; j < rowMax - rowMax / 12.8; j++ ) {
                data = doubleImage.ptr< uchar >( j );
                if ( data[ i - 1 ] == 255 ) {
                    addFirst++;
                }
            }

            for ( int j = rowMax / 12.8; j < rowMax - rowMax / 12.8; j++ ) {
                data = doubleImage.ptr< uchar >( j );
                if ( data[ i ] == 255 ) {
                    add++;
                }
            }

            if ( !times ) {
                if ( !oneCutEnd && ( !addFirst && add ) )
                    k1 = i - 1;
                if ( addFirst && !add ) {
                    k2        = i;
                    oneCutEnd = 1;
                    if ( k2 - k1 > colMax / 11.25 ) {
                        times = 1;
                        if ( k2 - k1 < colMax / 5.625 )
                            singleCharCut( doubleImage, k1, k2 );
                        else
                            i = 2;
                    }
                }
            }
            else {

                if ( !addFirst && add )
                    k1 = i - 1;
                if ( addFirst && !add ) {
                    k2 = i;
                    if ( k2 - k1 > colMax / 32 )
                        singleCharCut( doubleImage, k1, k2 );
                    else {  //!!!!!!!!!!!!!!处理中间分割点与‘ 1 ’!!!!!!!!!!!!!!!!!!!!!!!!!!!!
                        for ( int a = k1; a <= k2; a++ ) {
                            data = doubleImage.ptr< uchar >( rowMax / 5 );
                            if ( data[ a ] == 255 )
                                kTemp++;
                        }
                        if ( kTemp > 0 )
                            singleCharCut( doubleImage, k1, k2 );
                        kTemp = 0;
                    }
                }
            }
        }
    }
}

static void singleCharCut( Mat doubleImage, int k1, int k2 )
{
    // printf("k1 = %d ,k2 = %d\n", k1, k2);

    int rowMax = doubleImage.rows;
    Mat image  = doubleImage( Rect( k1, 0, k2 - k1, rowMax ) );

    int row1         = 0;
    int row2         = image.rows;
    rowMax           = image.rows - 1;  //开区间，防止越界
    int    colMax    = image.cols;      //开区间，防止越界
    int    addFirst  = 2;
    int    addFirst1 = 0;
    int    addFirst2 = 0;
    uchar* data;
    //测中间像素
    // image.at<uchar>(rowMax-1, colMax-1);
    // cout << "Width:" << j << endl;

    int addFirstTemp = addFirst;  //第一次用时已经改变数值，容易忽略！！！！！

    //裁剪上下边。。。。。。。。。。。。。。。。。。。。。。。。。。。。。。。。。。。。。。。。。。。。。。。。。。。。。。。。。。。。。。。。。。。。。。
    //上边
    for ( int i = 0; i < rowMax / 4; i++, addFirst1 = 0 )  //   6     刚刚好
    {
        data = image.ptr< uchar >( i );
        for ( int j = 0; j < colMax; j++ ) {
            if ( data[ j ] == 255 ) {
                addFirst1++;
            }
        }
        if ( addFirst1 < addFirst )  //筛选最小值所在行
        {
            row1     = i;
            addFirst = addFirst1 + 1;
        }
    }

    //下边
    for ( int i = rowMax; i > rowMax - rowMax / 4; i--, addFirst2 = 0 )  //   6     刚刚好
    {
        data = image.ptr< uchar >( i );
        for ( int j = 2; j < colMax; j++ ) {
            if ( data[ j ] == 255 ) {
                addFirst2++;
            }
        }
        if ( addFirst2 < addFirstTemp )  //筛选最小值所在行
        {
            row2         = i;
            addFirstTemp = addFirst2 + 1;
        }
    }

    int orow;
    orow = row2 - row1;
    Mat w_image;
    w_image = image( Rect( 0, row1, colMax, orow ) );
    singleChar.push_back( w_image );
}

static void showChar()
{
    int length = singleChar.size();
    for ( int i = 0; i < length; i++ ) {

        resize( singleChar[ i ], singleChar[ i ], Size( 20, 40 ) );  //字符图像归一化

        // namedWindow(to_string(i) + "图");
        updateCameraImage( singleChar[ i ] );
    }
}

//读取省份模板
struct stu {
    Mat    image;
    double matchDegree;
};
EXT_RAM_ATTR struct stu first[ 35 ];

static void readProvince()
{
    int i = 0;
    //读取字符
    {
        first[ i ].image = imread( "match\\zw1.bmp", 0 );
        i++;
        first[ i ].image = imread( "match\\zw2.bmp", 0 );
        i++;
        first[ i ].image = imread( "match\\zw3.bmp", 0 );
        i++;
        first[ i ].image = imread( "match\\zw4.bmp", 0 );
        i++;
        first[ i ].image = imread( "match\\zw5.bmp", 0 );
        i++;
        first[ i ].image = imread( "match\\zw6.bmp", 0 );
        i++;
        first[ i ].image = imread( "match\\zw7.bmp", 0 );
        i++;
        first[ i ].image = imread( "match\\zw8.bmp", 0 );
        i++;
        first[ i ].image = imread( "match\\zw9.bmp", 0 );
        i++;
        first[ i ].image = imread( "match\\zw10.bmp", 0 );
        i++;
        first[ i ].image = imread( "match\\zw11.bmp", 0 );
        i++;
        first[ i ].image = imread( "match\\zw12.bmp", 0 );
        i++;
        first[ i ].image = imread( "match\\zw13.bmp", 0 );
        i++;
        first[ i ].image = imread( "match\\zw14.bmp", 0 );
        i++;
        first[ i ].image = imread( "match\\zw15.bmp", 0 );
        i++;
        first[ i ].image = imread( "match\\zw16.bmp", 0 );
        i++;
        first[ i ].image = imread( "match\\zw17.bmp", 0 );
        i++;
        first[ i ].image = imread( "match\\zw18.bmp", 0 );
        i++;
        first[ i ].image = imread( "match\\zw19.bmp", 0 );
        i++;
        first[ i ].image = imread( "match\\zw20.bmp", 0 );
        i++;
        first[ i ].image = imread( "match\\zw21.bmp", 0 );
        i++;
        first[ i ].image = imread( "match\\zw22.bmp", 0 );
        i++;
        first[ i ].image = imread( "match\\zw23.bmp", 0 );
        i++;
        first[ i ].image = imread( "match\\zw24.bmp", 0 );
        i++;
        first[ i ].image = imread( "match\\zw25.bmp", 0 );
        i++;
        first[ i ].image = imread( "match\\zw26.bmp", 0 );
        i++;
        first[ i ].image = imread( "match\\zw27.bmp", 0 );
        i++;
        first[ i ].image = imread( "match\\zw28.bmp", 0 );
        i++;
        first[ i ].image = imread( "match\\zw29.bmp", 0 );
        i++;
        first[ i ].image = imread( "match\\zw30.bmp", 0 );
        i++;
        first[ i ].image = imread( "match\\zw31.bmp", 0 );
        i++;
        first[ i ].image = imread( "match\\zw32.bmp", 0 );
        i++;
        first[ i ].image = imread( "match\\zw33.bmp", 0 );
        i++;
        first[ i ].image = imread( "match\\zw34.bmp", 0 );
        i++;
        first[ i ].image = imread( "match\\zw35.bmp", 0 );
    }
}

//识别省份字符
static void matchProvince()
{
    int    rowMax  = 40;
    int    colMax  = 20;
    int    add     = 0;
    int    addTemp = 0;
    Mat    absCutImage;
    double temp;
    int    index = 0;
    uchar* data;

    for ( int i = 0; i < rowMax; i++ ) {
        data = singleChar[ 0 ].ptr< uchar >( i );
        for ( int j = 0; j < colMax; j++ ) {
            if ( data[ j ] == 255 ) {
                add++;
            }
        }
    }

    int firstLength = end( first ) - begin( first );
    // printf("数组长度1         %d\n",firstLength);
    for ( int x = 0; x < firstLength; x++, addTemp = 0 ) {
        absCutImage = abs( first[ x ].image - singleChar[ 0 ] );

        for ( int i = 0; i < rowMax; i++ ) {
            data = absCutImage.ptr< uchar >( i );
            for ( int j = 0; j < colMax; j++ ) {
                if ( data[ j ] == 255 ) {
                    addTemp++;
                }
            }
        }
        temp = 1.0 - 1.0 * addTemp / add;
        if ( temp <= 1 )
            first[ x ].matchDegree = temp;
        else
            first[ x ].matchDegree = 0;

        if ( x > 0 && first[ x ].matchDegree > first[ index ].matchDegree )
            index = x;
    }

    // matchTemplate();

    /*absCutImage = abs(singleChar[0] - first[index].image);
    namedWindow("省份图片相减后图" + to_string(0));
    imshow("省份图片相减后图" + to_string(0), absCutImage);*/

    printf( "省份字符最大匹配度：  %lf\n", first[ index ].matchDegree );

    switch ( index ) {
    case 0:
        printf( "藏" );
        carPlateProvince += "藏";
        break;
    case 1:
        printf( "川" );
        carPlateProvince += "川";
        break;
    case 2:
        printf( "鄂" );
        carPlateProvince += "鄂";
        break;
    case 3:
        printf( "甘" );
        carPlateProvince += "甘";
        break;
    case 4:
        printf( "赣" );
        carPlateProvince += "赣";
        break;
    case 5:
        printf( "贵" );
        carPlateProvince += "贵";
        break;
    case 6:
        printf( "桂" );
        carPlateProvince += "桂";
        break;
    case 7:
        printf( "黑" );
        carPlateProvince += "黑";
        break;
    case 8:
        printf( "泸" );
        carPlateProvince += "泸";
        break;
    case 9:
        printf( "吉" );
        carPlateProvince += "吉";
        break;
    case 10:
        printf( "翼" );
        carPlateProvince += "翼";
        break;
    case 11:
        printf( "津" );
        carPlateProvince += "津";
        break;
    case 12:
        printf( "晋" );
        carPlateProvince += "晋";
        break;
    case 13:
        printf( "京" );
        carPlateProvince += "京";
        break;
    case 14:
        printf( "辽" );
        carPlateProvince += "辽";
        break;
    case 15:
        printf( "鲁" );
        carPlateProvince += "鲁";
        break;
    case 16:
        printf( "蒙" );
        carPlateProvince += "蒙";
        break;
    case 17:
        printf( "闽" );
        carPlateProvince += "闽";
        break;
    case 18:
        printf( "宁" );
        carPlateProvince += "宁";
        break;
    case 19:
        printf( "青" );
        carPlateProvince += "青";
        break;
    case 20:
        printf( "琼" );
        carPlateProvince += "琼";
        break;
    case 21:
        printf( "陕" );
        carPlateProvince += "陕";
        break;
    case 22:
        printf( "苏" );
        carPlateProvince += "苏";
        break;
    case 23:
        printf( "皖" );
        carPlateProvince += "皖";
        break;
    case 24:
        printf( "湘" );
        carPlateProvince += "湘";
        break;
    case 25:
        printf( "新" );
        carPlateProvince += "新";
        break;
    case 26:
        printf( "渝" );
        carPlateProvince += "渝";
        break;
    case 27:
        printf( "豫" );
        carPlateProvince += "豫";
        break;
    case 28:
        printf( "粤" );
        carPlateProvince += "粤";
        break;
    case 29:
        printf( "云" );
        carPlateProvince += "云";
        break;
    case 30:
        printf( "浙" );
        carPlateProvince += "浙";
        break;
    case 31:
        printf( "湘" );
        carPlateProvince += "湘";
        break;
    case 32:
        printf( "湘" );
        carPlateProvince += "湘";
        break;
    case 33:
        printf( "鲁" );
        carPlateProvince += "鲁";
        break;
    case 34:
        printf( "粤" );
        carPlateProvince += "粤";
        break;
    }
    printf( "\n" );
}

//读取字母和数字模板
struct stu1 {
    char   number;
    Mat    image;
    double matchDegree;
};
EXT_RAM_ATTR struct stu1 second[ 49 ];

static void readNumber()
{
    for ( int i = 0; i < 10; i++ ) {
        char match[ 128 ] = { 0 };
        sprintf( match, "match\\%d.bmp", i );
        second[ i ].image  = imread( match, 0 );
        second[ i ].number = 48 + i;
    }

    //读取字符
    {
        int i              = 10;
        second[ i ].image  = imread( "match\\6a.bmp", 0 );
        second[ i ].number = '6';
        i++;
        second[ i ].image  = imread( "match\\3a.bmp", 0 );
        second[ i ].number = '3';
        i++;
        second[ i ].image  = imread( "match\\P1.bmp", 0 );
        second[ i ].number = 'P';
        i++;
        second[ i ].image  = imread( "match\\8b.bmp", 0 );
        second[ i ].number = '8';
        i++;
        second[ i ].image  = imread( "match\\K1.bmp", 0 );
        second[ i ].number = 'K';
        i++;
        second[ i ].image  = imread( "match\\9a.bmp", 0 );
        second[ i ].number = '9';
        i++;
        second[ i ].image  = imread( "match\\B2.bmp", 0 );
        second[ i ].number = 'B';
        i++;
        second[ i ].image  = imread( "match\\G1.bmp", 0 );
        second[ i ].number = 'G';
        i++;
        second[ i ].image  = imread( "match\\T1.bmp", 0 );
        second[ i ].number = 'T';
        i++;
        second[ i ].image  = imread( "match\\B1.bmp", 0 );
        second[ i ].number = 'B';
        i++;
        second[ i ].image  = imread( "match\\8a.bmp", 0 );
        second[ i ].number = '8';
        i++;
        second[ i ].image  = imread( "match\\1a.bmp", 0 );
        second[ i ].number = '1';
        i++;
        second[ i ].image  = imread( "match\\7a.bmp", 0 );
        second[ i ].number = '7';
        i++;
        second[ i ].image  = imread( "match\\D1.bmp", 0 );
        second[ i ].number = 'D';
        i++;
        second[ i ].image  = imread( "match\\0a.bmp", 0 );
        second[ i ].number = '0';
        i++;
        second[ i ].image  = imread( "match\\A.bmp", 0 );
        second[ i ].number = 'A';
        i++;
        second[ i ].image  = imread( "match\\B.bmp", 0 );
        second[ i ].number = 'B';
        i++;
        second[ i ].image  = imread( "match\\C.bmp", 0 );
        second[ i ].number = 'C';
        i++;
        second[ i ].image  = imread( "match\\D.bmp", 0 );
        second[ i ].number = 'D';
        i++;
        second[ i ].image  = imread( "match\\E.bmp", 0 );
        second[ i ].number = 'E';
        i++;
        second[ i ].image  = imread( "match\\F.bmp", 0 );
        second[ i ].number = 'F';
        i++;
        second[ i ].image  = imread( "match\\G.bmp", 0 );
        second[ i ].number = 'G';
        i++;
        second[ i ].image  = imread( "match\\H.bmp", 0 );
        second[ i ].number = 'H';
        i++;
        second[ i ].image  = imread( "match\\J.bmp", 0 );
        second[ i ].number = 'J';
        i++;
        second[ i ].image  = imread( "match\\K.bmp", 0 );
        second[ i ].number = 'K';
        i++;
        second[ i ].image  = imread( "match\\L.bmp", 0 );
        second[ i ].number = 'L';
        i++;
        second[ i ].image  = imread( "match\\M.bmp", 0 );
        second[ i ].number = 'M';
        i++;
        second[ i ].image  = imread( "match\\N.bmp", 0 );
        second[ i ].number = 'N';
        i++;
        second[ i ].image  = imread( "match\\P.bmp", 0 );
        second[ i ].number = 'P';
        i++;
        second[ i ].image  = imread( "match\\Q.bmp", 0 );
        second[ i ].number = 'Q';
        i++;
        second[ i ].image  = imread( "match\\R.bmp", 0 );
        second[ i ].number = 'R';
        i++;
        second[ i ].image  = imread( "match\\S.bmp", 0 );
        second[ i ].number = 'S';
        i++;
        second[ i ].image  = imread( "match\\T.bmp", 0 );
        second[ i ].number = 'T';
        i++;
        second[ i ].image  = imread( "match\\U.bmp", 0 );
        second[ i ].number = 'U';
        i++;
        second[ i ].image  = imread( "match\\V.bmp", 0 );
        second[ i ].number = 'V';
        i++;
        second[ i ].image  = imread( "match\\W.bmp", 0 );
        second[ i ].number = 'W';
        i++;
        second[ i ].image  = imread( "match\\X.bmp", 0 );
        second[ i ].number = 'X';
        i++;
        second[ i ].image  = imread( "match\\Y.bmp", 0 );
        second[ i ].number = 'Y';
        i++;
        second[ i ].image  = imread( "match\\Z.bmp", 0 );
        second[ i ].number = 'Z';
    }
}

//识别其他字符
static void matchNumber()
{
    int    rowMax  = 40;
    int    colMax  = 20;
    int    add     = 0;
    int    addTemp = 0;
    Mat    absCutImage;
    double temp;
    int    index      = 0;
    int    charNum    = singleChar.size();
    int    charNumNum = end( second ) - begin( second );
    // printf("数组长度2         %d   \n", secondLength);

    uchar* data;
    int    q = 0;

    for ( int y = 1; y < charNum; y++, add = 0, index = 0 ) {
        if ( y > 6 )  //防止多读
            break;

        //统计要读取字符的白色像素总值
        for ( int i = 0; i < rowMax; i++ ) {
            data = singleChar[ y ].ptr< uchar >( i );
            for ( int j = 0; j < colMax; j++ ) {
                if ( data[ j ] == 255 ) {
                    add++;
                }
            }
        }

        //逐个字符识别
        for ( int x = 0; x < charNumNum; x++, addTemp = 0 ) {

            absCutImage = abs( singleChar[ y ] - second[ x ].image );

            //统计相减之后的图像白色像素总值
            for ( int i = 0; i < rowMax; i++ ) {
                data = absCutImage.ptr< uchar >( i );
                for ( int j = 0; j < colMax; j++ ) {
                    if ( data[ j ] == 255 ) {
                        addTemp++;
                    }
                }
            }
            temp = 1.0 - 1.0 * addTemp / add;
            if ( temp <= 1 && temp > 0 )
                second[ x ].matchDegree = temp;
            else
                second[ x ].matchDegree = 0;

            //获取最大匹配度对应索引index
            if ( x > 0 && second[ x ].matchDegree > second[ index ].matchDegree )
                index = x;
        }

        absCutImage = abs( singleChar[ y ] - second[ index ].image );
        /*	namedWindow("图片相减后图"+to_string(y));
            imshow("图片相减后图" + to_string(y), absCutImage);*/
        printf( "最大匹配度：  %lf\n", second[ index ].matchDegree );
        printf( "对应字符：    %c\n", second[ index ].number );
        test[ q ] = second[ index ].number;
        // printf("\ntest11111           %c\n", test[q]);
        q++;
    }
    test[ q ] = '\0';
}
#endif
#pragma GCC diagnostic pop
