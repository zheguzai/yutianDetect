//
//  ViewController.m
//  yutianDetect
//
//  Created by fred on 4/24/13.
//  Copyright (c) 2013 fred. All rights reserved.
//

#import "UIImage+OpenCV.h"
#import "ViewController.h"

#include "opencv2/core/core.hpp"
#include "opencv2/features2d/features2d.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/calib3d/calib3d.hpp"
//#include "opencv2/nonfree/features2d.hpp"
#include <sys/time.h>
#import <dispatch/dispatch.h>
#import <CoreMotion/CoreMotion.h>
#import <QuartzCore/QuartzCore.h>
#import <AudioToolbox/AudioToolbox.h>

typedef enum {
    DetectType_min = 0,
    DetectType_1,
    DetectType_2,
    DetectType_3,
    DetectType_4,
    DetectType_5,
    DetectType_max
} DetectType;

using namespace std;
using namespace cv;

@interface ViewController ()
- (BOOL)createCaptureSessionForCamera:(NSInteger)camera qualityPreset:(NSString *)qualityPreset grayscale:(BOOL)grayscale;
- (void)destroyCaptureSession;
- (void)processFrame:(cv::Mat&)mat videoRect:(CGRect)rect videoOrientation:(AVCaptureVideoOrientation)orientation;
@end

@implementation ViewController {
    DetectType _detectType;
    
    BOOL _isDebugging;// 是否是测试状态
    
    timeval _lastDetectImageTime;// 间隔为 0.5 秒，才开始检测图片
    
    NSOperationQueue *_detectQueue;
    
    double preAccelerationX_min;
    double preAccelerationY_min;
    double preAccelerationZ_min;
    double preAccelerationX_max;
    double preAccelerationY_max;
    double preAccelerationZ_max;
    
    cv::Mat _img_object;
    std::vector<KeyPoint> _keypoints_object;
    Mat _descriptors_object;
    
    SystemSoundID tapSoundID;
}

#pragma mark - init
- (id)initWithNibName:(NSString *)nibNameOrNil bundle:(NSBundle *)nibBundleOrNil {
    nibNameOrNil = ADDIPADSUFFIX(nibNameOrNil);
    
    self = [super initWithNibName:nibNameOrNil bundle:nibBundleOrNil];
    if (self) {
        _isDebugging = YES;
        
        timeval timeNow;
        gettimeofday(&timeNow, NULL);
        self.lastMotionTime = [NSNumber numberWithDouble:timeNow.tv_sec + timeNow.tv_usec / 1000 / 1000.0];
        
        gettimeofday(&_lastDetectImageTime, NULL);
        
        _detectQueue = [[NSOperationQueue alloc] init];
        _detectQueue.maxConcurrentOperationCount = 4;
        
        preAccelerationX_min = 1000;
        preAccelerationY_min = 1000;
        preAccelerationZ_min = 1000;
        preAccelerationX_max = -1000;
        preAccelerationY_max = -1000;
        preAccelerationZ_max = -1000;
        
        _detectType = DetectType_2;
        if (![self prepareImageObject]) {
            exit(0);
        }
        
        NSURL *soundFilePath = [[NSBundle mainBundle] URLForResource:@"endSound" withExtension: @"wav"];
        AudioServicesCreateSystemSoundID((CFURLRef)soundFilePath, &tapSoundID);
        
        self.detectTimes = [NSNumber numberWithInt:0];
    }
    return self;
}
- (void)viewDidLoad {
    [super viewDidLoad];
    
    _imageView_scene.layer.transform =  CATransform3DMakeRotation(M_PI/2, 0, 0, 1);
    
    [self createCaptureSessionForCamera:0 qualityPreset:AVCaptureSessionPresetMedium grayscale:YES];
    [_captureSession startRunning];
}
- (BOOL)prepareImageObject {
    NSString *filePath = [[NSBundle mainBundle] pathForResource:@"buddha" ofType:@"png"];
    _img_object = cv::imread([filePath UTF8String], CV_LOAD_IMAGE_GRAYSCALE);
    if (!_img_object.data) {
        return NO;
    }
    
    switch (_detectType) {
        case DetectType_1: {
            //-- Step 1 -- Step 2: 检测关键点，获取 descriptor
            ORB orb_object(30,ORB::CommonParams(1.2,1));
            orb_object(_img_object, Mat(), _keypoints_object, _descriptors_object, false);
            if (_keypoints_object.size() <= 0) {
                return NO;
            }
            break;
        }
        case DetectType_2: {
            //-- Step 1: Detect the keypoints using SURF Detector
            FastFeatureDetector detector(15);
            detector.detect( _img_object, _keypoints_object );
            if (_keypoints_object.size() <= 0) {
                return NO;
            }
            
            //-- Step 2: Calculate descriptors (feature vectors)
            BriefDescriptorExtractor extractor;
            extractor.compute( _img_object, _keypoints_object, _descriptors_object );
            break;
        }
        case DetectType_3: {
            //-- Step 1: Detect the keypoints using SURF Detector
            FastFeatureDetector detector(15);
            detector.detect( _img_object, _keypoints_object );
            if (_keypoints_object.size() <= 0) {
                return NO;
            }
            
            //-- Step 2: Calculate descriptors (feature vectors)
            SurfDescriptorExtractor extractor;
            extractor.compute( _img_object, _keypoints_object, _descriptors_object );
            break;
        }
        case DetectType_4: {
            //-- Step 1: Detect the keypoints using SURF Detector
            int minHessian = 1600; //最小 400，命中率高，速度慢
            SurfFeatureDetector detector( minHessian );
            detector.detect( _img_object, _keypoints_object );
            if (_keypoints_object.size() == 0) {
                return NO;
            }
            
            //-- Step 2: Calculate descriptors (feature vectors)
            SurfDescriptorExtractor extractor;
            extractor.compute( _img_object, _keypoints_object, _descriptors_object );
            break;
        }
        case DetectType_5: {
            //-- Step 1: Detect the keypoints using SURF Detector
            int minHessian = 1600; //最小 400，命中率高，速度慢
            SurfFeatureDetector detector( minHessian );
            detector.detect( _img_object, _keypoints_object );
            if (_keypoints_object.size() == 0) {
                return NO;
            }
            
            //-- Step 2: Calculate descriptors (feature vectors)
            SurfDescriptorExtractor extractor;
            extractor.compute( _img_object, _keypoints_object, _descriptors_object );
            break;
        }
        default: {
            return NO;
        }
    }
    
    return YES;
}
#pragma mark - 识别获取到的 mat 数据
- (void)processFrame:(cv::Mat&)mat videoRect:(CGRect)rect videoOrientation:(AVCaptureVideoOrientation)orientation {
    [self detectImage:mat];
}
- (void)detectImage:(cv::Mat&)mat {
    timeval timeNow;
    gettimeofday(&timeNow, NULL);
    double elapsedTime;
    elapsedTime = (timeNow.tv_sec - _lastDetectImageTime.tv_sec) * 1000.0;
    elapsedTime += (timeNow.tv_usec - _lastDetectImageTime.tv_usec) / 1000.0;
    
    if (elapsedTime < 500 || [_detectQueue operationCount] >= [_detectQueue maxConcurrentOperationCount]) {
        return;
    } else {
        _lastDetectImageTime = timeNow;
    }
    
    if (_isDebugging) {
        UIImage *image_object = [UIImage imageNamed:@"buddha.png"];
        UIImage *image_scene = [UIImage imageWithCVMat:mat];
        
        dispatch_async(dispatch_get_main_queue(), ^{
            _imageView_object.image = image_object;
            _imageView_scene.image = image_scene;
        });
    }
    
    cv::transpose(mat, mat);
    cv::flip(mat, mat, 1);
    
    NSDictionary *argumentDictionary = [NSDictionary dictionaryWithObjectsAndKeys:
                                        [UIImage imageWithCVMat:_img_object], @"img_object",
                                        [UIImage imageWithCVMat:mat], @"img_scene", nil];
    NSInvocationOperation *invocaitonOp = nil;
    
    switch (_detectType) {
        case DetectType_1: {
            invocaitonOp = [[[NSInvocationOperation alloc] initWithTarget:self
                                                                 selector:@selector(detectImage_1:)
                                                                   object:argumentDictionary] autorelease];
            [_detectQueue addOperation:invocaitonOp];
            break;
        }
        case DetectType_2: {
            invocaitonOp = [[[NSInvocationOperation alloc] initWithTarget:self
                                                                 selector:@selector(detectImage_2:)
                                                                   object:argumentDictionary] autorelease];
            [_detectQueue addOperation:invocaitonOp];
            break;
        }
        case DetectType_3: {
            invocaitonOp = [[[NSInvocationOperation alloc] initWithTarget:self
                                                                 selector:@selector(detectImage_3:)
                                                                   object:argumentDictionary] autorelease];
            [_detectQueue addOperation:invocaitonOp];
            break;
        }
        case DetectType_4: {
            invocaitonOp = [[[NSInvocationOperation alloc] initWithTarget:self
                                                                 selector:@selector(detectImage_4:)
                                                                   object:argumentDictionary] autorelease];
            [_detectQueue addOperation:invocaitonOp];
            break;
        }
        case DetectType_5: {
            invocaitonOp = [[[NSInvocationOperation alloc] initWithTarget:self
                                                                 selector:@selector(detectImage_5:)
                                                                   object:argumentDictionary] autorelease];
            [_detectQueue addOperation:invocaitonOp];
            break;
        }
        default: {
            break;
        }
    }
}

#pragma mark - ORB BruteForceMatcher
- (BOOL)detectImage_1:(NSDictionary *)argumentDictionary {
    if ([self shouldStopDetect]) {return NO;}
    cv::Mat img_scene = [((UIImage *)[argumentDictionary objectForKey:@"img_scene"]) CVGrayscaleMat];
    
    timeval taskStartTime, taskStopTime;
    double elapsedTime;
    gettimeofday(&taskStartTime, NULL);
    
    if(!img_scene.data) {
        return NO;
    }
    
    //-- Step 1 -- Step 2: 检测关键点，获取 descriptor
    ORB orb_scene(100,ORB::CommonParams(1.2,1));
    std::vector<KeyPoint> keypoints_scene;
    Mat descriptors_scene;
    
    if ([self shouldStopDetect]) {return NO;}
    orb_scene(img_scene, Mat(), keypoints_scene);
    if (keypoints_scene.size() <= 0) {
        return NO;
    }
    if ([self shouldStopDetect]) {return NO;}
    orb_scene(img_scene, Mat(), keypoints_scene, descriptors_scene, true);
    
    //-- Step 3: Matching descriptor vectors using FLANN matcher
    BruteForceMatcher<HammingLUT>matcher;
    std::vector< DMatch > matches;
    if ([self shouldStopDetect]) {return NO;}
    matcher.match( _descriptors_object, descriptors_scene, matches );
    if (matches.size() == 0) {
        return NO;
    }
    
    //-- Quick calculation of max and min distances between keypoints
    double max_dist = 0; double min_dist = 100;
    for( int i = 0; i < _descriptors_object.rows; i++ ) {
        double dist = matches[i].distance;
        if( dist < min_dist ) {
            min_dist = dist;
        }
        if( dist > max_dist ) {
            max_dist = dist;
        }
    }
    
    //-- Draw only "good" matches (i.e. whose distance is less than 3*min_dist )
    std::vector< DMatch > good_matches;
    for( int i = 0; i < _descriptors_object.rows; i++ ) {
        if( matches[i].distance < 3*min_dist ) {
            good_matches.push_back( matches[i]);
        }
    }
    if (good_matches.size() == 0) {
        return NO;
    }
    
    //-- Localize the object from img_1 in img_2
    std::vector<Point2f> obj;
    std::vector<Point2f> scene;
    for( size_t i = 0; i < good_matches.size(); i++ )
    {
        //-- Get the keypoints from the good matches
        obj.push_back( _keypoints_object[ good_matches[i].queryIdx ].pt );
        scene.push_back( keypoints_scene[ good_matches[i].trainIdx ].pt );
    }
    if (obj.size() < 4 || scene.size() < 4) {
        return NO;
    }
    
    if ([self shouldStopDetect]) {return NO;}
    Mat H = findHomography( obj, scene, CV_RANSAC );
    
    //-- Get the corners from the image_1 ( the object to be "detected" )
    std::vector<Point2f> obj_corners(4);
    obj_corners[0] = cvPoint(0,0);
    obj_corners[1] = cvPoint( _img_object.cols, 0 );
    obj_corners[2] = cvPoint( _img_object.cols, _img_object.rows );
    obj_corners[3] = cvPoint( 0, _img_object.rows );
    
    // 判断点是否是有效点
    bool exist = 1;
    std::vector<Point2f> scene_corners(4);
    perspectiveTransform( obj_corners, scene_corners, H);
    for (int j = 0; j < 4; j++) {
        if (cv::norm(scene_corners[(j+1) % 4] - scene_corners[j]) < 40) {
            exist = 0;
            break;
        }
    }
    
    if (_isDebugging) {
        if ([self shouldStopDetect]) {return NO;}
        // 画关键点
        Mat img_matches;
        drawMatches( _img_object, _keypoints_object, img_scene, keypoints_scene,
                    good_matches, img_matches, Scalar::all(-1), Scalar::all(-1),
                    vector<char>(), DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS );
        //-- Draw lines between the corners (the mapped object in the scene - image_2 )
        Point2f offset( (float)_img_object.cols, 0);
        line( img_matches, scene_corners[0] + offset, scene_corners[1] + offset, Scalar( 0, 255, 0), 4 );
        line( img_matches, scene_corners[1] + offset, scene_corners[2] + offset, Scalar( 0, 255, 0), 4 );
        line( img_matches, scene_corners[2] + offset, scene_corners[3] + offset, Scalar( 0, 255, 0), 4 );
        line( img_matches, scene_corners[3] + offset, scene_corners[0] + offset, Scalar( 0, 255, 0), 4 );
        [self showResultImage:[UIImage imageWithCVMat:img_matches]];
    }
    
    if (exist) {
        float ratio_1 = cv::norm(obj_corners[1] - obj_corners[0]) / cv::norm(scene_corners[1] - scene_corners[0]);
        for (int j = 1; j < 4; j++) {
            float ratio_2 = cv::norm(obj_corners[(j+1) % 4] - obj_corners[j]) / cv::norm(scene_corners[(j+1) % 4] - scene_corners[j]);
            if (ratio_2 / ratio_1 > 1.2 || ratio_2 / ratio_1 < 0.8) {
                exist = 0;
                break;
            }
        }
    }
    
    if (exist) {
        gettimeofday(&taskStopTime, NULL);
        elapsedTime = (taskStopTime.tv_sec - taskStartTime.tv_sec) * 1000.0;
        elapsedTime += (taskStopTime.tv_usec - taskStartTime.tv_usec) / 1000.0;
        std::cout << "总耗时 " << elapsedTime << endl;
        
        [self hasDetected:[NSString stringWithFormat:@"%s %f", sel_getName(_cmd), elapsedTime]];
        
        cout<< "四个顶点" << scene_corners[0] << " " << scene_corners[1] << " " << scene_corners[2] << " " << scene_corners[3] << endl;
        [self showResultCoverViwe:CGRectMake(scene_corners[0].x,
                                             scene_corners[0].y,
                                             scene_corners[1].x - scene_corners[0].x,
                                             scene_corners[2].y - scene_corners[1].y)];
    }
    
    return exist;
}

#pragma mark - FastFeatureDetector BriefDescriptorExtractor BruteForceMatcher
- (BOOL)detectImage_2:(NSDictionary *)argumentDictionary {
    if ([self shouldStopDetect]) {return NO;}
    cv::Mat img_scene = [((UIImage *)[argumentDictionary objectForKey:@"img_scene"]) CVGrayscaleMat];
    
    timeval taskStartTime, taskStopTime;
    double elapsedTime;
    gettimeofday(&taskStartTime, NULL);
    
    if( !img_scene.data ) {
        return NO;
    }
    
    //-- Step 1: Detect the keypoints using SURF Detector
    FastFeatureDetector detector(15);
    std::vector<KeyPoint> keypoints_scene;
    if ([self shouldStopDetect]) {return NO;}
    detector.detect( img_scene, keypoints_scene );
    if (keypoints_scene.size() == 0) {
        return NO;
    }
    
    //-- Step 2: Calculate descriptors (feature vectors)
    BriefDescriptorExtractor extractor;
    Mat descriptors_scene;
    if ([self shouldStopDetect]) {return NO;}
    extractor.compute( img_scene, keypoints_scene, descriptors_scene );
    
    //-- Step 3: Matching descriptor vectors using FLANN matcher
    BruteForceMatcher< Hamming > matcher;
    std::vector< DMatch > matches;
    if ([self shouldStopDetect]) {return NO;}
    matcher.match( _descriptors_object, descriptors_scene, matches );
    if (matches.size() == 0) {
        return NO;
    }
    
    //-- Quick calculation of max and min distances between keypoints
    if ([self shouldStopDetect]) {return NO;}
    double max_dist = 0; double min_dist = 100;
    for( int i = 0; i < _descriptors_object.rows; i++ ) {
        double dist = matches[i].distance;
        if( dist < min_dist ) {
            min_dist = dist;
        }
        if( dist > max_dist ) {
            max_dist = dist;
        }
    }
    
    //-- Draw only "good" matches (i.e. whose distance is less than 3*min_dist )
    if ([self shouldStopDetect]) {return NO;}
    std::vector< DMatch > good_matches;
    for( int i = 0; i < _descriptors_object.rows; i++ ) {
        if( matches[i].distance < 3*min_dist ) {
            good_matches.push_back( matches[i]);
        }
    }
    if (good_matches.size() == 0) {
        return NO;
    }
    
    //-- Localize the object from img_1 in img_2
    if ([self shouldStopDetect]) {return NO;}
    std::vector<Point2f> obj;
    std::vector<Point2f> scene;
    for( size_t i = 0; i < good_matches.size(); i++ )
    {
        //-- Get the keypoints from the good matches
        obj.push_back( _keypoints_object[ good_matches[i].queryIdx ].pt );
        scene.push_back( keypoints_scene[ good_matches[i].trainIdx ].pt );
    }
    if (obj.size() < 4 || scene.size() < 4) {
        return NO;
    }
    
    if ([self shouldStopDetect]) {return NO;}
    Mat H = findHomography( obj, scene, CV_RANSAC );
    
    //-- Get the corners from the image_1 ( the object to be "detected" )
    std::vector<Point2f> obj_corners(4);
    obj_corners[0] = cvPoint(0,0);
    obj_corners[1] = cvPoint( _img_object.cols, 0 );
    obj_corners[2] = cvPoint( _img_object.cols, _img_object.rows );
    obj_corners[3] = cvPoint( 0, _img_object.rows );
    
    // 判断点是否是有效点
    bool exist = 1;
    std::vector<Point2f> scene_corners(4);
    perspectiveTransform( obj_corners, scene_corners, H);
    for (int j = 0; j < 4; j++) {
        if (cv::norm(scene_corners[(j+1) % 4] - scene_corners[j]) < 40) {
            exist = 0;
            break;
        }
    }
    
    if (_isDebugging) {
        if ([self shouldStopDetect]) {return NO;}
        // 画关键点
        Mat img_matches;
        drawMatches( _img_object, _keypoints_object, img_scene, keypoints_scene,
                    good_matches, img_matches, Scalar::all(-1), Scalar::all(-1),
                    vector<char>(), DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS );
        //-- Draw lines between the corners (the mapped object in the scene - image_2 )
        Point2f offset( (float)_img_object.cols, 0);
        line( img_matches, scene_corners[0] + offset, scene_corners[1] + offset, Scalar( 0, 255, 0), 4 );
        line( img_matches, scene_corners[1] + offset, scene_corners[2] + offset, Scalar( 0, 255, 0), 4 );
        line( img_matches, scene_corners[2] + offset, scene_corners[3] + offset, Scalar( 0, 255, 0), 4 );
        line( img_matches, scene_corners[3] + offset, scene_corners[0] + offset, Scalar( 0, 255, 0), 4 );
        [self showResultImage:[UIImage imageWithCVMat:img_matches]];
    }
    
    if (exist) {
        
        float ratio_1 = cv::norm(obj_corners[1] - obj_corners[0]) / cv::norm(scene_corners[1] - scene_corners[0]);
        for (int j = 1; j < 4; j++) {
            float ratio_2 = cv::norm(obj_corners[(j+1) % 4] - obj_corners[j]) / cv::norm(scene_corners[(j+1) % 4] - scene_corners[j]);
            if (ratio_2 / ratio_1 > 1.2 || ratio_2 / ratio_1 < 0.8) {
                exist = 0;
                break;
            }
        }
    }
    
    if (exist) {
        gettimeofday(&taskStopTime, NULL);
        elapsedTime = (taskStopTime.tv_sec - taskStartTime.tv_sec) * 1000.0;
        elapsedTime += (taskStopTime.tv_usec - taskStartTime.tv_usec) / 1000.0;
        //        std::cout << "总耗时 " << elapsedTime << endl;
        
        [self hasDetected:[NSString stringWithFormat:@"%s %f", sel_getName(_cmd), elapsedTime]];
        
        //        cout<< "四个顶点" << scene_corners[0] << " " << scene_corners[1] << " " << scene_corners[2] << " " << scene_corners[3] << endl;
        [self showResultCoverViwe:
         CGRectMake(
                    scene_corners[0].x,
                    (scene_corners[0].y + scene_corners[1].y) / 2,
                    //                    (scene_corners[1].x + scene_corners[2].x - scene_corners[0].x - scene_corners[3].x) / 2,
                    //                    (scene_corners[2].y + scene_corners[3].y + scene_corners[0].y - scene_corners[1].y) / 2)];
                    scene_corners[1].x - scene_corners[0].x,
                    scene_corners[2].y - scene_corners[1].y)];
    }
    
    return exist;
}

#pragma mark - FastFeatureDetector SurfDescriptorExtractor FlannBasedMatcher
- (BOOL)detectImage_3:(NSDictionary *)argumentDictionary {
    if ([self shouldStopDetect]) {return NO;}
    cv::Mat img_scene = [((UIImage *)[argumentDictionary objectForKey:@"img_scene"]) CVGrayscaleMat];
    
    timeval taskStartTime, taskStopTime;
    double elapsedTime;
    gettimeofday(&taskStartTime, NULL);
    
    if( !img_scene.data ) {
        return NO;
    }
    
    //-- Step 1: Detect the keypoints using SURF Detector
    FastFeatureDetector detector(15);
    std::vector<KeyPoint> keypoints_scene;
    if ([self shouldStopDetect]) {return NO;}
    detector.detect( img_scene, keypoints_scene );
    if (keypoints_scene.size() == 0) {
        return NO;
    }
    
    //-- Step 2: Calculate descriptors (feature vectors)
    SurfDescriptorExtractor extractor;
    Mat descriptors_scene;
    if ([self shouldStopDetect]) {return NO;}
    extractor.compute( img_scene, keypoints_scene, descriptors_scene );
    
    //-- Step 3: Matching descriptor vectors using FLANN matcher
    FlannBasedMatcher matcher;
    std::vector< DMatch > matches;
    if ([self shouldStopDetect]) {return NO;}
    matcher.match( _descriptors_object, descriptors_scene, matches );
    if (matches.size() == 0) {
        return NO;
    }
    
    //-- Quick calculation of max and min distances between keypoints
    double max_dist = 0; double min_dist = 100;
    if ([self shouldStopDetect]) {return NO;}
    for( int i = 0; i < _descriptors_object.rows; i++ ) {
        double dist = matches[i].distance;
        if( dist < min_dist ) {
            min_dist = dist;
        }
        if( dist > max_dist ) {
            max_dist = dist;
        }
    }
    
    //-- Draw only "good" matches (i.e. whose distance is less than 3*min_dist )
    std::vector< DMatch > good_matches;
    if ([self shouldStopDetect]) {return NO;}
    for( int i = 0; i < _descriptors_object.rows; i++ ) {
        if( matches[i].distance < 3*min_dist ) {
            good_matches.push_back( matches[i]);
        }
    }
    if (good_matches.size() == 0) {
        return NO;
    }
    
    //-- Localize the object from img_1 in img_2
    std::vector<Point2f> obj;
    std::vector<Point2f> scene;
    if ([self shouldStopDetect]) {return NO;}
    for( size_t i = 0; i < good_matches.size(); i++ )
    {
        //-- Get the keypoints from the good matches
        obj.push_back( _keypoints_object[ good_matches[i].queryIdx ].pt );
        scene.push_back( keypoints_scene[ good_matches[i].trainIdx ].pt );
    }
    if (obj.size() < 4 || scene.size() < 4) {
        return NO;
    }
    
    if ([self shouldStopDetect]) {return NO;}
    Mat H = findHomography( obj, scene, CV_RANSAC );
    
    //-- Get the corners from the image_1 ( the object to be "detected" )
    std::vector<Point2f> obj_corners(4);
    obj_corners[0] = cvPoint(0,0);
    obj_corners[1] = cvPoint( _img_object.cols, 0 );
    obj_corners[2] = cvPoint( _img_object.cols, _img_object.rows );
    obj_corners[3] = cvPoint( 0, _img_object.rows );
    
    // 判断点是否是有效点
    bool exist = 1;
    std::vector<Point2f> scene_corners(4);
    perspectiveTransform( obj_corners, scene_corners, H);
    for (int j = 0; j < 4; j++) {
        if (cv::norm(scene_corners[(j+1) % 4] - scene_corners[j]) < 40) {
            exist = 0;
            break;
        }
    }
    
    if (_isDebugging) {
        if ([self shouldStopDetect]) {return NO;}
        // 画关键点
        Mat img_matches;
        drawMatches( _img_object, _keypoints_object, img_scene, keypoints_scene,
                    good_matches, img_matches, Scalar::all(-1), Scalar::all(-1),
                    vector<char>(), DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS );
        //-- Draw lines between the corners (the mapped object in the scene - image_2 )
        Point2f offset( (float)_img_object.cols, 0);
        line( img_matches, scene_corners[0] + offset, scene_corners[1] + offset, Scalar( 0, 255, 0), 4 );
        line( img_matches, scene_corners[1] + offset, scene_corners[2] + offset, Scalar( 0, 255, 0), 4 );
        line( img_matches, scene_corners[2] + offset, scene_corners[3] + offset, Scalar( 0, 255, 0), 4 );
        line( img_matches, scene_corners[3] + offset, scene_corners[0] + offset, Scalar( 0, 255, 0), 4 );
        [self showResultImage:[UIImage imageWithCVMat:img_matches]];
    }
    
    if (exist) {
        float ratio_1 = cv::norm(obj_corners[1] - obj_corners[0]) / cv::norm(scene_corners[1] - scene_corners[0]);
        for (int j = 1; j < 4; j++) {
            float ratio_2 = cv::norm(obj_corners[(j+1) % 4] - obj_corners[j]) / cv::norm(scene_corners[(j+1) % 4] - scene_corners[j]);
            if (ratio_2 / ratio_1 > 1.2 || ratio_2 / ratio_1 < 0.8) {
                exist = 0;
                break;
            }
        }
    }
    
    if (exist) {
        gettimeofday(&taskStopTime, NULL);
        elapsedTime = (taskStopTime.tv_sec - taskStartTime.tv_sec) * 1000.0;
        elapsedTime += (taskStopTime.tv_usec - taskStartTime.tv_usec) / 1000.0;
        std::cout << "总耗时 " << elapsedTime << endl;
        
        [self hasDetected:[NSString stringWithFormat:@"%s %f", sel_getName(_cmd), elapsedTime]];
        
        cout<< "四个顶点" << scene_corners[0] << " " << scene_corners[1] << " " << scene_corners[2] << " " << scene_corners[3] << endl;
        [self showResultCoverViwe:CGRectMake(scene_corners[0].x,
                                             scene_corners[0].y,
                                             scene_corners[1].x - scene_corners[0].x,
                                             scene_corners[2].y - scene_corners[1].y)];
    }
    
    return exist;
}

#pragma mark - SurfFeatureDetector SurfDescriptorExtractor FlannBasedMatcher
- (BOOL)detectImage_4:(NSDictionary *)argumentDictionary {
    if ([self shouldStopDetect]) {return NO;}
    cv::Mat img_scene = [((UIImage *)[argumentDictionary objectForKey:@"img_scene"]) CVGrayscaleMat];
    
    timeval taskStartTime, taskStopTime;
    double elapsedTime;
    gettimeofday(&taskStartTime, NULL);
    
    if(!img_scene.data ) {
        return NO;
    }
    
    //-- Step 1: Detect the keypoints using SURF Detector
    int minHessian = 1600; //最小 400，命中率高，速度慢
    SurfFeatureDetector detector( minHessian );
    std::vector<KeyPoint> keypoints_scene;
    if ([self shouldStopDetect]) {return NO;}
    detector.detect( img_scene, keypoints_scene );
    if (keypoints_scene.size() == 0) {
        return NO;
    }
    
    //-- Step 2: Calculate descriptors (feature vectors)
    SurfDescriptorExtractor extractor;
    Mat descriptors_scene;
    if ([self shouldStopDetect]) {return NO;}
    extractor.compute( img_scene, keypoints_scene, descriptors_scene );
    
    //-- Step 3: Matching descriptor vectors using FLANN matcher
    FlannBasedMatcher matcher;
    std::vector< DMatch > matches;
    if ([self shouldStopDetect]) {return NO;}
    matcher.match( _descriptors_object, descriptors_scene, matches );
    if (matches.size() == 0) {
        return NO;
    }
    
    //-- Quick calculation of max and min distances between keypoints
    double max_dist = 0; double min_dist = 100;
    if ([self shouldStopDetect]) {return NO;}
    for( int i = 0; i < _descriptors_object.rows; i++ ) {
        double dist = matches[i].distance;
        if( dist < min_dist ) {
            min_dist = dist;
        }
        if( dist > max_dist ) {
            max_dist = dist;
        }
    }
    
    //-- Draw only "good" matches (i.e. whose distance is less than 3*min_dist )
    std::vector< DMatch > good_matches;
    if ([self shouldStopDetect]) {return NO;}
    for( int i = 0; i < _descriptors_object.rows; i++ ) {
        if( matches[i].distance < 3*min_dist ) {
            good_matches.push_back( matches[i]);
        }
    }
    if (good_matches.size() == 0) {
        return NO;
    }
    
    //-- Localize the object from img_1 in img_2
    std::vector<Point2f> obj;
    std::vector<Point2f> scene;
    if ([self shouldStopDetect]) {return NO;}
    for( size_t i = 0; i < good_matches.size(); i++ )
    {
        //-- Get the keypoints from the good matches
        obj.push_back( _keypoints_object[ good_matches[i].queryIdx ].pt );
        scene.push_back( keypoints_scene[ good_matches[i].trainIdx ].pt );
    }
    if (obj.size() < 4 || scene.size() < 4) {
        return NO;
    }
    
    if ([self shouldStopDetect]) {return NO;}
    Mat H = findHomography( obj, scene, CV_RANSAC );
    
    //-- Get the corners from the image_1 ( the object to be "detected" )
    std::vector<Point2f> obj_corners(4);
    obj_corners[0] = cvPoint(0,0);
    obj_corners[1] = cvPoint( _img_object.cols, 0 );
    obj_corners[2] = cvPoint( _img_object.cols, _img_object.rows );
    obj_corners[3] = cvPoint( 0, _img_object.rows );
    
    // 判断点是否是有效点
    bool exist = 1;
    std::vector<Point2f> scene_corners(4);
    if ([self shouldStopDetect]) {return NO;}
    perspectiveTransform( obj_corners, scene_corners, H);
    for (int j = 0; j < 4; j++) {
        if (cv::norm(scene_corners[(j+1) % 4] - scene_corners[j]) < 40) {
            exist = 0;
            break;
        }
    }
    
    if (_isDebugging) {
        if ([self shouldStopDetect]) {return NO;}
        // 画关键点
        Mat img_matches;
        drawMatches( _img_object, _keypoints_object, img_scene, keypoints_scene,
                    good_matches, img_matches, Scalar::all(-1), Scalar::all(-1),
                    vector<char>(), DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS );
        //-- Draw lines between the corners (the mapped object in the scene - image_2 )
        Point2f offset( (float)_img_object.cols, 0);
        line( img_matches, scene_corners[0] + offset, scene_corners[1] + offset, Scalar( 0, 255, 0), 4 );
        line( img_matches, scene_corners[1] + offset, scene_corners[2] + offset, Scalar( 0, 255, 0), 4 );
        line( img_matches, scene_corners[2] + offset, scene_corners[3] + offset, Scalar( 0, 255, 0), 4 );
        line( img_matches, scene_corners[3] + offset, scene_corners[0] + offset, Scalar( 0, 255, 0), 4 );
        [self showResultImage:[UIImage imageWithCVMat:img_matches]];
    }
    
    if (exist) {
        float ratio_1 = cv::norm(obj_corners[1] - obj_corners[0]) / cv::norm(scene_corners[1] - scene_corners[0]);
        for (int j = 1; j < 4; j++) {
            float ratio_2 = cv::norm(obj_corners[(j+1) % 4] - obj_corners[j]) / cv::norm(scene_corners[(j+1) % 4] - scene_corners[j]);
            if (ratio_2 / ratio_1 > 1.2 || ratio_2 / ratio_1 < 0.8) {
                exist = 0;
                break;
            }
        }
    }
    
    if (exist) {
        gettimeofday(&taskStopTime, NULL);
        elapsedTime = (taskStopTime.tv_sec - taskStartTime.tv_sec) * 1000.0;
        elapsedTime += (taskStopTime.tv_usec - taskStartTime.tv_usec) / 1000.0;
        std::cout << "总耗时 " << elapsedTime << endl;
        
        [self hasDetected:[NSString stringWithFormat:@"%s %f", sel_getName(_cmd), elapsedTime]];
        
        cout<< "四个顶点" << scene_corners[0] << " " << scene_corners[1] << " " << scene_corners[2] << " " << scene_corners[3] << endl;
        [self showResultCoverViwe:CGRectMake(scene_corners[0].x,
                                             scene_corners[0].y,
                                             scene_corners[1].x - scene_corners[0].x,
                                             scene_corners[2].y - scene_corners[1].y)];
    }
    
    return exist;
}

#pragma mark - SurfFeatureDetector SurfDescriptorExtractor BruteForceMatcher
- (BOOL)detectImage_5:(NSDictionary *)argumentDictionary {
    if ([self shouldStopDetect]) {return NO;}
    cv::Mat img_object = [((UIImage *)[argumentDictionary objectForKey:@"img_object"]) CVGrayscaleMat];
    if ([self shouldStopDetect]) {return NO;}
    cv::Mat img_scene = [((UIImage *)[argumentDictionary objectForKey:@"img_scene"]) CVGrayscaleMat];
    
    timeval taskStartTime, taskStopTime;
    double elapsedTime;
    gettimeofday(&taskStartTime, NULL);
    
    if( !img_object.data || !img_scene.data ) {
        return NO;
    }
    
    //-- Step 1: Detect the keypoints using SURF Detector
    int minHessian = 1600; //最小 400，命中率高，速度慢
    SurfFeatureDetector detector( minHessian );
    std::vector<KeyPoint> keypoints_object, keypoints_scene;
    if ([self shouldStopDetect]) {return NO;}
    detector.detect( img_object, keypoints_object );
    if ([self shouldStopDetect]) {return NO;}
    detector.detect( img_scene, keypoints_scene );
    if (keypoints_object.size() == 0 || keypoints_scene.size() == 0) {
        return NO;
    }
    
    //-- Step 2: Calculate descriptors (feature vectors)
    SurfDescriptorExtractor extractor;
    Mat descriptors_object, descriptors_scene;
    if ([self shouldStopDetect]) {return NO;}
    extractor.compute( img_object, keypoints_object, descriptors_object );
    if ([self shouldStopDetect]) {return NO;}
    extractor.compute( img_scene, keypoints_scene, descriptors_scene );
    
    //-- Step 3: Matching descriptor vectors using FLANN matcher
    BruteForceMatcher< L2<float> > matcher;
    std::vector< DMatch > matches;
    if ([self shouldStopDetect]) {return NO;}
    matcher.match( descriptors_object, descriptors_scene, matches );
    if (matches.size() == 0) {
        return NO;
    }
    
    //-- Quick calculation of max and min distances between keypoints
    if ([self shouldStopDetect]) {return NO;}
    double max_dist = 0; double min_dist = 100;
    for( int i = 0; i < descriptors_object.rows; i++ ) {
        double dist = matches[i].distance;
        if( dist < min_dist ) {
            min_dist = dist;
        }
        if( dist > max_dist ) {
            max_dist = dist;
        }
    }
    
    //-- Draw only "good" matches (i.e. whose distance is less than 3*min_dist )
    if ([self shouldStopDetect]) {return NO;}
    std::vector< DMatch > good_matches;
    for( int i = 0; i < descriptors_object.rows; i++ ) {
        if( matches[i].distance < 3*min_dist ) {
            good_matches.push_back( matches[i]);
        }
    }
    if (good_matches.size() == 0) {
        return NO;
    }
    
    //-- Localize the object from img_1 in img_2
    if ([self shouldStopDetect]) {return NO;}
    std::vector<Point2f> obj;
    std::vector<Point2f> scene;
    for( size_t i = 0; i < good_matches.size(); i++ )
    {
        //-- Get the keypoints from the good matches
        obj.push_back( keypoints_object[ good_matches[i].queryIdx ].pt );
        scene.push_back( keypoints_scene[ good_matches[i].trainIdx ].pt );
    }
    if (obj.size() < 4 || scene.size() < 4) {
        return NO;
    }
    
    if ([self shouldStopDetect]) {return NO;}
    Mat H = findHomography( obj, scene, CV_RANSAC );
    
    //-- Get the corners from the image_1 ( the object to be "detected" )
    if ([self shouldStopDetect]) {return NO;}
    std::vector<Point2f> obj_corners(4);
    obj_corners[0] = cvPoint(0,0);
    obj_corners[1] = cvPoint( img_object.cols, 0 );
    obj_corners[2] = cvPoint( img_object.cols, img_object.rows );
    obj_corners[3] = cvPoint( 0, img_object.rows );
    
    // 判断点是否是有效点
    bool exist = 1;
    std::vector<Point2f> scene_corners(4);
    perspectiveTransform( obj_corners, scene_corners, H);
    for (int j = 0; j < 4; j++) {
        if (cv::norm(scene_corners[(j+1) % 4] - scene_corners[j]) < 40) {
            exist = 0;
            break;
        }
    }
    
    if (_isDebugging) {
        if ([self shouldStopDetect]) {return NO;}
        // 画关键点
        Mat img_matches;
        drawMatches( _img_object, _keypoints_object, img_scene, keypoints_scene,
                    good_matches, img_matches, Scalar::all(-1), Scalar::all(-1),
                    vector<char>(), DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS );
        //-- Draw lines between the corners (the mapped object in the scene - image_2 )
        Point2f offset( (float)_img_object.cols, 0);
        line( img_matches, scene_corners[0] + offset, scene_corners[1] + offset, Scalar( 0, 255, 0), 4 );
        line( img_matches, scene_corners[1] + offset, scene_corners[2] + offset, Scalar( 0, 255, 0), 4 );
        line( img_matches, scene_corners[2] + offset, scene_corners[3] + offset, Scalar( 0, 255, 0), 4 );
        line( img_matches, scene_corners[3] + offset, scene_corners[0] + offset, Scalar( 0, 255, 0), 4 );
        [self showResultImage:[UIImage imageWithCVMat:img_matches]];
    }
    
    if (exist) {
        float ratio_1 = cv::norm(obj_corners[1] - obj_corners[0]) / cv::norm(scene_corners[1] - scene_corners[0]);
        for (int j = 1; j < 4; j++) {
            float ratio_2 = cv::norm(obj_corners[(j+1) % 4] - obj_corners[j]) / cv::norm(scene_corners[(j+1) % 4] - scene_corners[j]);
            if (ratio_2 / ratio_1 > 1.2 || ratio_2 / ratio_1 < 0.8) {
                exist = 0;
                break;
            }
        }
    }
    
    if (exist) {
        gettimeofday(&taskStopTime, NULL);
        elapsedTime = (taskStopTime.tv_sec - taskStartTime.tv_sec) * 1000.0;
        elapsedTime += (taskStopTime.tv_usec - taskStartTime.tv_usec) / 1000.0;
        std::cout << "总耗时 " << elapsedTime << endl;
        
        [self hasDetected:[NSString stringWithFormat:@"%s %f", sel_getName(_cmd), elapsedTime]];
        
        cout<< "四个顶点" << scene_corners[0] << " " << scene_corners[1] << " " << scene_corners[2] << " " << scene_corners[3] << endl;
        [self showResultCoverViwe:CGRectMake(scene_corners[0].x,
                                             scene_corners[0].y,
                                             scene_corners[1].x - scene_corners[0].x,
                                             scene_corners[2].y - scene_corners[1].y)];
    }
    
    return exist;
}

#pragma mark -
- (void)showResultImage:(UIImage *)image {
    if (_isDebugging) {
        dispatch_async(dispatch_get_main_queue(), ^{
            _imageView_result.image = image;
        });
    }
}
- (void)showResultCoverViwe:(CGRect)rect {
    dispatch_async(dispatch_get_main_queue(), ^{
        //        if (_resultCoverViwe.hidden) {
        //            _resultCoverViwe.hidden = NO;
        //            _resultCoverViwe.frame = rect;
        //        }
        
        @synchronized(self.detectTimes) {
            int detectTimes = [self.detectTimes intValue];
            detectTimes++;
            self.detectTimes = [NSNumber numberWithInt:detectTimes];
            
            if (detectTimes == 1) {
                AudioServicesPlaySystemSound(tapSoundID);
            }
        }
        
        _introduceView.alpha = 0.6;
        [UIView animateWithDuration:2
                              delay:2
                            options:UIViewAnimationOptionCurveEaseInOut
                         animations:^{
                             //                             _resultCoverViwe.frame = rect;
                             _introduceView.alpha = 0;
                         } completion:^(BOOL finished) {
                             @synchronized(self.detectTimes) {
                                 int detectTimes = [self.detectTimes intValue];
                                 detectTimes--;
                                 self.detectTimes = [NSNumber numberWithInt:detectTimes];
                             }
                         }];
    });
}
- (IBAction)onClickRestartBtn:(id)sender {
    //    dispatch_async(dispatch_get_main_queue(), ^{
    //        [UIView animateWithDuration:0.3
    //                         animations:^{
    //                             _introduceView.alpha = 0;
    //                         }];
    //    });
}
- (void)hasDetected:(NSString *)message {
    //    FLOG(@"%@", message);
}

- (BOOL)shouldStopDetect {
    @synchronized(self.lastMotionTime) {
        timeval timeNow;
        gettimeofday(&timeNow, NULL);
        double elapsedTime;
        elapsedTime = timeNow.tv_sec + timeNow.tv_usec / 1000 / 1000.0 - [self.lastMotionTime doubleValue];
        
        if (elapsedTime < 0) {
            return YES;
        } else {
            return NO;
        }
    }
}
#pragma mark - 重力感应
- (void)viewDidAppear:(BOOL)animated {
    [super viewDidAppear:animated];
    [self startMyMotionDetect];
}
- (void)viewDidDisappear:(BOOL)animated {
    [super viewDidDisappear:animated];
    [self.motionManager stopAccelerometerUpdates];
}
- (CMMotionManager *)motionManager {
    CMMotionManager *motionManager = nil;
    id appDelegate = [UIApplication sharedApplication].delegate;
    if ([appDelegate respondsToSelector:@selector(motionManager)]) {
        motionManager = [appDelegate motionManager];
    }
    return motionManager;
}
- (void)startMyMotionDetect
{
    //velocity[i] = (acceleration[i] + acceleration[i-1])/2 * interval + velocity[i-1];
    //distance[i] = (velocity[i] + velocity[i-1])/2 * interval + distance[i-1];
    CMAccelerometerHandler handler = ^(CMAccelerometerData *accelerometerData, NSError *error) {
        if (preAccelerationX_min >= accelerometerData.acceleration.x) {
            preAccelerationX_min = accelerometerData.acceleration.x;
        }
        if (preAccelerationX_max < accelerometerData.acceleration.x) {
            preAccelerationX_max = accelerometerData.acceleration.x;
        }
        if (preAccelerationY_min >= accelerometerData.acceleration.y) {
            preAccelerationY_min = accelerometerData.acceleration.y;
        }
        if (preAccelerationY_max < accelerometerData.acceleration.y) {
            preAccelerationY_max = accelerometerData.acceleration.y;
        }
        if (preAccelerationZ_min >= accelerometerData.acceleration.z) {
            preAccelerationZ_min = accelerometerData.acceleration.z;
        }
        if (preAccelerationZ_max < accelerometerData.acceleration.z) {
            preAccelerationZ_max = accelerometerData.acceleration.z;
        }
        
        timeval timeNow;
        gettimeofday(&timeNow, NULL);
        double elapsedTime;
        elapsedTime = timeNow.tv_sec + timeNow.tv_usec / 1000 / 1000.0 - [self.lastMotionTime doubleValue];
        
        if (elapsedTime > 0.5) {
            self.lastMotionTime = [NSNumber numberWithDouble:timeNow.tv_sec + timeNow.tv_usec / 1000 / 1000.0];
            
            //            std::cout
            //            << "加速度 "
            //            << abs(preAccelerationX_max - preAccelerationX_min) << " "
            //            << abs(preAccelerationY_max - preAccelerationY_min) << " "
            //            << abs(preAccelerationZ_max - preAccelerationZ_min) << " "
            //            << std::endl;
            
            double limit = 0.1;
            if (abs(preAccelerationX_max - preAccelerationX_min) > limit ||
                abs(preAccelerationY_max - preAccelerationY_min) > limit ||
                abs(preAccelerationZ_max - preAccelerationZ_min) > limit)
            {
                [_detectQueue cancelAllOperations];
                for (NSInvocationOperation *operation in _detectQueue.operations) {
                    [operation cancel];
                }
                //                NSLog(@"加速了, left %d", _detectQueue.operationCount);
            }
            
            preAccelerationX_min = 1000;
            preAccelerationY_min = 1000;
            preAccelerationZ_min = 1000;
            preAccelerationX_max = -1000;
            preAccelerationY_max = -1000;
            preAccelerationZ_max = -1000;
        }
    };
    
    [self.motionManager startAccelerometerUpdatesToQueue:[[NSOperationQueue alloc] init] withHandler:handler];
}
#pragma mark - other
- (BOOL)createCaptureSessionForCamera:(NSInteger)camera qualityPreset:(NSString *)qualityPreset grayscale:(BOOL)grayscale {
    // 获取摄像头
    NSArray* devices = [AVCaptureDevice devicesWithMediaType:AVMediaTypeVideo];
    if ([devices count] == 0) {
        FLOG(@"No video capture devices found");
        return NO;
    }
    _captureDevice = [[devices objectAtIndex:camera] retain];
    
    // Create the capture session
    _captureSession = [[AVCaptureSession alloc] init];
    _captureSession.sessionPreset = AVCaptureSessionPresetMedium;
    
    // Create device input
    NSError *error = nil;
    AVCaptureDeviceInput *input = [[AVCaptureDeviceInput alloc] initWithDevice:_captureDevice error:&error];
    
    // Create and configure device output
    _videoOutput = [[AVCaptureVideoDataOutput alloc] init];
    dispatch_queue_t queue = dispatch_queue_create("cameraQueue", NULL);
    [_videoOutput setSampleBufferDelegate:self queue:queue];
    dispatch_release(queue);
    _videoOutput.alwaysDiscardsLateVideoFrames = YES;
    
    // For grayscale mode, the luminance channel from the YUV fromat is used
    // For color mode, BGRA format is used
    OSType format = kCVPixelFormatType_32BGRA;
    
    // Check YUV format is available before selecting it (iPhone 3 does not support it)
    if (grayscale &&
        [_videoOutput.availableVideoCVPixelFormatTypes containsObject:[NSNumber numberWithInt:kCVPixelFormatType_420YpCbCr8BiPlanarFullRange]]) {
        format = kCVPixelFormatType_420YpCbCr8BiPlanarFullRange;
    }
    _videoOutput.videoSettings = [NSDictionary dictionaryWithObject:[NSNumber numberWithUnsignedInt:format]
                                                             forKey:(id)kCVPixelBufferPixelFormatTypeKey];
    
    // Connect up inputs and outputs
    if ([_captureSession canAddInput:input]) {
        [_captureSession addInput:input];
    }
    if ([_captureSession canAddOutput:_videoOutput]) {
        [_captureSession addOutput:_videoOutput];
    }
    [input release];
    
    _videoPreviewLayer = [[AVCaptureVideoPreviewLayer alloc] initWithSession:_captureSession];
    [_videoPreviewLayer setFrame:self.view.bounds];
    _videoPreviewLayer.videoGravity = AVLayerVideoGravityResizeAspectFill;
    [self.view.layer insertSublayer:_videoPreviewLayer atIndex:0];
    
    return YES;
}
- (void)captureOutput:(AVCaptureOutput *)captureOutput didOutputSampleBuffer:(CMSampleBufferRef)sampleBuffer fromConnection:(AVCaptureConnection *)connection {
    NSAutoreleasePool* localpool = [[NSAutoreleasePool alloc] init];
    
    CVPixelBufferRef pixelBuffer = CMSampleBufferGetImageBuffer(sampleBuffer);
    OSType format = CVPixelBufferGetPixelFormatType(pixelBuffer);
    CGRect videoRect = CGRectMake(0.0f, 0.0f, CVPixelBufferGetWidth(pixelBuffer), CVPixelBufferGetHeight(pixelBuffer));
    AVCaptureVideoOrientation videoOrientation = [[[_videoOutput connections] objectAtIndex:0] videoOrientation];
    
    if (format == kCVPixelFormatType_420YpCbCr8BiPlanarFullRange) {
        // For grayscale mode, the luminance channel of the YUV data is used
        CVPixelBufferLockBaseAddress(pixelBuffer, 0);
        void *baseaddress = CVPixelBufferGetBaseAddressOfPlane(pixelBuffer, 0);
        cv::Mat mat(videoRect.size.height, videoRect.size.width, CV_8UC1, baseaddress, 0);
        
        [self processFrame:mat videoRect:videoRect videoOrientation:videoOrientation];
    }
    CVPixelBufferUnlockBaseAddress(pixelBuffer, 0);
    
    [localpool drain];
}
- (void)destroyCaptureSession {
    [_captureSession stopRunning];
    [_videoPreviewLayer removeFromSuperlayer];
    [_videoPreviewLayer release];
    [_videoOutput release];
    [_captureDevice release];
    [_captureSession release];
    _videoPreviewLayer = nil;
    _videoOutput = nil;
    _captureDevice = nil;
    _captureSession = nil;
}
- (void)viewDidUnload {
    [super viewDidUnload];
    [self destroyCaptureSession];
}
- (void)dealloc {
    [self destroyCaptureSession];
    [_imageView_result release];
    [_imageView_object release];
    [_imageView_scene release];
    [_restartBtn release];
    [_imageView_object release];
    [_detectQueue release];
    [_lastMotionTime release];
    [_resultCoverViwe release];
    [_introduceView release];
    [_detectTimes release];
    [super dealloc];
}

@end