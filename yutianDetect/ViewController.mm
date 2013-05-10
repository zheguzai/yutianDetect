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

using namespace std;
using namespace cv;

@interface ViewController ()
- (BOOL)createCaptureSessionForCamera:(NSInteger)camera qualityPreset:(NSString *)qualityPreset grayscale:(BOOL)grayscale;
- (void)destroyCaptureSession;
- (void)processFrame:(cv::Mat&)mat videoRect:(CGRect)rect videoOrientation:(AVCaptureVideoOrientation)orientation;
@end

@implementation ViewController

@synthesize captureSession = _captureSession;
@synthesize captureDevice = _captureDevice;
@synthesize videoOutput = _videoOutput;
@synthesize videoPreviewLayer = _videoPreviewLayer;

- (void)dealloc {
    [self destroyCaptureSession];
    
    [_imageView_result release];
    [resultLabel release];
    [scrollView_obj release];
    [scrollView_scene release];
    [timeLabel release];
    [detectTypeTextField release];
    [super dealloc];
}
- (void)viewDidUnload {
    [super viewDidUnload];
    
    [self destroyCaptureSession];
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
        
        CVPixelBufferUnlockBaseAddress(pixelBuffer, 0);
    }
    
    [localpool drain];
}
- (BOOL)createCaptureSessionForCamera:(NSInteger)camera qualityPreset:(NSString *)qualityPreset grayscale:(BOOL)grayscale {
    // 获取摄像头
    NSArray* devices = [AVCaptureDevice devicesWithMediaType:AVMediaTypeVideo];
    if ([devices count] == 0) {
        NSLog(@"No video capture devices found");
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
- (void)processFrame:(cv::Mat&)mat videoRect:(CGRect)rect videoOrientation:(AVCaptureVideoOrientation)orientation {
    [self detectImage:mat];
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
- (BOOL)textFieldShouldReturn:(UITextField *)textField {
    [textField resignFirstResponder];
    return YES;
}
#pragma mark -
- (void)viewDidLoad
{
    [super viewDidLoad];
    
    //    [self createCaptureSessionForCamera:0 qualityPreset:AVCaptureSessionPresetMedium grayscale:YES];
    //    [_captureSession startRunning];
    
    UIImageView *imageView = nil;
    CGRect frame = CGRectZero;
    
    int count = 6;
    for (int i = 0; i <= count; i++) {
        imageView = [[[UIImageView alloc] init] autorelease];
        imageView.contentMode = UIViewContentModeScaleAspectFit;
        frame = scrollView_obj.bounds;
        frame.origin.x = scrollView_obj.bounds.size.width * i;
        imageView.frame = frame;
        imageView.image = [UIImage imageNamed:[NSString stringWithFormat:@"%d.jpg", i]];
        [scrollView_obj addSubview:imageView];
        
        imageView = [[[UIImageView alloc] init] autorelease];
        imageView.contentMode = UIViewContentModeScaleAspectFit;
        frame = scrollView_scene.bounds;
        frame.origin.x = scrollView_scene.bounds.size.width * i;
        imageView.frame = frame;
        imageView.image = [UIImage imageNamed:[NSString stringWithFormat:@"background_%d.jpg", i]];
        [scrollView_scene addSubview:imageView];
    }
    
    CGSize contentSize = CGSizeZero;
    contentSize = scrollView_obj.contentSize;
    contentSize.width = scrollView_obj.bounds.size.width * (count + 1);
    scrollView_obj.contentSize = contentSize;
    
    contentSize = scrollView_scene.contentSize;
    contentSize.width = scrollView_scene.bounds.size.width * (count + 1);
    scrollView_scene.contentSize = contentSize;
}

- (IBAction)clickDetectBtn:(id)sender {
    resultLabel.text = @"";
    timeLabel.text = @"";
    
    detectType = [detectTypeTextField.text intValue];
    
    currentIndex_obj = scrollView_obj.contentOffset.x / scrollView_obj.bounds.size.width;
    currentIndex_scene = scrollView_scene.contentOffset.x / scrollView_scene.bounds.size.width;
    
    NSString *filePath = nil;
    filePath = [[NSBundle mainBundle] pathForResource:[NSString stringWithFormat:@"background_%d", currentIndex_scene] ofType:@"jpg"];
    cv::Mat img_scene = cv::imread([filePath UTF8String], CV_LOAD_IMAGE_GRAYSCALE );
    [self detectImage:img_scene];
}

- (void)detectImage:(cv::Mat&)mat {
    switch (detectType) {
        case 0:
        {
            [self detectImage_0:mat];
            break;
        }
        case 1:
        {
            [self detectImage_1:mat];
            break;
        }
        case 2:
        {
            [self detectImage_2:mat];
            break;
        }
        case 3:
        {
            [self detectImage_3:mat];
            break;
        }
        case 4:
        {
            [self detectImage_4:mat];
            break;
        }
        default: {
            break;
        }
    }
}

#pragma mark - ORB BruteForceMatcher
- (void)detectImage_0:(cv::Mat&)mat {
    std::cout << "开始\n";
    
    timeval tasksStartTime, taskStartTime, t1, t2;
    double elapsedTime;
    gettimeofday(&tasksStartTime, NULL);
    
    cv::Mat img_scene = mat;
    NSString *filePath = nil;
    int i = currentIndex_obj;
    int count = currentIndex_obj;
    
    for (; i <= count; i++) {
        std::cout << "扫描第 " << i << " 张" << endl;
        
        gettimeofday(&taskStartTime, NULL);
        
        filePath = [[NSBundle mainBundle] pathForResource:[NSString stringWithFormat:@"%d", i] ofType:@"jpg"];
        cv::Mat img_object = cv::imread([filePath UTF8String], CV_LOAD_IMAGE_GRAYSCALE);
        if( !img_object.data || !img_scene.data ) {
            std::cout << "没有数据\n";
            return;
        }
        
        gettimeofday(&t1, NULL);
        //-- Step 1 -- Step 2: 检测关键点，获取 descriptor
        ORB orb_object(30,ORB::CommonParams(1.2,1));
        ORB orb_scene(100,ORB::CommonParams(1.2,1));
        
        std::vector<KeyPoint> keypoints_object, keypoints_scene;
        Mat descriptors_object, descriptors_scene;
        
        orb_object(img_object,Mat(), keypoints_object, descriptors_object, false);
        orb_scene(img_scene,Mat(), keypoints_scene, descriptors_scene, false);
        cout<< "keypoints_object.size() " << keypoints_object.size() << endl;
        cout<< "keypoints_scene.size() " << keypoints_scene.size() << endl;
        if (keypoints_object.size() == 0 || keypoints_scene.size() == 0) {
            continue;
        }
        gettimeofday(&t2, NULL);
        elapsedTime = (t2.tv_sec - t1.tv_sec) * 1000.0;
        elapsedTime += (t2.tv_usec - t1.tv_usec) / 1000.0;
        std::cout<< "检测关键点，获取 descriptor 耗时 " << elapsedTime << endl;
        
        gettimeofday(&t1, NULL);
        //-- Step 3: Matching descriptor vectors using FLANN matcher
        BruteForceMatcher<HammingLUT>matcher;
        std::vector< DMatch > matches;
        matcher.match( descriptors_object, descriptors_scene, matches );
        cout<< "matches.size() " << matches.size() << endl;
        if (matches.size() == 0) {
            continue;
        }
        gettimeofday(&t2, NULL);
        elapsedTime = (t2.tv_sec - t1.tv_sec) * 1000.0;
        elapsedTime += (t2.tv_usec - t1.tv_usec) / 1000.0;
        std::cout<< "获取 matches 耗时 " << elapsedTime << endl;
        
        //-- Quick calculation of max and min distances between keypoints
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
        printf("-- Max dist : %f \n", max_dist );
        printf("-- Min dist : %f \n", min_dist );
        
        //-- Draw only "good" matches (i.e. whose distance is less than 3*min_dist )
        std::vector< DMatch > good_matches;
        for( int i = 0; i < descriptors_object.rows; i++ ) {
            if( matches[i].distance < 3*min_dist ) {
                good_matches.push_back( matches[i]);
            }
        }
        cout<< "good_matches.size() " << good_matches.size() << endl;
        if (good_matches.size() == 0) {
            continue;
        }
        
        // 画关键点
        Mat img_matches;
        drawMatches( img_object, keypoints_object, img_scene, keypoints_scene,
                    good_matches, img_matches, Scalar::all(-1), Scalar::all(-1),
                    vector<char>(), DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS );
        
        //-- Localize the object from img_1 in img_2
        std::vector<Point2f> obj;
        std::vector<Point2f> scene;
        for( size_t i = 0; i < good_matches.size(); i++ )
        {
            //-- Get the keypoints from the good matches
            obj.push_back( keypoints_object[ good_matches[i].queryIdx ].pt );
            scene.push_back( keypoints_scene[ good_matches[i].trainIdx ].pt );
        }
        
        gettimeofday(&t1, NULL);
        Mat H = findHomography( obj, scene, CV_RANSAC );
        gettimeofday(&t2, NULL);
        elapsedTime = (t2.tv_sec - t1.tv_sec) * 1000.0;
        elapsedTime += (t2.tv_usec - t1.tv_usec) / 1000.0;
        std::cout<< "findHomography 耗时 " << elapsedTime << endl;
        
        //-- Get the corners from the image_1 ( the object to be "detected" )
        std::vector<Point2f> obj_corners(4);
        obj_corners[0] = cvPoint(0,0);
        obj_corners[1] = cvPoint( img_object.cols, 0 );
        obj_corners[2] = cvPoint( img_object.cols, img_object.rows );
        obj_corners[3] = cvPoint( 0, img_object.rows );
        for (int j = 0; j < 4; j++) {
            cout<< "obj_corners[" << j << "] = " << obj_corners[j].x << " " << obj_corners[j].y << endl;
        }
        for (int j = 0; j < 4; j++) {
            cout<< "长度 obj_corners " << j << " = " << cv::norm(obj_corners[(j+1) % 4] - obj_corners[j]) << endl;
        }
        
        // 判断点是否是有效点
        bool exist = 1;
        std::vector<Point2f> scene_corners(4);
        perspectiveTransform( obj_corners, scene_corners, H);
        for (int j = 0; j < 4; j++) {
            cout<< "点 scene_corners[" << j << "] = " << scene_corners[j].x << " " << scene_corners[j].y << endl;
        }
        for (int j = 0; j < 4; j++) {
            cout<< "长度 scene_corners " << j << " = " << cv::norm(scene_corners[(j+1) % 4] - scene_corners[j]) << endl;
            if (cv::norm(scene_corners[(j+1) % 4] - scene_corners[j]) == 0) {
                exist = 0;
                break;
            }
        }
        
        //-- Draw lines between the corners (the mapped object in the scene - image_2 )
        Point2f offset( (float)img_object.cols, 0);
        line( img_matches, scene_corners[0] + offset, scene_corners[1] + offset, Scalar( 0, 255, 0), 4 );
        line( img_matches, scene_corners[1] + offset, scene_corners[2] + offset, Scalar( 0, 255, 0), 4 );
        line( img_matches, scene_corners[2] + offset, scene_corners[3] + offset, Scalar( 0, 255, 0), 4 );
        line( img_matches, scene_corners[3] + offset, scene_corners[0] + offset, Scalar( 0, 255, 0), 4 );
        
        _imageView_result.image = [UIImage imageWithCVMat:img_matches];
        
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
        
        gettimeofday(&t2, NULL);
        elapsedTime = (t2.tv_sec - taskStartTime.tv_sec) * 1000.0;
        elapsedTime += (t2.tv_usec - taskStartTime.tv_usec) / 1000.0;
        std::cout << "扫描第 " << i << " 张耗时 " << elapsedTime << endl << endl;
        
        if (exist) {
            break;
        }
    }
    
    gettimeofday(&t2, NULL);
    elapsedTime = (t2.tv_sec - tasksStartTime.tv_sec) * 1000.0;
    elapsedTime += (t2.tv_usec - tasksStartTime.tv_usec) / 1000.0;
    std::cout << "总耗时 " << elapsedTime << endl;
    if (i <= count) {
        std::cout << "这是第 " << i << " 张\n\n";
    } else {
        std::cout << "没有对应图片\n\n";
    }
    timeLabel.text = [NSString stringWithFormat:@"%f", elapsedTime];
}

#pragma mark - FastFeatureDetector BriefDescriptorExtractor BruteForceMatcher
- (void)detectImage_1:(cv::Mat&)mat {
    std::cout << "开始\n";
    
    timeval tasksStartTime, taskStartTime, t1, t2;
    double elapsedTime;
    gettimeofday(&tasksStartTime, NULL);
    
    cv::Mat img_scene = mat;
    NSString *filePath = nil;
    int i = currentIndex_obj;
    int count = currentIndex_obj;
    
    for (; i <= count; i++) {
        std::cout << "扫描第 " << i << " 张" << endl;
        
        gettimeofday(&taskStartTime, NULL);
        
        filePath = [[NSBundle mainBundle] pathForResource:[NSString stringWithFormat:@"%d", i] ofType:@"jpg"];
        cv::Mat img_object = cv::imread([filePath UTF8String], CV_LOAD_IMAGE_GRAYSCALE);
        if( !img_object.data || !img_scene.data ) {
            std::cout << "没有数据\n";
            return;
        }
        
        gettimeofday(&t1, NULL);
        //-- Step 1: Detect the keypoints using SURF Detector
        FastFeatureDetector detector(15);
        
        std::vector<KeyPoint> keypoints_object, keypoints_scene;
        detector.detect( img_object, keypoints_object );
        detector.detect( img_scene, keypoints_scene );
        //        cout<< "keypoints_object.size() " << keypoints_object.size() << endl;
        //        cout<< "keypoints_scene.size() " << keypoints_scene.size() << endl;
        if (keypoints_object.size() == 0 || keypoints_scene.size() == 0) {
            continue;
        }
        gettimeofday(&t2, NULL);
        elapsedTime = (t2.tv_sec - t1.tv_sec) * 1000.0;
        elapsedTime += (t2.tv_usec - t1.tv_usec) / 1000.0;
        std::cout<< "获取关键点耗时 " << elapsedTime << endl;
        
        gettimeofday(&t1, NULL);
        //-- Step 2: Calculate descriptors (feature vectors)
        BriefDescriptorExtractor extractor;
        
        Mat descriptors_object, descriptors_scene;
        extractor.compute( img_object, keypoints_object, descriptors_object );
        extractor.compute( img_scene, keypoints_scene, descriptors_scene );
        gettimeofday(&t2, NULL);
        elapsedTime = (t2.tv_sec - t1.tv_sec) * 1000.0;
        elapsedTime += (t2.tv_usec - t1.tv_usec) / 1000.0;
        std::cout<< "计算关键点的描述对象耗时 " << elapsedTime << endl;
        
        gettimeofday(&t1, NULL);
        //-- Step 3: Matching descriptor vectors using FLANN matcher
        BruteForceMatcher< Hamming > matcher;
        
        std::vector< DMatch > matches;
        matcher.match( descriptors_object, descriptors_scene, matches );
        //        cout<< "matches.size() " << matches.size() << endl;
        if (matches.size() == 0) {
            continue;
        }
        gettimeofday(&t2, NULL);
        elapsedTime = (t2.tv_sec - t1.tv_sec) * 1000.0;
        elapsedTime += (t2.tv_usec - t1.tv_usec) / 1000.0;
        std::cout<< "获取 matches 耗时 " << elapsedTime << endl;
        
        //-- Quick calculation of max and min distances between keypoints
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
        //        printf("-- Max dist : %f \n", max_dist );
        //        printf("-- Min dist : %f \n", min_dist );
        
        //-- Draw only "good" matches (i.e. whose distance is less than 3*min_dist )
        std::vector< DMatch > good_matches;
        for( int i = 0; i < descriptors_object.rows; i++ ) {
            if( matches[i].distance < 3*min_dist ) {
                good_matches.push_back( matches[i]);
            }
        }
        //        cout<< "good_matches.size() " << good_matches.size() << endl;
        if (good_matches.size() == 0) {
            continue;
        }
        
        // 画关键点
        Mat img_matches;
        drawMatches( img_object, keypoints_object, img_scene, keypoints_scene,
                    good_matches, img_matches, Scalar::all(-1), Scalar::all(-1),
                    vector<char>(), DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS );
        
        //-- Localize the object from img_1 in img_2
        std::vector<Point2f> obj;
        std::vector<Point2f> scene;
        for( size_t i = 0; i < good_matches.size(); i++ )
        {
            //-- Get the keypoints from the good matches
            obj.push_back( keypoints_object[ good_matches[i].queryIdx ].pt );
            scene.push_back( keypoints_scene[ good_matches[i].trainIdx ].pt );
        }
        
        gettimeofday(&t1, NULL);
        Mat H = findHomography( obj, scene, CV_RANSAC );
        gettimeofday(&t2, NULL);
        elapsedTime = (t2.tv_sec - t1.tv_sec) * 1000.0;
        elapsedTime += (t2.tv_usec - t1.tv_usec) / 1000.0;
        std::cout<< "findHomography 耗时 " << elapsedTime << endl;
        
        //-- Get the corners from the image_1 ( the object to be "detected" )
        std::vector<Point2f> obj_corners(4);
        obj_corners[0] = cvPoint(0,0);
        obj_corners[1] = cvPoint( img_object.cols, 0 );
        obj_corners[2] = cvPoint( img_object.cols, img_object.rows );
        obj_corners[3] = cvPoint( 0, img_object.rows );
        //        for (int j = 0; j < 4; j++) {
        //            cout<< "obj_corners[" << j << "] = " << obj_corners[j].x << " " << obj_corners[j].y << endl;
        //        }
        //        for (int j = 0; j < 4; j++) {
        //            cout<< "长度 obj_corners " << j << " = " << cv::norm(obj_corners[(j+1) % 4] - obj_corners[j]) << endl;
        //        }
        
        // 判断点是否是有效点
        bool exist = 1;
        std::vector<Point2f> scene_corners(4);
        perspectiveTransform( obj_corners, scene_corners, H);
        //        for (int j = 0; j < 4; j++) {
        //            cout<< "点 scene_corners[" << j << "] = " << scene_corners[j].x << " " << scene_corners[j].y << endl;
        //        }
        for (int j = 0; j < 4; j++) {
            //            cout<< "长度 scene_corners " << j << " = " << cv::norm(scene_corners[(j+1) % 4] - scene_corners[j]) << endl;
            if (cv::norm(scene_corners[(j+1) % 4] - scene_corners[j]) == 0) {
                exist = 0;
                break;
            }
        }
        
        //-- Draw lines between the corners (the mapped object in the scene - image_2 )
        Point2f offset( (float)img_object.cols, 0);
        line( img_matches, scene_corners[0] + offset, scene_corners[1] + offset, Scalar( 0, 255, 0), 4 );
        line( img_matches, scene_corners[1] + offset, scene_corners[2] + offset, Scalar( 0, 255, 0), 4 );
        line( img_matches, scene_corners[2] + offset, scene_corners[3] + offset, Scalar( 0, 255, 0), 4 );
        line( img_matches, scene_corners[3] + offset, scene_corners[0] + offset, Scalar( 0, 255, 0), 4 );
        
        _imageView_result.image = [UIImage imageWithCVMat:img_matches];
        
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
        
        gettimeofday(&t2, NULL);
        elapsedTime = (t2.tv_sec - taskStartTime.tv_sec) * 1000.0;
        elapsedTime += (t2.tv_usec - taskStartTime.tv_usec) / 1000.0;
        std::cout << "扫描第 " << i << " 张耗时 " << elapsedTime << endl << endl;
        
        if (exist) {
            break;
        }
    }
    
    gettimeofday(&t2, NULL);
    elapsedTime = (t2.tv_sec - tasksStartTime.tv_sec) * 1000.0;
    elapsedTime += (t2.tv_usec - tasksStartTime.tv_usec) / 1000.0;
    std::cout << "总耗时 " << elapsedTime << endl;
    if (i <= count) {
        std::cout << "这是第 " << i << " 张\n\n";
        resultLabel.text = @"V";
    } else {
        std::cout << "没有对应图片\n\n";
        resultLabel.text = @"X";
    }
    timeLabel.text = [NSString stringWithFormat:@"%f", elapsedTime];
}

#pragma mark - FastFeatureDetector SurfDescriptorExtractor FlannBasedMatcher
- (void)detectImage_2:(cv::Mat&)mat {
    std::cout << "开始\n";
    
    timeval tasksStartTime, taskStartTime, t1, t2;
    double elapsedTime;
    gettimeofday(&tasksStartTime, NULL);
    
    cv::Mat img_scene = mat;
    NSString *filePath = nil;
    int i = currentIndex_obj;
    int count = currentIndex_obj;
    
    for (; i <= count; i++) {
        std::cout << "扫描第 " << i << " 张" << endl;
        
        gettimeofday(&taskStartTime, NULL);
        
        filePath = [[NSBundle mainBundle] pathForResource:[NSString stringWithFormat:@"%d", i] ofType:@"jpg"];
        cv::Mat img_object = cv::imread([filePath UTF8String], CV_LOAD_IMAGE_GRAYSCALE);
        if( !img_object.data || !img_scene.data ) {
            std::cout << "没有数据\n";
            return;
        }
        
        gettimeofday(&t1, NULL);
        //-- Step 1: Detect the keypoints using SURF Detector
        FastFeatureDetector detector(15);
        
        std::vector<KeyPoint> keypoints_object, keypoints_scene;
        detector.detect( img_object, keypoints_object );
        detector.detect( img_scene, keypoints_scene );
        //        cout<< "keypoints_object.size() " << keypoints_object.size() << endl;
        //        cout<< "keypoints_scene.size() " << keypoints_scene.size() << endl;
        if (keypoints_object.size() == 0 || keypoints_scene.size() == 0) {
            continue;
        }
        gettimeofday(&t2, NULL);
        elapsedTime = (t2.tv_sec - t1.tv_sec) * 1000.0;
        elapsedTime += (t2.tv_usec - t1.tv_usec) / 1000.0;
        std::cout<< "获取关键点耗时 " << elapsedTime << endl;
        
        gettimeofday(&t1, NULL);
        //-- Step 2: Calculate descriptors (feature vectors)
        SurfDescriptorExtractor extractor;
        Mat descriptors_object, descriptors_scene;
        extractor.compute( img_object, keypoints_object, descriptors_object );
        extractor.compute( img_scene, keypoints_scene, descriptors_scene );
        gettimeofday(&t2, NULL);
        elapsedTime = (t2.tv_sec - t1.tv_sec) * 1000.0;
        elapsedTime += (t2.tv_usec - t1.tv_usec) / 1000.0;
        std::cout<< "计算关键点的描述对象耗时 " << elapsedTime << endl;
        
        gettimeofday(&t1, NULL);
        //-- Step 3: Matching descriptor vectors using FLANN matcher
        FlannBasedMatcher matcher;
        std::vector< DMatch > matches;
        matcher.match( descriptors_object, descriptors_scene, matches );
        //        cout<< "matches.size() " << matches.size() << endl;
        if (matches.size() == 0) {
            continue;
        }
        gettimeofday(&t2, NULL);
        elapsedTime = (t2.tv_sec - t1.tv_sec) * 1000.0;
        elapsedTime += (t2.tv_usec - t1.tv_usec) / 1000.0;
        std::cout<< "获取 matches 耗时 " << elapsedTime << endl;
        
        //-- Quick calculation of max and min distances between keypoints
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
        //        printf("-- Max dist : %f \n", max_dist );
        //        printf("-- Min dist : %f \n", min_dist );
        
        //-- Draw only "good" matches (i.e. whose distance is less than 3*min_dist )
        std::vector< DMatch > good_matches;
        for( int i = 0; i < descriptors_object.rows; i++ ) {
            if( matches[i].distance < 3*min_dist ) {
                good_matches.push_back( matches[i]);
            }
        }
        //        cout<< "good_matches.size() " << good_matches.size() << endl;
        if (good_matches.size() == 0) {
            continue;
        }
        
        // 画关键点
        Mat img_matches;
        drawMatches( img_object, keypoints_object, img_scene, keypoints_scene,
                    good_matches, img_matches, Scalar::all(-1), Scalar::all(-1),
                    vector<char>(), DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS );
        
        //-- Localize the object from img_1 in img_2
        std::vector<Point2f> obj;
        std::vector<Point2f> scene;
        for( size_t i = 0; i < good_matches.size(); i++ )
        {
            //-- Get the keypoints from the good matches
            obj.push_back( keypoints_object[ good_matches[i].queryIdx ].pt );
            scene.push_back( keypoints_scene[ good_matches[i].trainIdx ].pt );
        }
        
        gettimeofday(&t1, NULL);
        Mat H = findHomography( obj, scene, CV_RANSAC );
        gettimeofday(&t2, NULL);
        elapsedTime = (t2.tv_sec - t1.tv_sec) * 1000.0;
        elapsedTime += (t2.tv_usec - t1.tv_usec) / 1000.0;
        std::cout<< "findHomography 耗时 " << elapsedTime << endl;
        
        //-- Get the corners from the image_1 ( the object to be "detected" )
        std::vector<Point2f> obj_corners(4);
        obj_corners[0] = cvPoint(0,0);
        obj_corners[1] = cvPoint( img_object.cols, 0 );
        obj_corners[2] = cvPoint( img_object.cols, img_object.rows );
        obj_corners[3] = cvPoint( 0, img_object.rows );
        //        for (int j = 0; j < 4; j++) {
        //            cout<< "obj_corners[" << j << "] = " << obj_corners[j].x << " " << obj_corners[j].y << endl;
        //        }
        //        for (int j = 0; j < 4; j++) {
        //            cout<< "长度 obj_corners " << j << " = " << cv::norm(obj_corners[(j+1) % 4] - obj_corners[j]) << endl;
        //        }
        
        // 判断点是否是有效点
        bool exist = 1;
        std::vector<Point2f> scene_corners(4);
        perspectiveTransform( obj_corners, scene_corners, H);
        //        for (int j = 0; j < 4; j++) {
        //            cout<< "点 scene_corners[" << j << "] = " << scene_corners[j].x << " " << scene_corners[j].y << endl;
        //        }
        for (int j = 0; j < 4; j++) {
            //            cout<< "长度 scene_corners " << j << " = " << cv::norm(scene_corners[(j+1) % 4] - scene_corners[j]) << endl;
            if (cv::norm(scene_corners[(j+1) % 4] - scene_corners[j]) == 0) {
                exist = 0;
                break;
            }
        }
        
        //-- Draw lines between the corners (the mapped object in the scene - image_2 )
        Point2f offset( (float)img_object.cols, 0);
        line( img_matches, scene_corners[0] + offset, scene_corners[1] + offset, Scalar( 0, 255, 0), 4 );
        line( img_matches, scene_corners[1] + offset, scene_corners[2] + offset, Scalar( 0, 255, 0), 4 );
        line( img_matches, scene_corners[2] + offset, scene_corners[3] + offset, Scalar( 0, 255, 0), 4 );
        line( img_matches, scene_corners[3] + offset, scene_corners[0] + offset, Scalar( 0, 255, 0), 4 );
        
        _imageView_result.image = [UIImage imageWithCVMat:img_matches];
        
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
        
        gettimeofday(&t2, NULL);
        elapsedTime = (t2.tv_sec - taskStartTime.tv_sec) * 1000.0;
        elapsedTime += (t2.tv_usec - taskStartTime.tv_usec) / 1000.0;
        std::cout << "扫描第 " << i << " 张耗时 " << elapsedTime << endl << endl;
        
        if (exist) {
            break;
        }
    }
    
    gettimeofday(&t2, NULL);
    elapsedTime = (t2.tv_sec - tasksStartTime.tv_sec) * 1000.0;
    elapsedTime += (t2.tv_usec - tasksStartTime.tv_usec) / 1000.0;
    std::cout << "总耗时 " << elapsedTime << endl;
    if (i <= count) {
        std::cout << "这是第 " << i << " 张\n\n";
        resultLabel.text = @"V";
    } else {
        std::cout << "没有对应图片\n\n";
        resultLabel.text = @"X";
    }
    timeLabel.text = [NSString stringWithFormat:@"%f", elapsedTime];
}

#pragma mark - SurfFeatureDetector SurfDescriptorExtractor FlannBasedMatcher
- (void)detectImage_3:(cv::Mat&)mat {
    std::cout << "开始\n";
    
    timeval tasksStartTime, taskStartTime, t1, t2;
    double elapsedTime;
    gettimeofday(&tasksStartTime, NULL);
    
    cv::Mat img_scene = mat;
    NSString *filePath = nil;
    int i = currentIndex_obj;
    int count = currentIndex_obj;
    
    for (; i <= count; i++) {
        std::cout << "扫描第 " << i << " 张" << endl;
        
        gettimeofday(&taskStartTime, NULL);
        
        filePath = [[NSBundle mainBundle] pathForResource:[NSString stringWithFormat:@"%d", i] ofType:@"jpg"];
        cv::Mat img_object = cv::imread([filePath UTF8String], CV_LOAD_IMAGE_GRAYSCALE);
        if( !img_object.data || !img_scene.data ) {
            std::cout << "没有数据\n";
            return;
        }
        
        gettimeofday(&t1, NULL);
        //-- Step 1: Detect the keypoints using SURF Detector
        int minHessian = 1600; //最小 400，命中率高，速度慢
        SurfFeatureDetector detector( minHessian );
        
        std::vector<KeyPoint> keypoints_object, keypoints_scene;
        detector.detect( img_object, keypoints_object );
        detector.detect( img_scene, keypoints_scene );
        //        cout<< "keypoints_object.size() " << keypoints_object.size() << endl;
        //        cout<< "keypoints_scene.size() " << keypoints_scene.size() << endl;
        if (keypoints_object.size() == 0 || keypoints_scene.size() == 0) {
            continue;
        }
        gettimeofday(&t2, NULL);
        elapsedTime = (t2.tv_sec - t1.tv_sec) * 1000.0;
        elapsedTime += (t2.tv_usec - t1.tv_usec) / 1000.0;
        std::cout<< "获取关键点耗时 " << elapsedTime << endl;
        
        gettimeofday(&t1, NULL);
        //-- Step 2: Calculate descriptors (feature vectors)
        SurfDescriptorExtractor extractor;
        Mat descriptors_object, descriptors_scene;
        extractor.compute( img_object, keypoints_object, descriptors_object );
        extractor.compute( img_scene, keypoints_scene, descriptors_scene );
        gettimeofday(&t2, NULL);
        elapsedTime = (t2.tv_sec - t1.tv_sec) * 1000.0;
        elapsedTime += (t2.tv_usec - t1.tv_usec) / 1000.0;
        std::cout<< "计算关键点的描述对象耗时 " << elapsedTime << endl;
        
        gettimeofday(&t1, NULL);
        //-- Step 3: Matching descriptor vectors using FLANN matcher
        FlannBasedMatcher matcher;
        std::vector< DMatch > matches;
        matcher.match( descriptors_object, descriptors_scene, matches );
        //        cout<< "matches.size() " << matches.size() << endl;
        if (matches.size() == 0) {
            continue;
        }
        gettimeofday(&t2, NULL);
        elapsedTime = (t2.tv_sec - t1.tv_sec) * 1000.0;
        elapsedTime += (t2.tv_usec - t1.tv_usec) / 1000.0;
        std::cout<< "获取 matches 耗时 " << elapsedTime << endl;
        
        //-- Quick calculation of max and min distances between keypoints
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
        //        printf("-- Max dist : %f \n", max_dist );
        //        printf("-- Min dist : %f \n", min_dist );
        
        //-- Draw only "good" matches (i.e. whose distance is less than 3*min_dist )
        std::vector< DMatch > good_matches;
        for( int i = 0; i < descriptors_object.rows; i++ ) {
            if( matches[i].distance < 3*min_dist ) {
                good_matches.push_back( matches[i]);
            }
        }
        //        cout<< "good_matches.size() " << good_matches.size() << endl;
        if (good_matches.size() == 0) {
            continue;
        }
        
        // 画关键点
        Mat img_matches;
        drawMatches( img_object, keypoints_object, img_scene, keypoints_scene,
                    good_matches, img_matches, Scalar::all(-1), Scalar::all(-1),
                    vector<char>(), DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS );
        
        //-- Localize the object from img_1 in img_2
        std::vector<Point2f> obj;
        std::vector<Point2f> scene;
        for( size_t i = 0; i < good_matches.size(); i++ )
        {
            //-- Get the keypoints from the good matches
            obj.push_back( keypoints_object[ good_matches[i].queryIdx ].pt );
            scene.push_back( keypoints_scene[ good_matches[i].trainIdx ].pt );
        }
        
        gettimeofday(&t1, NULL);
        Mat H = findHomography( obj, scene, CV_RANSAC );
        gettimeofday(&t2, NULL);
        elapsedTime = (t2.tv_sec - t1.tv_sec) * 1000.0;
        elapsedTime += (t2.tv_usec - t1.tv_usec) / 1000.0;
        std::cout<< "findHomography 耗时 " << elapsedTime << endl;
        
        //-- Get the corners from the image_1 ( the object to be "detected" )
        std::vector<Point2f> obj_corners(4);
        obj_corners[0] = cvPoint(0,0);
        obj_corners[1] = cvPoint( img_object.cols, 0 );
        obj_corners[2] = cvPoint( img_object.cols, img_object.rows );
        obj_corners[3] = cvPoint( 0, img_object.rows );
        //        for (int j = 0; j < 4; j++) {
        //            cout<< "obj_corners[" << j << "] = " << obj_corners[j].x << " " << obj_corners[j].y << endl;
        //        }
        //        for (int j = 0; j < 4; j++) {
        //            cout<< "长度 obj_corners " << j << " = " << cv::norm(obj_corners[(j+1) % 4] - obj_corners[j]) << endl;
        //        }
        
        // 判断点是否是有效点
        bool exist = 1;
        std::vector<Point2f> scene_corners(4);
        perspectiveTransform( obj_corners, scene_corners, H);
        //        for (int j = 0; j < 4; j++) {
        //            cout<< "点 scene_corners[" << j << "] = " << scene_corners[j].x << " " << scene_corners[j].y << endl;
        //        }
        for (int j = 0; j < 4; j++) {
            //            cout<< "长度 scene_corners " << j << " = " << cv::norm(scene_corners[(j+1) % 4] - scene_corners[j]) << endl;
            if (cv::norm(scene_corners[(j+1) % 4] - scene_corners[j]) == 0) {
                exist = 0;
                break;
            }
        }
        
        //-- Draw lines between the corners (the mapped object in the scene - image_2 )
        Point2f offset( (float)img_object.cols, 0);
        line( img_matches, scene_corners[0] + offset, scene_corners[1] + offset, Scalar( 0, 255, 0), 4 );
        line( img_matches, scene_corners[1] + offset, scene_corners[2] + offset, Scalar( 0, 255, 0), 4 );
        line( img_matches, scene_corners[2] + offset, scene_corners[3] + offset, Scalar( 0, 255, 0), 4 );
        line( img_matches, scene_corners[3] + offset, scene_corners[0] + offset, Scalar( 0, 255, 0), 4 );
        
        _imageView_result.image = [UIImage imageWithCVMat:img_matches];
        
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
        
        gettimeofday(&t2, NULL);
        elapsedTime = (t2.tv_sec - taskStartTime.tv_sec) * 1000.0;
        elapsedTime += (t2.tv_usec - taskStartTime.tv_usec) / 1000.0;
        std::cout << "扫描第 " << i << " 张耗时 " << elapsedTime << endl << endl;
        
        if (exist) {
            break;
        }
    }
    
    gettimeofday(&t2, NULL);
    elapsedTime = (t2.tv_sec - tasksStartTime.tv_sec) * 1000.0;
    elapsedTime += (t2.tv_usec - tasksStartTime.tv_usec) / 1000.0;
    std::cout << "总耗时 " << elapsedTime << endl;
    if (i <= count) {
        std::cout << "这是第 " << i << " 张\n\n";
        resultLabel.text = @"V";
    } else {
        std::cout << "没有对应图片\n\n";
        resultLabel.text = @"X";
    }
    timeLabel.text = [NSString stringWithFormat:@"%f", elapsedTime];
}

#pragma mark - SurfFeatureDetector SurfDescriptorExtractor BruteForceMatcher
- (void)detectImage_4:(cv::Mat&)mat {
    std::cout << "开始\n";
    
    timeval tasksStartTime, taskStartTime, t1, t2;
    double elapsedTime;
    gettimeofday(&tasksStartTime, NULL);
    
    cv::Mat img_scene = mat;
    NSString *filePath = nil;
    int i = currentIndex_obj;
    int count = currentIndex_obj;
    
    for (; i <= count; i++) {
        std::cout << "扫描第 " << i << " 张" << endl;
        
        gettimeofday(&taskStartTime, NULL);
        
        filePath = [[NSBundle mainBundle] pathForResource:[NSString stringWithFormat:@"%d", i] ofType:@"jpg"];
        cv::Mat img_object = cv::imread([filePath UTF8String], CV_LOAD_IMAGE_GRAYSCALE);
        if( !img_object.data || !img_scene.data ) {
            std::cout << "没有数据\n";
            return;
        }
        
        gettimeofday(&t1, NULL);
        //-- Step 1: Detect the keypoints using SURF Detector
        int minHessian = 1600; //最小 400，命中率高，速度慢
        SurfFeatureDetector detector( minHessian );
        
        std::vector<KeyPoint> keypoints_object, keypoints_scene;
        detector.detect( img_object, keypoints_object );
        detector.detect( img_scene, keypoints_scene );
        //        cout<< "keypoints_object.size() " << keypoints_object.size() << endl;
        //        cout<< "keypoints_scene.size() " << keypoints_scene.size() << endl;
        if (keypoints_object.size() == 0 || keypoints_scene.size() == 0) {
            continue;
        }
        gettimeofday(&t2, NULL);
        elapsedTime = (t2.tv_sec - t1.tv_sec) * 1000.0;
        elapsedTime += (t2.tv_usec - t1.tv_usec) / 1000.0;
        std::cout<< "获取关键点耗时 " << elapsedTime << endl;
        
        gettimeofday(&t1, NULL);
        //-- Step 2: Calculate descriptors (feature vectors)
        SurfDescriptorExtractor extractor;
        Mat descriptors_object, descriptors_scene;
        extractor.compute( img_object, keypoints_object, descriptors_object );
        extractor.compute( img_scene, keypoints_scene, descriptors_scene );
        gettimeofday(&t2, NULL);
        elapsedTime = (t2.tv_sec - t1.tv_sec) * 1000.0;
        elapsedTime += (t2.tv_usec - t1.tv_usec) / 1000.0;
        std::cout<< "计算关键点的描述对象耗时 " << elapsedTime << endl;
        
        gettimeofday(&t1, NULL);
        //-- Step 3: Matching descriptor vectors using FLANN matcher
        BruteForceMatcher< L2<float> > matcher;
        std::vector< DMatch > matches;
        matcher.match( descriptors_object, descriptors_scene, matches );
        //        cout<< "matches.size() " << matches.size() << endl;
        if (matches.size() == 0) {
            continue;
        }
        gettimeofday(&t2, NULL);
        elapsedTime = (t2.tv_sec - t1.tv_sec) * 1000.0;
        elapsedTime += (t2.tv_usec - t1.tv_usec) / 1000.0;
        std::cout<< "获取 matches 耗时 " << elapsedTime << endl;
        
        //-- Quick calculation of max and min distances between keypoints
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
        //        printf("-- Max dist : %f \n", max_dist );
        //        printf("-- Min dist : %f \n", min_dist );
        
        //-- Draw only "good" matches (i.e. whose distance is less than 3*min_dist )
        std::vector< DMatch > good_matches;
        for( int i = 0; i < descriptors_object.rows; i++ ) {
            if( matches[i].distance < 3*min_dist ) {
                good_matches.push_back( matches[i]);
            }
        }
        //        cout<< "good_matches.size() " << good_matches.size() << endl;
        if (good_matches.size() == 0) {
            continue;
        }
        
        // 画关键点
        Mat img_matches;
        drawMatches( img_object, keypoints_object, img_scene, keypoints_scene,
                    good_matches, img_matches, Scalar::all(-1), Scalar::all(-1),
                    vector<char>(), DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS );
        
        //-- Localize the object from img_1 in img_2
        std::vector<Point2f> obj;
        std::vector<Point2f> scene;
        for( size_t i = 0; i < good_matches.size(); i++ )
        {
            //-- Get the keypoints from the good matches
            obj.push_back( keypoints_object[ good_matches[i].queryIdx ].pt );
            scene.push_back( keypoints_scene[ good_matches[i].trainIdx ].pt );
        }
        
        gettimeofday(&t1, NULL);
        Mat H = findHomography( obj, scene, CV_RANSAC );
        gettimeofday(&t2, NULL);
        elapsedTime = (t2.tv_sec - t1.tv_sec) * 1000.0;
        elapsedTime += (t2.tv_usec - t1.tv_usec) / 1000.0;
        std::cout<< "findHomography 耗时 " << elapsedTime << endl;
        
        //-- Get the corners from the image_1 ( the object to be "detected" )
        std::vector<Point2f> obj_corners(4);
        obj_corners[0] = cvPoint(0,0);
        obj_corners[1] = cvPoint( img_object.cols, 0 );
        obj_corners[2] = cvPoint( img_object.cols, img_object.rows );
        obj_corners[3] = cvPoint( 0, img_object.rows );
        //        for (int j = 0; j < 4; j++) {
        //            cout<< "obj_corners[" << j << "] = " << obj_corners[j].x << " " << obj_corners[j].y << endl;
        //        }
        //        for (int j = 0; j < 4; j++) {
        //            cout<< "长度 obj_corners " << j << " = " << cv::norm(obj_corners[(j+1) % 4] - obj_corners[j]) << endl;
        //        }
        
        // 判断点是否是有效点
        bool exist = 1;
        std::vector<Point2f> scene_corners(4);
        perspectiveTransform( obj_corners, scene_corners, H);
        //        for (int j = 0; j < 4; j++) {
        //            cout<< "点 scene_corners[" << j << "] = " << scene_corners[j].x << " " << scene_corners[j].y << endl;
        //        }
        for (int j = 0; j < 4; j++) {
            //            cout<< "长度 scene_corners " << j << " = " << cv::norm(scene_corners[(j+1) % 4] - scene_corners[j]) << endl;
            if (cv::norm(scene_corners[(j+1) % 4] - scene_corners[j]) == 0) {
                exist = 0;
                break;
            }
        }
        
        //-- Draw lines between the corners (the mapped object in the scene - image_2 )
        Point2f offset( (float)img_object.cols, 0);
        line( img_matches, scene_corners[0] + offset, scene_corners[1] + offset, Scalar( 0, 255, 0), 4 );
        line( img_matches, scene_corners[1] + offset, scene_corners[2] + offset, Scalar( 0, 255, 0), 4 );
        line( img_matches, scene_corners[2] + offset, scene_corners[3] + offset, Scalar( 0, 255, 0), 4 );
        line( img_matches, scene_corners[3] + offset, scene_corners[0] + offset, Scalar( 0, 255, 0), 4 );
        
        _imageView_result.image = [UIImage imageWithCVMat:img_matches];
        
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
        
        gettimeofday(&t2, NULL);
        elapsedTime = (t2.tv_sec - taskStartTime.tv_sec) * 1000.0;
        elapsedTime += (t2.tv_usec - taskStartTime.tv_usec) / 1000.0;
        std::cout << "扫描第 " << i << " 张耗时 " << elapsedTime << endl << endl;
        
        if (exist) {
            break;
        }
    }
    
    gettimeofday(&t2, NULL);
    elapsedTime = (t2.tv_sec - tasksStartTime.tv_sec) * 1000.0;
    elapsedTime += (t2.tv_usec - tasksStartTime.tv_usec) / 1000.0;
    std::cout << "总耗时 " << elapsedTime << endl;
    if (i <= count) {
        std::cout << "这是第 " << i << " 张\n\n";
        resultLabel.text = @"V";
    } else {
        std::cout << "没有对应图片\n\n";
        resultLabel.text = @"X";
    }
    timeLabel.text = [NSString stringWithFormat:@"%f", elapsedTime];
}

@end