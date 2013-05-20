//
//  ViewController.h
//  yutianDetect
//
//  Created by fred on 4/24/13.
//  Copyright (c) 2013 fred. All rights reserved.
//

#import <UIKit/UIKit.h>
#import <AVFoundation/AVFoundation.h>

@interface ViewController : UIViewController<AVCaptureVideoDataOutputSampleBufferDelegate> {
    AVCaptureSession *_captureSession;
    AVCaptureDevice *_captureDevice;
    AVCaptureVideoDataOutput *_videoOutput;
    AVCaptureVideoPreviewLayer *_videoPreviewLayer;
    
    IBOutlet UIImageView *_imageView_result;
    IBOutlet UIImageView *_imageView_object;
    IBOutlet UIImageView *_imageView_scene;
    IBOutlet UIButton *_restartBtn;
    
    IBOutlet UIView *_resultRectView;
}

@property (nonatomic, readonly) AVCaptureSession *captureSession;
@property (nonatomic, readonly) AVCaptureDevice *captureDevice;
@property (nonatomic, readonly) AVCaptureVideoDataOutput *videoOutput;
@property (nonatomic, readonly) AVCaptureVideoPreviewLayer *videoPreviewLayer;

@property (nonatomic, retain) NSNumber *detected;
@property (nonatomic, retain) NSNumber *lastMotionTime;

- (IBAction)onClickRestartBtn:(id)sender;

@end