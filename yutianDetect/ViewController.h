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
    IBOutlet UIImageView *_imageView_obj;
    IBOutlet UIImageView *_imageView_scene;
}

@property (nonatomic, readonly) AVCaptureSession *captureSession;
@property (nonatomic, readonly) AVCaptureDevice *captureDevice;
@property (nonatomic, readonly) AVCaptureVideoDataOutput *videoOutput;
@property (nonatomic, readonly) AVCaptureVideoPreviewLayer *videoPreviewLayer;

@end