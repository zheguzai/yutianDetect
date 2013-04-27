//
//  ViewController.h
//  yutianDetect
//
//  Created by fred on 4/24/13.
//  Copyright (c) 2013 fred. All rights reserved.
//

#import <UIKit/UIKit.h>
#import <AVFoundation/AVFoundation.h>

@interface ViewController : UIViewController<AVCaptureVideoDataOutputSampleBufferDelegate, UITextFieldDelegate> {
    AVCaptureSession *_captureSession;
    AVCaptureDevice *_captureDevice;
    AVCaptureVideoDataOutput *_videoOutput;
    AVCaptureVideoPreviewLayer *_videoPreviewLayer;
    
    IBOutlet UIImageView *_imageView_result;
    IBOutlet UILabel *resultLabel;
    IBOutlet UILabel *timeLabel;
    IBOutlet UITextField *detectTypeTextField;
    
    IBOutlet UIScrollView *scrollView_obj;
    IBOutlet UIScrollView *scrollView_scene;
    
    int currentIndex_obj;
    int currentIndex_scene;
    int detectType;
}

@property (nonatomic, readonly) AVCaptureSession *captureSession;
@property (nonatomic, readonly) AVCaptureDevice *captureDevice;
@property (nonatomic, readonly) AVCaptureVideoDataOutput *videoOutput;
@property (nonatomic, readonly) AVCaptureVideoPreviewLayer *videoPreviewLayer;

- (IBAction)clickDetectBtn:(id)sender;

@end