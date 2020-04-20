//
//  ViewController.m
//  MyDroneIOS
//
//  Created by kangzhiyong on 2020/4/1.
//  Copyright © 2020 kangzhiyong. All rights reserved.
//

#import "ViewController.h"
#import "MapAnnotation.h"
#import <CoreLocation/CoreLocation.h>
#import <MapKit/MapKit.h>

@interface ViewController ()<CLLocationManagerDelegate>{
    CLLocationManager *_locationManager;
}
-(void)addAnnotation:(double)lo andla:(double)la andalt:(double) alt;
@end

void callBack(void *userData,double lo, double la, double alt)
{
    ViewController *vController = (__bridge ViewController *)(userData);
    [vController addAnnotation:lo andla:la andalt:alt];
}

@implementation ViewController
@synthesize mapView;

- (void)viewDidLoad {
    [super viewDidLoad];
    // Do any additional setup after loading the view.

//    CGRect rect = [UIScreen mainScreen].bounds;
//    [mapView setFrame:rect];
    
    _locationManager = [[CLLocationManager alloc]init];
    if (![CLLocationManager locationServicesEnabled])
    {
        NSLog(@"定位服务当前可能尚未打开，请设置打开！");
        return;
    }
    
    if ([CLLocationManager authorizationStatus] == kCLAuthorizationStatusNotDetermined)
    {
        [_locationManager requestWhenInUseAuthorization];
    }
    else if ([CLLocationManager authorizationStatus] == kCLAuthorizationStatusAuthorizedWhenInUse)
    {
        _locationManager.delegate = self;
        _locationManager.desiredAccuracy = kCLLocationAccuracyBest;
        CLLocationDistance distance = 1.0;
        _locationManager.distanceFilter = distance;
        [_locationManager startUpdatingLocation];
    }
    mapView.userTrackingMode = MKUserTrackingModeFollow;
    mapView.mapType = MKMapTypeStandard;
    NSString *newPath=[NSString stringWithFormat:@"%@",[[NSBundle mainBundle]resourcePath]];
    [MyDroneIOS setAppPath:newPath.fileSystemRepresentation];
    [MyDroneIOS registerCallBackFunction:(void *)self andCallBack: callBack];
}

-(void)locationManager:(CLLocationManager *)manager didUpdateLocations:(NSArray<CLLocation *> *)locations
{
    CLLocation *location = [locations firstObject];
    CLLocationCoordinate2D coordinate = location.coordinate;
    NSLog(@"经度： %f， 纬度：%f, 海拔：%f, 航向: %f, 行走速度: %f", coordinate.longitude, coordinate.latitude, location.altitude, location.course, location.speed);
    [_locationManager stopUpdatingLocation];
}
-(IBAction)FlyStartRectPlan
{
    [MyDroneIOS startMyDrone:0];
}

-(IBAction)FlyStartPathPlan
{
    [MyDroneIOS startMyDrone:1];
}

- (void)mapView:(MKMapView *)mv didAddAnnotationViews:(nonnull NSArray<MKAnnotationView *> *)views
{
    MKAnnotationView *annotationView = [views objectAtIndex:0];
    id <MKAnnotation> mp = [annotationView annotation];
    MKCoordinateRegion region = MKCoordinateRegionMakeWithDistance([mp coordinate], 1500, 1500);
    [mv setRegion:region animated:YES];
    [mv selectAnnotation:mp animated:YES];
}

-(void)addAnnotation:(double)lo andla:(double)la andalt:(double) alt {
    NSLog(@"Annotation: 经度：%f, 纬度：%f, 海拔：%f", lo, la, alt);
    CLLocationCoordinate2D location1=CLLocationCoordinate2DMake(la, lo);
    MapAnnotation *annotation1=[[MapAnnotation alloc]init];
    annotation1.title= [NSString stringWithFormat: @"经度：%f, 纬度：%f, 海拔：%f", lo, la, alt];
//    annotation1.subTitle = @"Kenshin Cui's Studios";
    annotation1.coordinate=location1;
//    annotation1.image=[UIImage imageNamed:@"icon_pin_floating.png"];
    [mapView addAnnotation:annotation1];
}
@end
