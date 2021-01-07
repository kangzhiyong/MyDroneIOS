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
@property(nonatomic, strong) NSMutableArray *logs;
-(void)addAnnotation:(double)lo andla:(double)la andalt:(double) alt;
@end

void callBack(void *userData,double lo, double la, double alt)
{
    ViewController *vController = (__bridge ViewController *)(userData);
    [vController addAnnotation:lo andla:la andalt:alt];
}

@implementation ViewController
@synthesize mapView;
@synthesize logTextView;
//@synthesize logTableView;
//@synthesize logs;

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
//    logs = [[NSMutableArray alloc]init];
    mapView.userTrackingMode = MKUserTrackingModeFollow;
    mapView.mapType = MKMapTypeStandard;
    NSString *newPath=[NSString stringWithFormat:@"%@",[[NSBundle mainBundle]resourcePath]];
    [MyDroneIOS setAppPath:newPath.fileSystemRepresentation];
    [MyDroneIOS registerCallBackFunction:(void *)self andCallBack: callBack];
//    [self redirectSTD:STDOUT_FILENO];
//    [self redirectSTD:STDERR_FILENO];
}

-(void)locationManager:(CLLocationManager *)manager didUpdateLocations:(NSArray<CLLocation *> *)locations
{
    CLLocation *location = [locations firstObject];
    CLLocationCoordinate2D coordinate = location.coordinate;
//    [MyDroneIOS setHomePosition:coordinate.longitude andLa:coordinate.latitude andAlt:location.altitude];
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

-(IBAction)FlyDisarm
{
    [MyDroneIOS disarmMyDrone];
}

-(IBAction)FlyArm
{
    [MyDroneIOS armMyDrone];
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
//    NSLog(@"Annotation: 经度：%f, 纬度：%f, 海拔：%f", lo, la, alt);
    CLLocationCoordinate2D location1=CLLocationCoordinate2DMake(la, lo);
    MapAnnotation *annotation1=[[MapAnnotation alloc]init];
    annotation1.title= [NSString stringWithFormat: @"经度：%f, 纬度：%f, 海拔：%f", lo, la, alt];
//    annotation1.subTitle = @"Kenshin Cui's Studios";
    annotation1.coordinate=location1;
//    annotation1.image=[UIImage imageNamed:@"icon_pin_floating.png"];
    [mapView addAnnotation:annotation1];
}

- (void)redirectNotificationHandle:(NSNotification *)nf{ // 通知方法
    NSData *data = [[nf userInfo] objectForKey:NSFileHandleNotificationDataItem];
    NSString *str = [[NSString alloc] initWithData:data encoding:NSUTF8StringEncoding];

    self.logTextView.text = [NSString stringWithFormat:@"%@\n%@",self.logTextView.text, str];// logTextView 就是要将日志输出的视图（UITextView）
    NSRange range;
    range.location = [self.logTextView.text length] - 1;
    range.length = 0;
    [self.logTextView scrollRangeToVisible:range];
    [[nf object] readInBackgroundAndNotify];
    
//    [self.logTableView beginUpdates];
//    [logs addObject:str];
//    NSIndexPath *indexPathOfNewItem = [NSIndexPath indexPathForItem:logs.count - 1 inSection:0];
//    [self.logTableView insertRowsAtIndexPaths:@[indexPathOfNewItem] withRowAnimation:(UITableViewRowAnimationAutomatic)];
//    [self.logTableView endUpdates];
//    [self.logTableView scrollToRowAtIndexPath:indexPathOfNewItem atScrollPosition:(UITableViewScrollPositionBottom) animated:YES];
}

- (void)redirectSTD:(int )fd{
    NSPipe * pipe = [NSPipe pipe] ;// 初始化一个NSPipe 对象
    NSFileHandle *pipeReadHandle = [pipe fileHandleForReading] ;
    dup2([[pipe fileHandleForWriting] fileDescriptor], fd) ;

    [[NSNotificationCenter defaultCenter] addObserver:self
                                             selector:@selector(redirectNotificationHandle:)
                                                 name:NSFileHandleReadCompletionNotification
                                               object:pipeReadHandle]; // 注册通知
    [pipeReadHandle readInBackgroundAndNotify];
}

//#pragma mark -- delegate方法
//- (NSInteger)numberOfSectionsInTableView:(UITableView *)tableView
//{
//    return 1;
//}
//
//- (NSInteger)tableView:(UITableView *)tableView numberOfRowsInSection:(NSInteger)section
//{
//    return [logs count];
//}
//
//- (UITableViewCell *)tableView:(UITableView *)tableView cellForRowAtIndexPath:(NSIndexPath *)indexPath
//{
//    static NSString *indentifier = @"cell";
//    UITableViewCell *cell = [tableView dequeueReusableCellWithIdentifier:indentifier];
//
//    if (!cell) {
//        cell = [[UITableViewCell alloc] initWithStyle:UITableViewCellStyleSubtitle reuseIdentifier:indentifier];
//    }
//
//    cell.textLabel.text = [logs objectAtIndex:indexPath.row];
//
//    return cell;
//}
//
//- (void)didReceiveMemoryWarning {
//    [super didReceiveMemoryWarning];
//    // Dispose of any resources that can be recreated.
//}
@end
