//
//  ViewController.h
//  MyDroneIOS
//
//  Created by kangzhiyong on 2020/4/1.
//  Copyright © 2020 kangzhiyong. All rights reserved.
//

#import <UIKit/UIKit.h>
#import <MapKit/MapKit.h>
#import "MyDroneIOS.h"

@interface ViewController : UIViewController {
    MKMapView *mapView;
    MyDroneIOS *myDrone;
}
@property(nonatomic, retain)IBOutlet MKMapView *mapView;
-(IBAction)FlyStartRectPlan;
-(IBAction)FlyStartPathPlan;
@end

