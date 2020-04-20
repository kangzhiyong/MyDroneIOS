//
//  MapAnnotation.m
//  MyDroneIOS
//
//  Created by kangzhiyong on 2020/4/2.
//  Copyright © 2020 kangzhiyong. All rights reserved.
//

#import "MapAnnotation.h"

@implementation MapAnnotation
-(id)initWithTitle:(NSString *)title andCoordinate:(CLLocationCoordinate2D)coordinate2d {
    self.title = title;
    self.coordinate = coordinate2d;
    return self;
}
@end
