//
//  MapAnnotation.h
//  MyDroneIOS
//
//  Created by kangzhiyong on 2020/4/2.
//  Copyright © 2020 kangzhiyong. All rights reserved.
//

#import <Foundation/Foundation.h>
#import <MapKit/MapKit.h>

NS_ASSUME_NONNULL_BEGIN

@interface MapAnnotation : NSObject<MKAnnotation>
@property (nonatomic, strong) NSString *title;
@property (nonatomic, readwrite) CLLocationCoordinate2D coordinate;
@property (nonatomic, copy) NSString *subTitle;

- (id)initWithTitle:(NSString *)title andCoordinate:
    (CLLocationCoordinate2D)coordinate2d;

@end

NS_ASSUME_NONNULL_END
