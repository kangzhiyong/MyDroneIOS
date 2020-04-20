//
//  MyDroneIOS.h
//  MyDroneIOS
//
//  Created by kangzhiyong on 2020/4/1.
//  Copyright © 2020 kangzhiyong. All rights reserved.
//

#import <Foundation/Foundation.h>
typedef void (*CallBackFunc)(void * data, double lo, double la, double alt);

NS_ASSUME_NONNULL_BEGIN

@interface MyDroneIOS : NSObject
    +(void) startMyDrone:(int) type;
    +(void) stopMyDrone;
    +(void) registerCallBackFunction:(void *)data andCallBack:(CallBackFunc) func;
    +(void) setAppPath:(const char *)path;
@end

NS_ASSUME_NONNULL_END
