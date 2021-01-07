//
//  MyDroneIOS.m
//  MyDroneIOS
//
//  Created by kangzhiyong on 2020/4/1.
//  Copyright © 2020 kangzhiyong. All rights reserved.
//

#import "MyDroneIOS.h"
#import "MyDrone/flyer.hpp"

@implementation MyDroneIOS
    Flyer fly(new MavlinkConnection("127.0.0.1:14555", true, true));
    +(void) startMyDrone:(int) type  {
        fly.start(type);
    }
    +(void) armMyDrone {
        fly.arming_transition();
    }
    +(void) disarmMyDrone {
        fly.disarming_transition();
    }
    +(void) registerCallBackFunction:(void *)data andCallBack:(CallBackFunc) func {
        fly.registerCallBackFunction(data, func);
    }
    +(void) setAppPath:(const char *)path {
        fly.setAppPath(path);
    }
    +(void) setHomePosition:(double)lo andLa:(double) la andAlt:(double) alt {
        fly.set_home_position(lo, la, alt);
    }
@end
